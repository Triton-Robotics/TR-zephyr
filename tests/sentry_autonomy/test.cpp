#include "platform.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <util/communications/jetson/Jetson.h>

// Include the real robot application, replacing only its hardware dependencies.
#define main sentry_firmware_main
#include "../../robots/Sentry/src/sentry.cpp"
#undef main

static bool close(float actual, float expected) {
    return std::fabs(actual - expected) < 0.0001f;
}

// Construct wire bytes independently of the production serializer/checksum.
static std::vector<char> packet(char header, std::initializer_list<float> values, char flag) {
    std::vector<char> bytes{header};
    for (float value : values) {
        const auto *raw = reinterpret_cast<const char *>(&value);
        bytes.insert(bytes.end(), raw, raw + sizeof(value));
    }
    bytes.push_back(flag);
    uint8_t sum = 0;
    for (size_t i = 1; i < bytes.size(); ++i) sum += static_cast<uint8_t>(bytes[i]);
    bytes.push_back(static_cast<char>(-sum));
    return bytes;
}

static void receive(std::initializer_list<std::vector<char>> chunks) {
    for (const auto &chunk : chunks) uart_input.push_back(chunk);
    try {
        threads.at(1)(); // Run the real Jetson RX worker until the fake UART empties.
    } catch (const StopThread &) {}
    assert(uart_input.empty());
}

static void assert_stopped(Sentry &sentry) {
    assert(sentry.chassis_.desired.vX == 0);
    assert(sentry.chassis_.desired.vY == 0);
    assert(sentry.chassis_.desired.vOmega == 0);
    assert(sentry.chassis_.power.LF == 0 && sentry.chassis_.power.RF == 0);
    assert(sentry.chassis_.power.LB == 0 && sentry.chassis_.power.RB == 0);
    assert(sentry.turret_.desired.turret_mode == SLEEP);
    assert(sentry.shooter_.desired == OFF);
}

static void test_decoder() {
    TurretPacket turret;
    ChassisReadPacket chassis;
    Jetson::ReadState state{};
    assert(turret.parse_buff(nullptr, 0, state) == -1);
    auto drive = packet(0xDD, {1.0f, 2.0f, 0.5f}, 1);
    assert(drive.size() == 15);
    // Known IEEE-754 little-endian payload and LRC.
    assert(static_cast<uint8_t>(drive[3]) == 0x80);
    assert(static_cast<uint8_t>(drive[4]) == 0x3F);
    assert(static_cast<uint8_t>(drive.back()) == 0xC1);
    assert(chassis.parse_buff(drive.data(), drive.size(), state) == 15);
    assert(state.desired_x_vel == 1 && state.desired_y_vel == 2);
    assert(state.desired_angular_vel == 0.5f && state.localization_calibration == 1);
    assert(state.chassis_stamp_us == test_time_us && state.turret_stamp_us == 0);

    test_time_us += 100;
    auto aim = packet(0xCC, {0.25f, -0.125f}, 1);
    assert(turret.parse_buff(aim.data(), aim.size(), state) == 11);
    assert(state.desired_yaw_rads == 0.25f && state.desired_pitch_rads == -0.125f);
    assert(state.shoot_status == 1);
    assert(state.turret_stamp_us == test_time_us);
    assert(state.chassis_stamp_us == test_time_us - 100);
    const auto good_state = state;

    test_time_us += 100;
    for (int length = 0; length < static_cast<int>(aim.size()); ++length) {
        assert(turret.parse_buff(aim.data(), length, state) == -1);
    }
    for (int length = 0; length < static_cast<int>(drive.size()); ++length) {
        assert(chassis.parse_buff(drive.data(), length, state) == -1);
    }
    aim.back() ^= 1;
    assert(turret.parse_buff(aim.data(), aim.size(), state) == -1);
    drive[0] = 0;
    assert(chassis.parse_buff(drive.data(), drive.size(), state) == -1);

    for (float invalid : {std::numeric_limits<float>::infinity(),
                          -std::numeric_limits<float>::infinity(),
                          std::numeric_limits<float>::quiet_NaN()}) {
        for (int field = 0; field < 3; ++field) {
            float values[3] = {1, 2, 3};
            values[field] = invalid;
            auto bad = packet(0xDD, {values[0], values[1], values[2]}, 0);
            assert(chassis.parse_buff(bad.data(), bad.size(), state) == -1);
        }
        for (int field = 0; field < 2; ++field) {
            auto bad = packet(0xCC, {field == 0 ? invalid : 0,
                                     field == 1 ? invalid : 0}, 0);
            assert(turret.parse_buff(bad.data(), bad.size(), state) == -1);
        }
    }
    assert(state.stamp_us == good_state.stamp_us);
    assert(state.chassis_stamp_us == good_state.chassis_stamp_us);
    assert(state.turret_stamp_us == good_state.turret_stamp_us);
    assert(state.desired_x_vel == good_state.desired_x_vel);
    assert(state.desired_yaw_rads == good_state.desired_yaw_rads);
    assert(state.shoot_status == good_state.shoot_status);
}

static void test_autonomy() {
    BaseRobot::Config config{};
    Sentry sentry(config);
    test_time_us = 1000;
    assert(sentry.jetson.read().stamp_us == 0);
    sentry.periodic(2000);
    assert_stopped(sentry);
    assert(sentry.stm_state.activate_CV == 1);
    assert(sentry.chassis_.power_at_command == 80);

    test_time_us = 2000;
    auto drive = packet(0xDD, {0.4f, 0.3f, 0.2f}, 0);
    // Exercise every possible two-chunk split through the actual RX buffer.
    for (size_t split = 1; split < drive.size(); ++split) {
        receive({{drive.begin(), drive.begin() + split},
                 {drive.begin() + split, drive.end()}});
        sentry.periodic(2000);
        assert(close(sentry.chassis_.desired.vX, 0.4f));
        assert(close(sentry.chassis_.desired.vY, -0.3f));
        assert(close(sentry.chassis_.desired.vOmega, 0.2f));
        assert(sentry.chassis_.mode == ChassisSubsystem::ROBOT_ORIENTED);
        assert(sentry.turret_.desired.turret_mode == SLEEP);
        assert(sentry.shooter_.desired == OFF);
    }

    test_time_us = 3000;
    auto aim = packet(0xCC, {PI / 2, 0.1f}, 1);
    receive({aim});
    // Nonzero remote inputs and all remote drive modes must leave autonomy intact.
    sentry.jx = sentry.jy = sentry.myaw = sentry.mpitch = sentry.jyaw = sentry.jpitch = 1;
    sentry.shot = 'd';
    sentry.cv_enabled_ = false;
    for (char mode : {'o', 'u', 'd', 'm', 'y'}) {
        sentry.drive = mode;
        sentry.periodic(2000);
        assert(close(sentry.chassis_.desired.vX, 0.4f));
        assert(close(sentry.turret_.desired.yaw_angle_degs, 90));
        assert(close(sentry.turret_.desired.pitch_angle_degs, radiansToDegrees(-0.1f)));
        assert(sentry.turret_.desired.turret_mode == AIM);
        assert(sentry.shooter_.desired == SHOOT);
    }

    test_time_us = 502000; // Chassis expires exactly at 500 ms; turret remains fresh.
    sentry.periodic(2000);
    assert(sentry.chassis_.desired.vX == 0 && sentry.chassis_.power.LF == 0);
    assert(sentry.turret_.desired.turret_mode == AIM && sentry.shooter_.desired == SHOOT);
    test_time_us = 503000;
    sentry.periodic(2000);
    assert_stopped(sentry);
    assert(!sentry.referee_.is_cv_on && !sentry.referee_.is_flywheel_on);

    test_time_us = 600000;
    receive({drive});
    sentry.periodic(2000);
    assert(sentry.chassis_.desired.vX != 0);
    assert(sentry.turret_.desired.turret_mode == SLEEP && sentry.shooter_.desired == OFF);
    test_time_us = 601000;
    receive({packet(0xCC, {9.0f * PI, -2.0f}, 0)});
    sentry.periodic(2000);
    assert(sentry.turret_.desired.turret_mode == AIM && sentry.shooter_.desired == OFF);
    assert(close(sentry.turret_.desired.pitch_angle_degs, 25));
    assert(std::fabs(sentry.turret_.desired.yaw_angle_degs) <= 180);

    test_time_us = 1100000;
    auto corrupt = drive;
    corrupt.back() ^= 1;
    receive({corrupt, std::vector<char>(100, 0)});
    sentry.periodic(2000);
    assert(sentry.jetson_state.chassis_stamp_us == 600000);
    assert(sentry.chassis_.desired.vX == 0);
    // A new turret/shoot packet must not reactivate the expired chassis command.
    receive({aim});
    sentry.periodic(2000);
    assert(sentry.chassis_.desired.vX == 0 && sentry.chassis_.power.LF == 0);
    assert(sentry.shooter_.desired == SHOOT);

    test_time_us += 100;
    receive({packet(0xDD, {1000, -1000, 1000}, 0), packet(0xCC, {0, 2}, 0)});
    sentry.referee_.robot_status.chassis_power_limit = 60;
    sentry.periodic(2000);
    assert(sentry.chassis_.desired.vX == MAX_VEL && sentry.chassis_.desired.vY == MAX_VEL);
    assert(sentry.chassis_.desired.vOmega == 8);
    assert(sentry.chassis_.power_at_command == 60);
    assert(close(sentry.turret_.desired.pitch_angle_degs, -22));
    assert(sentry.shooter_.desired == OFF);
    test_time_us += 500000;
    sentry.periodic(2000);
    assert_stopped(sentry);
}

int main() {
    test_decoder();
    test_autonomy();
    std::cout << "PASS: packet decoding, invalid/partial packets, independent timeouts, "
                 "remote independence, command limits, shooter control, and recovery\n";
}
