#pragma once
// Hardware substitutes for exercising the production decoder and Sentry loop on a host.
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <vector>

struct device {};
struct spi_dt_spec {};
struct gpio_dt_spec {};
struct i2c_dt_spec {};
struct pwm_dt_spec {};
inline device test_device;
#define DEVICE_DT_GET(...) (&test_device)
#define DT_NODELABEL(...) 0
#define DT_ALIAS(...) 0
#define GPIO_DT_SPEC_GET(...) gpio_dt_spec{}
#define I2C_DT_SPEC_GET(...) i2c_dt_spec{}
#define PWM_DT_SPEC_GET(...) pwm_dt_spec{}
#define USART3 nullptr
inline bool device_is_ready(const device *) { return true; }

inline uint64_t test_time_us = 1000;
inline std::deque<std::vector<char>> uart_input;
inline std::vector<std::function<void()>> threads;
struct StopThread {};
struct k_thread {};
#define K_THREAD_STACK_DEFINE(name, size) char name[size]
#define K_NO_WAIT 0
#define K_MSEC(ms) (ms)
inline void k_thread_create(k_thread *, char *, int,
                           void (*entry)(void *, void *, void *),
                           void *p1, void *p2, void *p3, int, int, int) {
    threads.emplace_back([=] { entry(p1, p2, p3); });
}
inline void k_yield() {
    if (uart_input.empty()) throw StopThread{};
}
inline void k_sleep(int) { throw StopThread{}; }
inline uint64_t k_uptime_ticks() { return test_time_us; }
inline uint64_t k_ticks_to_us_floor64(uint64_t ticks) { return ticks; }
class Mutex {
  public:
    void lock() {}
    void unlock() {}
};
class SerialBase {
  public:
    SerialBase(const device *, void *) {}
    bool readable() { return !uart_input.empty(); }
    int read(char *buffer, int size) {
        auto &chunk = uart_input.front();
        const int count = std::min(size, static_cast<int>(chunk.size()));
        std::memcpy(buffer, chunk.data(), count);
        chunk.erase(chunk.begin(), chunk.begin() + count);
        if (chunk.empty()) uart_input.pop_front();
        return count;
    }
    int write(char *, int size) { return size; }
};

class PID {
  public:
    struct config { float kp, ki, kd, output_cap = 0, integral_cap = 0; };
};
class CANHandler {
  public:
    enum CANBus { CANBUS_1, CANBUS_2 };
};
enum motorType { M3508 };
class IMU {
  public:
    struct EulerAngles { float yaw = 0, pitch = 0, roll = 0; };
};
class ISM330 : public IMU {
  public:
    ISM330(i2c_dt_spec) {}
    void begin(float, int) {}
    void mahonyUpdateIMU(float) {}
    EulerAngles getImuAngles() { return {}; }
};
class MA4 {
  public:
    MA4(const pwm_dt_spec *) {}
};
struct ChassisSpeeds { float vX, vY, vOmega; };
struct WheelSpeeds { float LF, RF, LB, RB; };
#define MAX_VEL 2.92f
class ChassisSubsystem {
  public:
    struct Config {
        int lf, rf, lb, rb;
        float radius, ff, offset;
        IMU &imu;
        MA4 *encoder;
    };
    enum DRIVE_MODE { YAW_ORIENTED, ROBOT_ORIENTED };
    ChassisSpeeds desired{};
    WheelSpeeds power{};
    DRIVE_MODE mode = YAW_ORIENTED;
    float power_limit = 0, power_at_command = 0;
    ChassisSubsystem(Config) {}
    void setChassisSpeeds(ChassisSpeeds command, DRIVE_MODE drive_mode) {
        desired = command;
        mode = drive_mode;
        power_at_command = power_limit;
        power = {1, 1, 1, 1}; // Detect a missing explicit timeout stop.
    }
    void setWheelPower(WheelSpeeds command) { power = command; }
    ChassisSpeeds getChassisSpeeds() { return {}; }
    void periodic(IMU::EulerAngles *) {}
};
enum TurretState { SLEEP, AIM };
class TurretSubsystem {
  public:
    struct config {
        const device *yaw_dev;
        CANHandler::CANBus yaw_bus;
        short yaw_id;
        motorType yaw_type;
        const device *pitch_dev;
        CANHandler::CANBus pitch_bus;
        short pitch_id;
        motorType pitch_type;
        PID::config yaw_vel, yaw_pos, pitch_vel, pitch_pos;
        float yaw_static, yaw_kinetic, pitch_gravity, pitch_static, pitch_kinetic;
        int forward;
        float gear_ratio, lower, upper;
    };
    struct TurretInfo {
        float yaw_angle_degs = 0, yaw_velo_rad_s = 0;
        float pitch_angle_degs = 0, pitch_velo_rad_s = 0;
        TurretState turret_mode = SLEEP;
    };
    TurretInfo desired{};
    TurretSubsystem(config, IMU &) {}
    void setState(TurretInfo command) { desired = command; }
    TurretInfo getState() { return {}; }
    void periodic(float) {}
};
enum ShootState { OFF, FLYWHEEL, SHOOT, JAM };
class ShooterSubsystem {
  public:
    enum ShooterType { BURST, AUTO };
    struct config {
        const device *dev;
        CANHandler::CANBus bus;
        ShooterType type;
        int heat_limit;
        short left, right, indexer;
        PID::config left_pid, right_pid, indexer_vel, indexer_pos;
        bool invert;
    };
    ShootState desired = OFF;
    ShooterSubsystem(config) {}
    void setState(ShootState command) { desired = command; }
    void periodic(int, int) {}
};
class DJIRemote2 {};
class BaseRobot {
  public:
    struct Config {
        const gpio_dt_spec *led0_dev, *led1_dev, *led2_dev;
        const device *controller_uart_dev, *referee_uart_dev, *can1_dev, *can2_dev;
        void *referee_usart_type;
    };
    struct {
        struct { int chassis_power_limit = 0, shooter_barrel_heat_limit = 0; } robot_status;
        struct { int shooter_17mm_1_barrel_heat = 0; } power_heat_data;
        bool is_aligned = false, is_cv_on = false, is_spinning = false, is_flywheel_on = false;
        int get_game_progress() { return 0; }
        int get_remain_hp() { return 0; }
        int is_red_or_blue() { return 0; }
    } referee_;
    float jx = 0, jy = 0, myaw = 0, mpitch = 0, jyaw = 0, jpitch = 0;
    char drive = 'o', shot = 'o';
    bool cv_enabled_ = false;
    BaseRobot(const Config &) {}
    virtual void init() = 0;
    virtual void periodic(unsigned long) = 0;
    virtual unsigned int main_loop_dt_ms() { return 1; }
    void main_loop() {}
};
