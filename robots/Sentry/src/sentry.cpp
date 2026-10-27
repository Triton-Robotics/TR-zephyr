#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>
#include <algorithm>
#include <cmath>
#include <util/communications/DJIRemote2.h>
#include <util/communications/CANHandler.h>
#include <util/communications/jetson/Jetson.h>
#include <util/peripherals/imu/ISM330.h>
#include <util/algorithms/PID.h>
#include <base_robot/BaseRobot.h>
#include <zephyr/dt-bindings/pwm/pwm.h>


// Robot Constants
constexpr float PITCH_LOWER_BOUND{-22.0};
constexpr float PITCH_UPPER_BOUND{25.0};

// Remote (infantry-style) control sensitivities
constexpr float JOYSTICK_YAW_SENSITIVITY_DPS = 300;
constexpr float JOYSTICK_PITCH_SENSITIVITY_DPS = 150;

// Jetson (autonomy) limits
constexpr uint64_t JETSON_COMMAND_TIMEOUT_US = 500000;
constexpr float JETSON_MAX_ANGULAR_VEL = 8.0f;

constexpr PID::config YAW_VEL_PID     = {181,0,10, 32000, 1000};
constexpr PID::config YAW_POS_PID     = {1, 0, 0, 45, 2};
const float yaw_static_friction       = 0;       // We multiply it by dir
const float yaw_kinetic_friction      = 0;       // We multiply this by yawvelo

constexpr PID::config PITCH_VEL_PID   = {173.8994f, 4.898f * static_cast<float>(10e-6), 12.474f * static_cast<float>(10e3) * 1.5f, 16000, 2000}; //{25, 0.001, 5, 16000, 1000};
constexpr PID::config PITCH_POS_PID   = {1, 0, 0,30,2}; //{1, 0, 0, 30, 2};
const float pitch_gravity_feedforward = -1200;    // We multiply this by cos(angle)
const float pitch_static_friction     = 0;       // We multiply it by dir
const float pitch_kinetic_friction    = 0; //5.5;     // We multiply this by pitchvelo

constexpr PID::config FL_VEL_CONFIG = {3, 0, 0};
constexpr PID::config FR_VEL_CONFIG = {3, 0, 0};
constexpr PID::config BL_VEL_CONFIG = {3, 0, 0};
constexpr PID::config BR_VEL_CONFIG = {3, 0, 0};

constexpr PID::config FLYWHEEL_L_PID = {7.1849, 0.000042634, 0};
constexpr PID::config FLYWHEEL_R_PID = {7.1849, 0.000042634, 0};
constexpr PID::config INDEXER_PID_VEL = {2.7, 0.001, 0};
constexpr PID::config INDEXER_PID_POS = {0.1, 0, 0.001};


// Devices from DT

const struct device *canbus1_dev = DEVICE_DT_GET(DT_NODELABEL(can1));

const struct device *canbus2_dev = DEVICE_DT_GET(DT_NODELABEL(can2));
constexpr short yaw_id = 3;
constexpr short pitch_id = 5;

const struct gpio_dt_spec led0_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
const struct gpio_dt_spec led1_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
const struct gpio_dt_spec led2_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);


static const struct i2c_dt_spec imu_spec = I2C_DT_SPEC_GET(DT_NODELABEL(imu));

static const struct pwm_dt_spec encoderSpec = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_encoder_ch));
static const struct device *controllerUart = DEVICE_DT_GET(DT_NODELABEL(usart1));
static const struct device *refUartDev = DEVICE_DT_GET(DT_NODELABEL(usart3));

static const struct device *jetsonUart = DEVICE_DT_GET(DT_NODELABEL(uart5));

// Subsystem Configs
TurretSubsystem::config turret_config = {
    canbus1_dev,
    CANHandler::CANBUS_1,
    yaw_id,
    M3508,
    canbus2_dev,
    CANHandler::CANBUS_2, // TODO: Ideally we could grab canbus thru the device
    pitch_id,
    M3508,
    YAW_VEL_PID,
    YAW_POS_PID,
    PITCH_VEL_PID,
    PITCH_POS_PID,
    yaw_static_friction,
    yaw_kinetic_friction,
    pitch_gravity_feedforward,
    pitch_static_friction,
    pitch_kinetic_friction,
    1,
    1.0/3.0,
    PITCH_LOWER_BOUND,
    PITCH_UPPER_BOUND
};
ShooterSubsystem::config shooter_config = {
    canbus2_dev,
    CANHandler::CANBUS_2,
    ShooterSubsystem::BURST,
    0,
    2,
    4,
    6,
    FLYWHEEL_L_PID,
    FLYWHEEL_R_PID,
    INDEXER_PID_VEL,
    INDEXER_PID_POS,
    false
};

// State variables (remote control path)
ChassisSpeeds des_chassis_state;
TurretSubsystem::TurretInfo des_turret_state;
ShootState des_shoot_state;

float pitch_desired_angle = 0.0;
float yaw_desired_angle = 0.0;

// A stream must have received a valid packet before it can enable actuators.
static bool command_is_fresh(uint64_t stamp_us, uint64_t current_us) {
    return stamp_us != 0 && current_us >= stamp_us &&
           current_us - stamp_us < JETSON_COMMAND_TIMEOUT_US;
}

class Sentry : public BaseRobot {
  public:
    ISM330 imu_;
    MA4 encoder_;
    Jetson jetson;
    Jetson::WriteState stm_state{};
    Jetson::ReadState jetson_state{};
    TurretSubsystem turret_;
    ShooterSubsystem shooter_;
    ChassisSubsystem chassis_;

    IMU::EulerAngles imuAngles;

    // Enum to more easily control robotstates, at comp we can easily just set neutral to jetson and profit 
    enum class ControlMode {
        REMOTE, // Mode N: yaw-oriented, remote/keyboard control
        JETSON, // Mode S: Jetson commands chassis / turret / shooter
        NEUTRAL    // Anything else: everything off
    };
    ControlMode control_mode_{ControlMode::NEUTRAL};

    Sentry(Config &config)
        : BaseRobot(config),
          imu_(imu_spec),
          encoder_(&encoderSpec),
          jetson(jetsonUart),
          turret_(turret_config, imu_),
          shooter_(shooter_config),
          chassis_(ChassisSubsystem::Config{
              5,        // left_front_can_id (Infantry hardware)
              1,        // right_front_can_id
              2,        // left_back_can_id
              4,        // right_back_can_id
              0.22617,  // radius
              0.065,    // speed_pid_ff_ks
              35,       // yaw_initial_offset_ticks
              imu_,
              &encoder_
          }) {}

    void init() override {
        if (!device_is_ready(controllerUart)) {
            printf("[ERROR] controllerUart (usart1) not ready!\n");
        }
        if (!device_is_ready(jetsonUart)) {
            printf("[ERROR] jetsonUart (uart5) not ready!\n");
        }
        imu_.begin(0.9, 0);
    }

    // Same override scheme as infantry: 'u' forces the first mode, 'd' forces
    // the second, 'o' defers to the remote's mode switch.
    //   N  (or drive 'u') -> REMOTE
    //   S  (or drive 'd') -> JETSON
    //   otherwise         -> NEUTRAL
    ControlMode select_mode() {
        if (drive == 'u' || (drive == 'o' && remote_.getMode() == DJIRemote2::ModeSwitch::MODE_N)) {
            return ControlMode::REMOTE;
        }
        if (drive == 'd' || (drive == 'o' && remote_.getMode() == DJIRemote2::ModeSwitch::MODE_S)) {
            return ControlMode::JETSON;
        }
        return ControlMode::NEUTRAL;
    }

    void periodic(unsigned long dt_us) override {
        imu_.mahonyUpdateIMU(dt_us / 1000000.0f);
        imuAngles = imu_.getImuAngles();

        // Always read the Jetson so its state stays current regardless of mode
        jetson_state = jetson.read();

        // Set the power budget before calculating chassis motor outputs.
        float power_limit = referee_.robot_status.chassis_power_limit;
        if (power_limit <= 0) {
            power_limit = 80;
        }
        chassis_.power_limit = power_limit;

        control_mode_ = select_mode();
        switch (control_mode_) {
            case ControlMode::REMOTE:   run_remote_mode(dt_us); break;
            case ControlMode::JETSON:   run_jetson_mode();      break;
            case ControlMode::NEUTRAL:  run_neutral_mode();        break;
        }

        turret_.periodic(chassis_.getChassisSpeeds().vOmega * 60.0f / (2.0f * PI));
        chassis_.periodic(&imuAngles);
        shooter_.periodic(referee_.power_heat_data.shooter_17mm_1_barrel_heat,
                          referee_.robot_status.shooter_barrel_heat_limit);

        set_jetson_state();
        jetson.write(stm_state);
    }

    unsigned int main_loop_dt_ms() override { return 2; } // 500 Hz loop

    // ---------------------------------------------------------------
    // Mode N: Infantry-mode basically, the remote does everything
    // ---------------------------------------------------------------
    void run_remote_mode(unsigned long dt_us) {
        const float dt_s = dt_us / 1000000.0f;

        // Chassis: yaw-oriented translation, no spin
        max_linear_vel = MAX_VEL;
        des_chassis_state.vX = jy * max_linear_vel;
        des_chassis_state.vY = jx * max_linear_vel;
        des_chassis_state.vOmega = 0;
        chassis_.setChassisSpeeds(des_chassis_state, ChassisSubsystem::DRIVE_MODE::YAW_ORIENTED);

        // Turret from remote (joystick + mouse)
        yaw_desired_angle -= myaw * 0.01f;
        yaw_desired_angle -= jyaw * JOYSTICK_YAW_SENSITIVITY_DPS * dt_s;
        yaw_desired_angle = capAngle(yaw_desired_angle);
        des_turret_state.yaw_angle_degs = yaw_desired_angle;

        pitch_desired_angle -= mpitch * 0.01f;
        pitch_desired_angle -= jpitch * JOYSTICK_PITCH_SENSITIVITY_DPS * dt_s;
        pitch_desired_angle = std::clamp(pitch_desired_angle, PITCH_LOWER_BOUND, PITCH_UPPER_BOUND);
        des_turret_state.pitch_angle_degs = pitch_desired_angle;
        des_turret_state.turret_mode = TurretState::AIM;

        // Shooter from remote
        if ((remote_.PAUSEToggled() == true && remote_.TriggerPressed() == true) || remote_.getMouseL()) {
            des_shoot_state = ShootState::SHOOT;
        } else if (remote_.CUSTRPressed() == true && remote_.PAUSEToggled() == true) {
            des_shoot_state = ShootState::JAM;
        } else if (remote_.PAUSEToggled() == true || shot == 'd') {
            des_shoot_state = ShootState::FLYWHEEL;
        } else {
            des_shoot_state = ShootState::OFF;
        }

        turret_.setState(des_turret_state);
        shooter_.setState(des_shoot_state);

        referee_.is_aligned = false;
        referee_.is_cv_on = false;
        referee_.is_spinning = false;
        referee_.is_flywheel_on = des_shoot_state != ShootState::OFF;
    }

    // ---------------------------------------------------------------
    // Mode S: Jetson mode 
    // ---------------------------------------------------------------
    void run_jetson_mode() {
        const uint64_t current_us = now_us();
        const bool chassis_enabled = command_is_fresh(jetson_state.chassis_stamp_us, current_us);
        const bool turret_enabled = command_is_fresh(jetson_state.turret_stamp_us, current_us);

        ChassisSpeeds chassis_command{};
        if (chassis_enabled) {
            // Preserve Sentry's Jetson axis signs; velocities are chassis-relative.
            chassis_command.vX = std::clamp(jetson_state.desired_x_vel, -MAX_VEL, MAX_VEL);
            chassis_command.vY = std::clamp(-jetson_state.desired_y_vel, -MAX_VEL, MAX_VEL);
            chassis_command.vOmega = std::clamp(jetson_state.desired_angular_vel,
                                               -JETSON_MAX_ANGULAR_VEL, JETSON_MAX_ANGULAR_VEL);
        }
        chassis_.setChassisSpeeds(chassis_command, ChassisSubsystem::DRIVE_MODE::ROBOT_ORIENTED);
        if (!chassis_enabled) {
            // Clear any PID output while waiting for the first packet or after timeout.
            chassis_.setWheelPower({0, 0, 0, 0});
        }

        TurretSubsystem::TurretInfo turret_command{};
        ShootState shoot_command = ShootState::OFF;
        if (turret_enabled) {
            turret_command.turret_mode = TurretState::AIM;
            turret_command.yaw_angle_degs = radiansToDegrees(
                std::remainder(jetson_state.desired_yaw_rads, 2.0f * PI));
            turret_command.pitch_angle_degs = radiansToDegrees(std::clamp(
                -jetson_state.desired_pitch_rads,
                degreesToRadians(PITCH_LOWER_BOUND), degreesToRadians(PITCH_UPPER_BOUND)));
            // The existing protocol has a single shoot flag, not separate flywheel/jam commands.
            if (jetson_state.shoot_status != 0) {
                shoot_command = ShootState::SHOOT;
            }
        } else {
            turret_command.turret_mode = TurretState::SLEEP;
        }
        turret_.setState(turret_command);
        shooter_.setState(shoot_command);

        // Keep the remote-mode setpoints tracking the real turret so switching
        // S -> N doesn't make the turret snap to a stale target.
        yaw_desired_angle = turret_.getState().yaw_angle_degs;
        pitch_desired_angle = std::clamp(turret_.getState().pitch_angle_degs,
                                         PITCH_LOWER_BOUND, PITCH_UPPER_BOUND);

        referee_.is_aligned = false;
        referee_.is_cv_on = turret_enabled;
        referee_.is_spinning = chassis_enabled && chassis_command.vOmega != 0.0f;
        referee_.is_flywheel_on = shoot_command != ShootState::OFF;
    }

    // ---------------------------------------------------------------
    // Any other switch position: everything off
    // ---------------------------------------------------------------
    void run_neutral_mode() {
        chassis_.setWheelPower({0, 0, 0, 0});

        des_turret_state.turret_mode = TurretState::SLEEP;
        des_turret_state.yaw_angle_degs = turret_.getState().yaw_angle_degs;
        des_turret_state.pitch_angle_degs = 0;
        yaw_desired_angle = turret_.getState().yaw_angle_degs;
        pitch_desired_angle = 0;

        turret_.setState(des_turret_state);
        shooter_.setState(ShootState::OFF);

        referee_.is_aligned = false;
        referee_.is_cv_on = false;
        referee_.is_spinning = false;
        referee_.is_flywheel_on = false;
    }

    void set_jetson_state() {
        // Only request autonomy while the remote has handed control to the Jetson.
        stm_state.activate_CV = (control_mode_ == ControlMode::JETSON) ? 1 : 0;
        stm_state.calibration = 0;
        stm_state.game_state = referee_.get_game_progress();
        stm_state.robot_hp = referee_.get_remain_hp();
        stm_state.team_color = referee_.is_red_or_blue();

        stm_state.chassis_x_velocity = chassis_.getChassisSpeeds().vX;
        stm_state.chassis_y_velocity = chassis_.getChassisSpeeds().vY;
        stm_state.chassis_rotation = chassis_.getChassisSpeeds().vOmega;

        stm_state.yaw_angle_rads = degreesToRadians(turret_.getState().yaw_angle_degs);
        stm_state.yaw_velocity = turret_.getState().yaw_velo_rad_s; // already rad/s
        stm_state.pitch_angle_rads = degreesToRadians(turret_.getState().pitch_angle_degs);
        stm_state.pitch_velocity = turret_.getState().pitch_velo_rad_s; // already rad/s
    }
};

int main(void) {
    BaseRobot::Config config{};
    config.led0_dev = &led0_dev;
    config.led1_dev = &led1_dev;
    config.led2_dev = &led2_dev;
    config.controller_uart_dev = controllerUart;
    config.referee_uart_dev = refUartDev;
    config.referee_usart_type = USART3;
    config.can1_dev = canbus1_dev;
    config.can2_dev = canbus2_dev;
    static Sentry sentry(config);
    sentry.main_loop();
}