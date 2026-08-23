#include "stm32f446xx.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>
#include <util/communications/DJIRemote2.h>
#include <util/communications/CANHandler.h>
#include <util/peripherals/imu/ISM330.h>
#include <util/algorithms/PID.h>


// Robot Constants
constexpr float PITCH_LOWER_BOUND{-22.0};
constexpr float PITCH_UPPER_BOUND{25.0};

constexpr float JOYSTICK_YAW_SENSITIVITY_DPS = 300;
constexpr float JOYSTICK_PITCH_SENSITIVITY_DPS = 150;
// Mouse sensitivity initialized
constexpr float MOUSE_SENSITIVITY_YAW_DPS = 1.0;
constexpr float MOUSE_SENSITIVITY_PITCH_DPS = 1.0;

constexpr PID::config YAW_VEL_PID     = {181, 3.655 * 10e-3, 4.51 * 7.5, 32000, 1000};
constexpr PID::config YAW_POS_PID     = {1, 0, 0, 45, 2};
const float yaw_static_friction       = 0;//-150;       // We multiply it by dir
const float yaw_kinetic_friction      = 0;       // We multiply this by yawvelo

constexpr PID::config PITCH_VEL_PID   = {173.8994, 4.898 * 10e-6, 12.474 * 10e3, 16000, 2000}; //{25, 0.001, 5, 16000, 1000};
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


const struct device *gpio_devb = DEVICE_DT_GET(DT_NODELABEL(gpiob));
const struct device *gpio_devc= DEVICE_DT_GET(DT_NODELABEL(gpioc));
const struct device *controllerUart = DEVICE_DT_GET(DT_NODELABEL(usart1));

DJIRemote2 controller(controllerUart);

// IMU Constucting
static const struct i2c_dt_spec imu_spec = I2C_DT_SPEC_GET(DT_NODELABEL(imu));
ISM330 imu(imu_spec); //Lowk beautiful, 1 year ago Dil would not believe his eyes

// Subsystem Configs




int main(void)
{
    if (!device_is_ready(gpio_devb) || !device_is_ready(gpio_devc) || (!device_is_ready(controllerUart))) {
        printf("GPIO device not ready!\n");
        return 0;
    }

    gpio_pin_configure(gpio_devb, 0, GPIO_OUTPUT);
    gpio_pin_configure(gpio_devc, 1, GPIO_OUTPUT);
    gpio_pin_configure(gpio_devc, 0, GPIO_OUTPUT);
    int timer = 0;

    while(1) {
        timer++;
        if (timer % 50 == 0) {
            gpio_pin_toggle(gpio_devb, 0);
        }

        if (timer % 200 == 0) {
            gpio_pin_toggle(gpio_devc, 1);
        }

        if (timer % 500 == 0) {
            gpio_pin_toggle(gpio_devc, 0);
        }

        k_usleep(1000);
    }
}