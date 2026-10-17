// #include "stm32f446xx.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>
#include <util/communications/DJIRemote2.h>
#include <util/communications/CANHandler.h>
#include <util/communications/jetson/Jetson.h>
#include <util/peripherals/imu/ISM330.h>
#include <util/algorithms/PID.h>
#include <base_robot/BaseRobot.h>
#include <zephyr/dt-bindings/pwm/pwm.h>

// Devices from DT

const struct device *canbus1_dev = DEVICE_DT_GET(DT_NODELABEL(can1));

const struct device *canbus2_dev = DEVICE_DT_GET(DT_NODELABEL(can2));

const struct gpio_dt_spec led0_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
const struct gpio_dt_spec led1_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
const struct gpio_dt_spec led2_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);


static const struct device *controllerUart = DEVICE_DT_GET(DT_NODELABEL(usart1));
// DJIRemote2 controller(controllerUart);

static const struct device *jetsonUart = DEVICE_DT_GET(DT_NODELABEL(uart5));


void get_controller_data(DJIRemote2 &remote)
{
    float jyaw = remote.getJoystickValue(DJIRemote2::Joystick::RIGHT_HORIZONTAL);
    float jpitch = remote.getJoystickValue(DJIRemote2::Joystick::RIGHT_VERTICAL);
    float jx = remote.getJoystickValue(DJIRemote2::Joystick::LEFT_HORIZONTAL);
    float jy = remote.getJoystickValue(DJIRemote2::Joystick::LEFT_VERTICAL);

    DJIRemote2::ModeSwitch mode = remote.getMode();
    const char *mode_name = (mode == DJIRemote2::ModeSwitch::MODE_C) ? "C"
                          : (mode == DJIRemote2::ModeSwitch::MODE_N) ? "N" : "S";

    bool trigger = remote.TriggerPressed();

    printf("jyaw: %.2f, jpitch: %.2f, jx: %.2f, jy: %.2f, mode: %s, trigger: %d\n",
           jyaw, jpitch, jx, jy, mode_name, trigger);
}

void periodic(DJIRemote2 &remote){
    get_controller_data(remote);
}


int main(void)
{
    printf("HELLO\n");
    
    DJIRemote2 remote_(controllerUart);


    while (true) {
        periodic(remote_);
        k_msleep(100);
    }
}
