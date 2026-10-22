// #include "stm32f446xx.h"
// #include "syscalls/can.h"
// #include "stm32f4xx_hal_can.h"
// #include "syscalls/can.h"
// #include <zephyr/drivers/can.h>
#include <cerrno>
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
#include <util/motor/DJIMotor.h>



// Devices from DT
const struct gpio_dt_spec led0_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
const struct gpio_dt_spec led1_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
const struct gpio_dt_spec led2_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
const struct device *canbus1_dev = DEVICE_DT_GET(DT_NODELABEL(can1));

short constexpr MOTOR_ID = 1;

DJIMotor::config indexerConfig = {
    canbus1_dev,
    MOTOR_ID,
    CANHandler::CANBUS_1,
    M2006,
    "indexer",
    {2.7, 0.001, 0},
    {0.1, 0, 0.001}
};
DJIMotor indexer(indexerConfig);


double AG[6];

void periodic() {
    indexer.setSpeed(500);
    DJIMotor::sendValues(false);   // true = debug prints
}

int main(void)
{
    if (!device_is_ready(canbus1_dev)) {
        printk("CANBUS 1 device is not ready\n");
        return -1;
    }
    printk("CANBUS 1 device is ready\n");

    DJIMotor::setCanHandlers();   // must happen before any send/read

    while (true) {
        periodic();
        k_sleep(K_MSEC(200));
    }
}
