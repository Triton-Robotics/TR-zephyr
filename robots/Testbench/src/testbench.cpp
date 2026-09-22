// #include "stm32f446xx.h"
// #include "syscalls/can.h"
#include "stm32f4xx_hal_can.h"
#include "syscalls/can.h"
#include <zephyr/drivers/can.h>
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

short constexpr MOTOR_ID = 2;
int constexpr SWAP_TIME = 1000;
int curr_time = 0;
short current_motor_power = 1000;

DJIMotor::config motorConfig = {canbus1_dev, MOTOR_ID, CANHandler::CANBUS_1, M3508};
DJIMotor motor(motorConfig);


double AG[6];

void periodic() {

    if(curr_time > SWAP_TIME) {
        motor.setPower(current_motor_power);
        current_motor_power *= -1;
        curr_time = 0;
        // printf("current time is : %d, swapping power to %d\n", curr_time, current_motor_power);

    } else {
        curr_time += 200;
        // printf("current time is : %d\n", curr_time);
    }
}

int main(void)
{
    printk("HERROoooo\n");

    // int res = -100;
    // res = can_start(canbus1_dev);
    
    // printk("CAN START %d\n", res);
    // printk("CAN SET MODE %d\n", can_set_mode(canbus1_dev, CAN_MODE_NORMAL));
    // can_state state;
    // can_bus_err_cnt err_cnt;
    // can_get_state(canbus1_dev, &state, &err_cnt);
    // printk("");

    if (device_is_ready(canbus1_dev)) {
        printk("CANBUS 1 device is ready\n");
    } else {
        printk("CANBUS 1 device is not ready\n");
        return -1;
    }

    
    while (true) {
        periodic();
        k_sleep(K_MSEC(200));
    }
}
