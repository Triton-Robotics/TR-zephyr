
#include <algorithm>
#include <cerrno>
#include "stm32f446xx.h"
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



// Devices from DT | Leaving these here for now, but these are debugging LEDs on the coverboard
const struct gpio_dt_spec led0_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
const struct gpio_dt_spec led1_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
const struct gpio_dt_spec led2_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);


short constexpr MOTOR_ID = 1;
static const struct device* canbus1 = DEVICE_DT_GET(DT_NODELABEL(can1));
//static const struct device* canbus2 = DEVICE_DT_GET(DT_NODELABEL(can2));

DJIMotor::config motorConfig = {
    canbus1,
    MOTOR_ID,
    CANHandler::CANBUS_1,
    M2006,
};
CANHandler* can1;
DJIMotor motor = DJIMotor(motorConfig);

float current{};
float torque{};

void testOne() {
    motor.setPower(600);
    DJIMotor::sendValues(true);
}

int motorPower{0};
bool increasing{true};

void testTwo() {
    if (motorPower >= 750) {
        increasing = false;
    } else if (motorPower <= -750) {
        increasing = true;
    }

    if (increasing) {
        motorPower += 25;
    } else {
        motorPower -= 25;
    }

    motorPower = std::clamp(motorPower, -750, 750);
    DJIMotor::sendValues(true);
}


void periodic() {
    can1->readAllCan();

    testOne();
    // testTwo();

    current = motor.getData(POWEROUT);
    torque = motor.getData(TORQUE);
}

int main(void)
{
    // I'm keeping these here just as debugging things. Don't worry about them
    // printk("CAN SET MODE %d\n", can_set_mode(canbus1_dev, CAN_MODE_NORMAL));
    // printk("EBUSY %d, EIO %d, ENOTSUP %d\n", -EBUSY, -EIO, -ENOTSUP);


    // can_state state;
    // can_bus_err_cnt err_cnt;
    // can_get_state(canbus1_dev, &state, &err_cnt);
    // printk("STATE %d, RX ERROR %d, TX ERROR %d\n", state, err_cnt.rx_err_cnt, err_cnt.tx_err_cnt);


    // --------------------------- Your code after this line

    if (!device_is_ready(canbus1)) {
        printf("fail to init can1!\n");
        return -1;
    }

    // if (!device_is_ready(canbus2)) {
    //     printf("fail to init can2!\n");
    //     return -1;
    // }

    DJIMotor::setCanHandlers();
    can1 = DJIMotor::getCanHandler(CANHandler::CANBUS_1);
    if (!can1) {
        printk("CAN bus 1 initialization failed\n");
        return -1;
    }

    can1->registerCallback(0x201, 0x208, DJIMotor::getCanOneFeedback);

    while (true) {
        periodic();
        k_msleep(100);
    }
}
