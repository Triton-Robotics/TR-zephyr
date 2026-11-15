
#include "stm32f446xx.h"
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



// Devices from DT | Leaving these here for now, but these are debugging LEDs on the coverboard
const struct gpio_dt_spec led0_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
const struct gpio_dt_spec led1_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
const struct gpio_dt_spec led2_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

static const struct device *canbus1 = DEVICE_DT_GET(DT_NODELABEL(can1));
static const struct device *canbus2 = DEVICE_DT_GET(DT_NODELABEL(can2));

short constexpr MOTOR_ID = 2;

DJIMotor testMotor(MOTOR_ID, CANHandler::CANBUS_1, GM6020, "testMotor"); 

float motorCurrent;
float motorTorque;


void periodic(CANHandler* can) {
    can->readAllCan();
    motorCurrent = testMotor.getData(POWEROUT);
    motorTorque = testMotor.getData(TORQUE);
}

void testOne() {
    testMotor.setPower(600);
    DJIMotor::sendValues(true);
}

bool increasing = true;
int power = 0;

void testTwo() {
    if (power <= -750) {
        increasing = true;
        power+= 25;
    }
    else if (power >= 750) {
        increasing = false;
        power -= 25;
    }
    else if (increasing) {
        power+= 25;
    }
    else {
        power -= 25;
    }
    testMotor.setPower(std::clamp(power, -800, 800));
    testMotor.setPower(power);
    DJIMotor::sendValues(true);
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

    if (device_is_ready(canbus1)) {
        printk("canbus1\n");
    }
    else {
        printk("can1 not ready\n");
        return -1;
    }

    

    DJIMotor::setCanHandlers();

    CANHandler *can1 = DJIMotor::getCanHandler(CANHandler::CANBUS_1);
    if (!can1) {
        printk("CAN bus 1 initialization failed\n");
        return -1;
    }

    can1->registerCallback(0x201, 0x208, DJIMotor::getCanOneFeedback);

    while (true) {
        periodic(can1);
        testOne();
        k_sleep(K_MSEC(100));

    }
        
    




}
