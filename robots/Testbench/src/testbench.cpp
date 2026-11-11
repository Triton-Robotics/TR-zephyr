
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


short constexpr MOTOR_ID = 2;
static const struct device *canbus1 = DEVICE_DT_GET(DT_NODELABEL(can1));
static const struct device *canbus2 = DEVICE_DT_GET(DT_NODELABEL(can2));
DJIMotor::config motorCFG {
    .motorID = MOTOR_ID,
    .canBus = CANHandler::CANBUS_1,
    .type = STANDARD,
    .name = "placeholder",
};
DJIMotor motor(motorCFG);


void periodic() {

    
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
    if(device_is_ready(canbus1)){
        printk("can1 initailized");
    }
    else{
        motor.setSpeed(0);
    }
    DJIMotor::setCanHandlers();
    DJIMotor::getCanHandler(CANHandler::CANBUS_1)->registerCallback(
    0x201, 0x208, DJIMotor::getCanOneFeedback);
    int power = 0;
    bool increasing = true;
    while(true){
        if(power==1000){increasing =false;}
        if(power==0){increasing=true;}
        if(increasing){power+=20;}
        else{power-=20;}
        motor.setPower(power);
        DJIMotor::sendValues(true);
        k_sleep(K_MSEC(100));
    }
}
