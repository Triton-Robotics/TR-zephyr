
#include "stm32f446xx.h"
#include <cerrno>
#include <cstddef>
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


short constexpr MOTOR_ID = 7;

static const struct device* canbus1 = DEVICE_DT_GET(DT_NODELABEL(can1));
//static const struct device* canbus2 = DEVICE_DT_GET(DT_NODELABEL(can2));

DJIMotor::config motor_can_config {//developing the config to initialize the motor
        canbus1,
        MOTOR_ID,
        CANHandler::CANBus::CANBUS_1,
        M3508
};

DJIMotor motor {motor_can_config};
CANHandler* can1{nullptr};
float motor_current {};
float motor_torque {};

void test_1(){
    motor.setPower(600);
    DJIMotor::sendValues(true);
}


int power=0; 
int sign=1; 
int change=25;


void test_2(){

    motor.setPower(power);
    DJIMotor::sendValues(true);
    
    if(power==750){sign=-1;}
    if(power==-750){sign=1;}
    
    power+=sign*change;
    power = std::clamp(power, -750, 750);
}

void periodic() {
    can1->readAllCan();
    
    motor_current= motor.getData(motorDataType::POWEROUT);
    motor_torque= motor.getData(motorDataType::TORQUE);

    // test_1();
    test_2();
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
    /*
    struct config
    {
        const struct device *can_dev; 
        short motorID;
        CANHandler::CANBus canBus; 
        motorType type = STANDARD;
        std::string name = "NO_NAME";
        PID::config vel_cfg;
        PID::config pos_cfg = PID::config{0,0,0, 0, 0};
    };
    */
    
    

    DJIMotor::setCanHandlers();
    can1 = DJIMotor::getCanHandler(CANHandler::CANBUS_1);//
    if (!can1) {
        printk("CAN bus 1 initialization failed\n");
        return -1;
    }

    can1->registerCallback(0x201, 0x208, DJIMotor::getCanOneFeedback);
    while(true){
        periodic();
        k_sleep(K_MSEC(100));
    }
}

