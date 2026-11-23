// #include "stm32f446xx.h"
// #include "syscalls/can.h"
// #include "stm32f4xx_hal_can.h"
// #include "syscalls/can.h"
// #include <zephyr/drivers/can.h>
#include "zephyr/sys/printk.h"
#include "zephyr/sys/util.h"
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
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/counter.h>




// Devices from DT
const struct gpio_dt_spec led0_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
const struct gpio_dt_spec led1_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
const struct gpio_dt_spec led2_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
const struct device *canbus1_dev = DEVICE_DT_GET(DT_NODELABEL(can1));
const struct device *const counter_dev = DEVICE_DT_GET(DT_NODELABEL(timers3));

short constexpr MOTOR_ID = 7;
int constexpr SWAP_TIME = 1000;
int time_ctrl = 0;
int caseMulti = 0;
int currDeltaAngle = 0;
int lastMotorAngle = 0;
int lastLastMotorAngle = 0;
float current_motor_velocity = 0;
float current_motor_multiturn = 0;
float current_motor_multiturn_wrapped = 0;
int current_motor_multiturn_rev = 0;
float current_motor_angle = 0;
long current_motor_power = 750;
bool go = true;
unsigned long curr_time = 0;
int max_speed = 0;
int max_angle = 0;
unsigned int ticks = 0;

long curr_time_function_call = 0;
long function_call_counter = 0;

int high_speeds_case_one_false = 0;
int high_speeds_case_two_false = 0;
int high_speeds_case_three_false = 0;


int high_speeds_neg_skip_log = 0;
int high_speeds_pos_skip_log = 0;
int high_speeds_default_log = 0;
int log_index = 0;
int log_count = 0;


int high_speeds_last_angle = 0;
int high_speeds_curr_angle = 0;
float high_speeds_velocity_raw = 0;
int high_speeds_delta_angle = 0;
double integration_time = 0;
double delta_time = 0;
float raw_speed = 0;

// Simple variables to keep track whether or not Ozone can track all of this data in one go
//      or needs a multiple logs
bool start_logging = false;
bool end_logging = false;



CANHandler canHandler1_(canbus1_dev);


DJIMotor::config m3508Config = {canbus1_dev, MOTOR_ID, CANHandler::CANBUS_1, M3508};  //C620, motorid = 2
DJIMotor motorM3508(m3508Config);
// std::string buffer1Name = 
decltype(motorM3508.buffer1) buffer1;
decltype(motorM3508.buffer2) buffer2;
decltype(motorM3508.buffer3) buffer3;
decltype(motorM3508.buffer4) buffer4;
decltype(motorM3508.buffer5) buffer5;


void periodic() {

    DJIMotor::getCanHandler(CANHandler::CANBUS_1)->readAllCan();

    current_motor_velocity = motorM3508.getData(VELOCITY);
    current_motor_multiturn = motorM3508.getData(MULTITURNANGLE);
    current_motor_multiturn_wrapped = (((int)current_motor_multiturn % 8192) + 8192) % 8192;
    current_motor_multiturn_rev = static_cast<int>(current_motor_multiturn/8192);
    current_motor_angle = motorM3508.getData(ANGLE);    
    curr_time = now_us();

    start_logging = !start_logging;

    high_speeds_case_one_false = motorM3508.low_speeds_neg_skip_log;
    high_speeds_case_two_false = motorM3508.low_speeds_pos_skip_log;
    high_speeds_case_three_false = motorM3508.low_speeds_default_log;
    high_speeds_neg_skip_log = motorM3508.high_speeds_neg_skip_log;
    high_speeds_pos_skip_log = motorM3508.high_speeds_pos_skip_log;
    high_speeds_default_log = motorM3508.high_speeds_default_log;

    log_index = motorM3508.log_index;
    log_count = motorM3508.log_count;

    for(int i = 0; i < motorM3508.log_size; i++) {
        buffer1[i] = motorM3508.buffer1[(motorM3508.log_index + i) % motorM3508.log_size];
        buffer2[i] = motorM3508.buffer2[(motorM3508.log_index + i) % motorM3508.log_size];
        buffer3[i] = motorM3508.buffer3[(motorM3508.log_index + i) % motorM3508.log_size];
        buffer4[i] = motorM3508.buffer4[(motorM3508.log_index + i) % motorM3508.log_size];
        buffer5[i] = motorM3508.buffer5[(motorM3508.log_index + i) % motorM3508.log_size];
    }

    end_logging = !end_logging;

    bool moveMotor = true;
    
    if(time_ctrl < SWAP_TIME && moveMotor) {
        if(current_motor_power < 1000) {
            current_motor_power = CLAMP(current_motor_power + 5, 0, 1000);
        }
        // 1 rotation
        motorM3508.setPower(-current_motor_power);
        DJIMotor::sendValues(true);
    // } else if(time_ctrl < 6 * SWAP_TIME && time_ctrl > 5 * SWAP_TIME) {
    //     motorM3508.setPower(-current_motor_power);
    //     DJIMotor::sendValues(true);
    } else {
        motorM3508.setPower(0);
        DJIMotor::sendValues(true);
    }

    time_ctrl += 5;
}

int main(void)
{
    printk("HERROoooo\n");
    
    
    can_state state;
    can_bus_err_cnt err_cnt;
    can_get_state(canbus1_dev, &state, &err_cnt);
    printk("STATE %d, RX ERROR %d, TX ERROR %d\n", state, err_cnt.rx_err_cnt, err_cnt.tx_err_cnt);

    if (device_is_ready(canbus1_dev)) {
        printk("CANBUS 1 device is ready\n");
    } else {
        printk("CANBUS 1 device is not ready\n");
        return -1;
    }

    DJIMotor::setCanHandlers();
    DJIMotor::getCanHandler(CANHandler::CANBUS_1)->registerCallback(
    0x201, 0x208, DJIMotor::getCanOneFeedback);

    motorM3508.zeroTicks();

    while (true) {
        periodic();
        k_sleep(K_MSEC(2));
    }
}
