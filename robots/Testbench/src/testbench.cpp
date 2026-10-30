// #include "stm32f446xx.h"
// #include "syscalls/can.h"
// #include "stm32f4xx_hal_can.h"
// #include "syscalls/can.h"
// #include <zephyr/drivers/can.h>
#include "zephyr/sys/printk.h"
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

short constexpr MOTOR_ID = 2;
int constexpr SWAP_TIME = 1000;
int time_ctrl = 0;
int caseMulti = 0;
int caseFancy = 0;
int currDeltaAngle = 0;
int lastMotorAngle = 0;
int lastLastMotorAngle = 0;
float current_motor_velocity = 0;
float current_motor_multiturn = 0;
float current_motor_multiturn_wrapped = 0;
float fancy_current_motor_multiturn = 0;
float fancy_current_motor_multiturn_wrapped = 0;
float current_motor_angle = 0;
long current_motor_power = -1000;
bool go = true;
unsigned long curr_time = 0;
int max_speed = 0;
int max_angle = 0;
unsigned int ticks = 0;

long curr_time_function_call = 0;
long function_call_counter = 0;

int high_speeds_multiturn_angle = 0;
int low_speeds_multiturn_angle = 0;

int high_speeds_case_one_false = 0;
int high_speeds_case_two_false = 0;
int high_speeds_case_three_false = 0;

int low_speeds_neg_skip_log = 0;
int low_speeds_pos_skip_log = 0;
int low_speeds_default_log = 0;

int high_speeds_neg_skip_log = 0;
int high_speeds_pos_skip_log = 0;
int high_speeds_default_log = 0;

int low_speeds_case_one_false = 0;
int low_speeds_case_two_false = 0;
int low_speeds_case_three_false = 0;

int high_speeds_last_angle = 0;
int high_speeds_curr_angle = 0;
float high_speeds_velocity = 0;
float high_speeds_velocity_raw = 0;
int high_speeds_delta_angle = 0;


int low_speeds_last_angle = 0;
int low_speeds_curr_angle = 0;
int low_speeds_velocity = 0;
int low_speeds_delta_angle = 0;

// Simple variables to keep track whether or not Ozone can track all of this data in one go
//      or needs a multiple logs
bool start_logging = false;
bool end_logging = false;

// last 15 records for multiturn angle
int last_log_0 = 0;
int last_log_1 = 0;
int last_log_2 = 0;
int last_log_3 = 0;
int last_log_4 = 0;
int last_log_5 = 0;
int last_log_6 = 0;
int last_log_7 = 0;
int last_log_8 = 0;
int last_log_9 = 0;
int last_log_10 = 0;
int last_log_11 = 0;
int last_log_12 = 0;
int last_log_13 = 0;
int last_log_14 = 0;
int log_index = 0;

// last 15 records for current time
int last_time_log_0 = 0;
int last_time_log_1 = 0;
int last_time_log_2 = 0;
int last_time_log_3 = 0;
int last_time_log_4 = 0;
int last_time_log_5 = 0;
int last_time_log_6 = 0;
int last_time_log_7 = 0;
int last_time_log_8 = 0;
int last_time_log_9 = 0;
int last_time_log_10 = 0;
int last_time_log_11 = 0;
int last_time_log_12 = 0;
int last_time_log_13 = 0;
int last_time_log_14 = 0;

// last 15 records for raw encoder angle
int last_curr_log_0 = 0;
int last_curr_log_1 = 0;
int last_curr_log_2 = 0;
int last_curr_log_3 = 0;
int last_curr_log_4 = 0;
int last_curr_log_5 = 0;
int last_curr_log_6 = 0;
int last_curr_log_7 = 0;
int last_curr_log_8 = 0;
int last_curr_log_9 = 0;
int last_curr_log_10 = 0;
int last_curr_log_11 = 0;
int last_curr_log_12 = 0;
int last_curr_log_13 = 0;
int last_curr_log_14 = 0;




CANHandler canHandler1_(canbus1_dev);


DJIMotor::config m3508Config = {canbus1_dev, MOTOR_ID, CANHandler::CANBUS_1, M3508};  //C620, motorid = 2
DJIMotor motorM3508(m3508Config);


void periodic() {

    DJIMotor::getCanHandler(CANHandler::CANBUS_1)->readAllCan();

    current_motor_velocity = motorM3508.getData(VELOCITY);
    current_motor_multiturn = motorM3508.getData(MULTITURNANGLE);
    current_motor_multiturn_wrapped = (((int)current_motor_multiturn % 8192) + 8192) % 8192;
    fancy_current_motor_multiturn = motorM3508.multiTurnLowSpeeds;
    fancy_current_motor_multiturn_wrapped = (((int)current_motor_multiturn % 8192) + 8192) % 8192;
    current_motor_angle = motorM3508.getData(ANGLE);    
    curr_time = now_us();

    start_logging = !start_logging;

    high_speeds_case_one_false = motorM3508.high_speeds_case_one_false;
    high_speeds_case_two_false = motorM3508.high_speeds_case_two_false;
    high_speeds_case_three_false = motorM3508.high_speeds_case_three_false;
    high_speeds_neg_skip_log = motorM3508.high_speeds_neg_skip_log;
    high_speeds_pos_skip_log = motorM3508.high_speeds_pos_skip_log;
    high_speeds_default_log = motorM3508.high_speeds_default_log;
    high_speeds_last_angle = motorM3508.high_speeds_last_angle;
    high_speeds_curr_angle = motorM3508.high_speeds_curr_angle;
    high_speeds_velocity = motorM3508.high_speeds_velocity;
    high_speeds_velocity_raw = motorM3508.high_speeds_velocity_raw;
    high_speeds_delta_angle = motorM3508.high_speeds_delta_angle;
    high_speeds_multiturn_angle = motorM3508.high_speeds_multiturn_angle;
    function_call_counter = motorM3508.function_call_counter;
    curr_time_function_call = motorM3508.curr_time;

    low_speeds_neg_skip_log = motorM3508.low_speeds_neg_skip_log;
    low_speeds_pos_skip_log = motorM3508.low_speeds_pos_skip_log;
    low_speeds_default_log = motorM3508.low_speeds_default_log;
    low_speeds_case_one_false = motorM3508.low_speeds_case_one_false;
    low_speeds_case_two_false = motorM3508.low_speeds_case_two_false;
    low_speeds_case_three_false = motorM3508.low_speeds_case_three_false;
    low_speeds_last_angle = motorM3508.low_speeds_last_angle;
    low_speeds_curr_angle = motorM3508.low_speeds_curr_angle;
    low_speeds_velocity = motorM3508.low_speeds_velocity;
    low_speeds_delta_angle = motorM3508.low_speeds_delta_angle;
    low_speeds_multiturn_angle = motorM3508.low_speeds_multiturn_angle;

    log_index = motorM3508.log_index;

    int log_size = 15;

    // last 15 records for angles
    last_log_0 = motorM3508.last_multiturn_angles_log[log_index];
    last_log_1 = motorM3508.last_multiturn_angles_log[(log_index + 1) % log_size];
    last_log_2 = motorM3508.last_multiturn_angles_log[(log_index + 2) % log_size];
    last_log_3 = motorM3508.last_multiturn_angles_log[(log_index + 3) % log_size];
    last_log_4 = motorM3508.last_multiturn_angles_log[(log_index + 4) % log_size];
    last_log_5 = motorM3508.last_multiturn_angles_log[(log_index + 5) % log_size];
    last_log_6 = motorM3508.last_multiturn_angles_log[(log_index + 6) % log_size];
    last_log_7 = motorM3508.last_multiturn_angles_log[(log_index + 7) % log_size];
    last_log_8 = motorM3508.last_multiturn_angles_log[(log_index + 8) % log_size];
    last_log_9 = motorM3508.last_multiturn_angles_log[(log_index + 9) % log_size];
    last_log_10 = motorM3508.last_multiturn_angles_log[(log_index + 10) % log_size];
    last_log_11 = motorM3508.last_multiturn_angles_log[(log_index + 11) % log_size];
    last_log_12 = motorM3508.last_multiturn_angles_log[(log_index + 12) % log_size];
    last_log_13 = motorM3508.last_multiturn_angles_log[(log_index + 13) % log_size];
    last_log_14 = motorM3508.last_multiturn_angles_log[(log_index + 14) % log_size];

    // last 15 records for now_us()
    last_time_log_0 = motorM3508.last_time_log[log_index];
    last_time_log_1 = motorM3508.last_time_log[(log_index + 1) % log_size];
    last_time_log_2 = motorM3508.last_time_log[(log_index + 2) % log_size];
    last_time_log_3 = motorM3508.last_time_log[(log_index + 3) % log_size];
    last_time_log_4 = motorM3508.last_time_log[(log_index + 4) % log_size];
    last_time_log_5 = motorM3508.last_time_log[(log_index + 5) % log_size];
    last_time_log_6 = motorM3508.last_time_log[(log_index + 6) % log_size];
    last_time_log_7 = motorM3508.last_time_log[(log_index + 7) % log_size];
    last_time_log_8 = motorM3508.last_time_log[(log_index + 8) % log_size];
    last_time_log_9 = motorM3508.last_time_log[(log_index + 9) % log_size];
    last_time_log_10 = motorM3508.last_time_log[(log_index + 10) % log_size];
    last_time_log_11 = motorM3508.last_time_log[(log_index + 11) % log_size];
    last_time_log_12 = motorM3508.last_time_log[(log_index + 12) % log_size];
    last_time_log_13 = motorM3508.last_time_log[(log_index + 13) % log_size];
    last_time_log_14 = motorM3508.last_time_log[(log_index + 14) % log_size];
    // last_time_log_0 = motorM3508.last_encoder_angles_log[0];
    // last_time_log_1 = motorM3508.last_encoder_angles_log[1];
    // last_time_log_2 = motorM3508.last_encoder_angles_log[2];
    // last_time_log_3 = motorM3508.last_encoder_angles_log[3];
    // last_time_log_4 = motorM3508.last_encoder_angles_log[4];
    // last_time_log_5 = motorM3508.last_encoder_angles_log[5];
    // last_time_log_6 = motorM3508.last_encoder_angles_log[6];
    // last_time_log_7 = motorM3508.last_encoder_angles_log[7];
    // last_time_log_8 = motorM3508.last_encoder_angles_log[8];
    // last_time_log_9 = motorM3508.last_encoder_angles_log[9];
    // last_time_log_10 = motorM3508.last_encoder_angles_log[10];
    // last_time_log_11 = motorM3508.last_encoder_angles_log[11];
    // last_time_log_12 = motorM3508.last_encoder_angles_log[12];
    // last_time_log_13 = motorM3508.last_encoder_angles_log[13];
    // last_time_log_14 = motorM3508.last_encoder_angles_log[14];

    // last 15 record for raw encoder value
    last_curr_log_0 = motorM3508.last_encoder_angles_log[log_index];
    last_curr_log_1 = motorM3508.last_encoder_angles_log[(log_index + 1) % log_size];
    last_curr_log_2 = motorM3508.last_encoder_angles_log[(log_index + 2) % log_size];
    last_curr_log_3 = motorM3508.last_encoder_angles_log[(log_index + 3) % log_size];
    last_curr_log_4 = motorM3508.last_encoder_angles_log[(log_index + 4) % log_size];
    last_curr_log_5 = motorM3508.last_encoder_angles_log[(log_index + 5) % log_size];
    last_curr_log_6 = motorM3508.last_encoder_angles_log[(log_index + 6) % log_size];
    last_curr_log_7 = motorM3508.last_encoder_angles_log[(log_index + 7) % log_size];
    last_curr_log_8 = motorM3508.last_encoder_angles_log[(log_index + 8) % log_size];
    last_curr_log_9 = motorM3508.last_encoder_angles_log[(log_index + 9) % log_size];
    last_curr_log_10 = motorM3508.last_encoder_angles_log[(log_index + 10) % log_size];
    last_curr_log_11 = motorM3508.last_encoder_angles_log[(log_index + 11) % log_size];
    last_curr_log_12 = motorM3508.last_encoder_angles_log[(log_index + 12) % log_size];
    last_curr_log_13 = motorM3508.last_encoder_angles_log[(log_index + 13) % log_size];
    last_curr_log_14 = motorM3508.last_encoder_angles_log[(log_index + 14) % log_size];

    end_logging = !end_logging;

    
    // if(time_ctrl < SWAP_TIME) {
    //     motorM3508.setPower(current_motor_power);
    //     DJIMotor::sendValues(true);
    // } else if(time_ctrl < 6 * SWAP_TIME && time_ctrl > 5 * SWAP_TIME) {
    //     motorM3508.setPower(-current_motor_power);
    //     DJIMotor::sendValues(true);
    // } else {
    //     motorM3508.setPower(0);
    //     if(go) {
    //         DJIMotor::sendValues(true);
    //         go = false;
    //     }
    // }
    // printf("HOI\n");`

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
    while (true) {
        periodic();
        k_sleep(K_MSEC(2));
    }
}
