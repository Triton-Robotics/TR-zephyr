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

long curr_time_frame = 0;
long curr_loop_counter = 0;

int curr_multiturn_frame = 0;
int alt_curr_multiturn_frame = 0;

int curr_case_one_low_frame = 0;
int curr_case_two_low_frame = 0;
int curr_case_three_low_frame = 0;

int alt_curr_case_one_low_frame = 0;
int alt_curr_case_two_low_frame = 0;
int alt_curr_case_three_low_frame = 0;

int curr_case_one_high_frame = 0;
int curr_case_two_high_frame = 0;
int curr_case_three_high_frame = 0;

int alt_curr_case_one_high_frame = 0;
int alt_curr_case_two_high_frame = 0;
int alt_curr_case_three_high_frame = 0;

int last_motor_angle_frame = 0;
int curr_motor_angle_frame = 0;
int curr_motor_speed_frame = 0;
int curr_delta_angle_frame = 0;


int alt_last_motor_angle_frame = 0;
int alt_curr_motor_angle_frame = 0;
int alt_curr_motor_speed_frame = 0;
int alt_curr_delta_angle_frame = 0;



CANHandler canHandler1_(canbus1_dev);


DJIMotor::config m3508Config = {canbus1_dev, MOTOR_ID, CANHandler::CANBUS_1, M3508};  //C620, motorid = 2
DJIMotor motorM3508(m3508Config);


void periodic() {

    DJIMotor::getCanHandler(CANHandler::CANBUS_1)->readAllCan();

    current_motor_velocity = motorM3508.getData(VELOCITY);
    current_motor_multiturn = motorM3508.getData(MULTITURNANGLE);
    current_motor_multiturn_wrapped = (((int)current_motor_multiturn % 8192) + 8192) % 8192;
    fancy_current_motor_multiturn = motorM3508.multiTurnFancy;
    fancy_current_motor_multiturn_wrapped = (((int)current_motor_multiturn % 8192) + 8192) % 8192;
    current_motor_angle = motorM3508.getData(ANGLE);    
    curr_time = now_us();


    curr_case_one_low_frame = motorM3508.curr_case_one_low_frame;
    curr_case_two_low_frame = motorM3508.curr_case_two_low_frame;
    curr_case_three_low_frame = motorM3508.curr_case_three_low_frame;
    curr_case_one_high_frame = motorM3508.curr_case_one_high_frame;
    curr_case_two_high_frame = motorM3508.curr_case_two_high_frame;
    curr_case_three_high_frame = motorM3508.curr_case_three_high_frame;
    last_motor_angle_frame = motorM3508.last_motor_angle_frame;
    curr_motor_angle_frame = motorM3508.curr_motor_angle_frame;
    curr_motor_speed_frame = motorM3508.curr_motor_speed_frame;
    curr_delta_angle_frame = motorM3508.curr_delta_angle_frame;
    curr_multiturn_frame = motorM3508.curr_multiturn_frame;
    curr_loop_counter = motorM3508.curr_loop_counter;
    curr_time_frame = motorM3508.curr_time_frame;

    alt_curr_case_one_low_frame = motorM3508.alt_curr_case_one_low_frame;
    alt_curr_case_two_low_frame = motorM3508.alt_curr_case_two_low_frame;
    alt_curr_case_three_low_frame = motorM3508.alt_curr_case_three_low_frame;
    alt_curr_case_one_high_frame = motorM3508.alt_curr_case_one_high_frame;
    alt_curr_case_two_high_frame = motorM3508.alt_curr_case_two_high_frame;
    alt_curr_case_three_high_frame = motorM3508.alt_curr_case_three_high_frame;
    alt_last_motor_angle_frame = motorM3508.alt_last_motor_angle_frame;
    alt_curr_motor_angle_frame = motorM3508.alt_curr_motor_angle_frame;
    alt_curr_motor_speed_frame = motorM3508.alt_curr_motor_speed_frame;
    alt_curr_delta_angle_frame = motorM3508.alt_curr_delta_angle_frame;
    alt_curr_multiturn_frame = motorM3508.alt_curr_multiturn_frame;


    
    if(time_ctrl < SWAP_TIME) {
        motorM3508.setPower(current_motor_power);
        DJIMotor::sendValues(true);
    } else if(time_ctrl < 6 * SWAP_TIME && time_ctrl > 5 * SWAP_TIME) {
        motorM3508.setPower(-current_motor_power);
        DJIMotor::sendValues(true);
    } else {
        motorM3508.setPower(0);
        if(go) {
            DJIMotor::sendValues(true);
            go = false;
        }
    }
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
