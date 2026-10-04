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
#include <zephyr/drivers/adc.h>



// Devices from DT
const struct gpio_dt_spec led0_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
const struct gpio_dt_spec led1_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
const struct gpio_dt_spec led2_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
const struct device *canbus1_dev = DEVICE_DT_GET(DT_NODELABEL(can1));
const struct adc_dt_spec pot_dev = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

short constexpr MOTOR_ID = 2;
int constexpr SWAP_TIME = 1000;
int curr_time = 0;
short current_motor_power = 1000;

DJIMotor::config m2006Config = {canbus1_dev, MOTOR_ID, CANHandler::CANBUS_1, M2006};  //C610
DJIMotor motorM2006(m2006Config);

DJIMotor::config m3508Config = {canbus1_dev, MOTOR_ID, CANHandler::CANBUS_1, M3508};  //C620
DJIMotor motorM3508(m3508Config);


double AG[6];

//P = amp * volt

const float V_REF = 5.0f; //Reference voltage
const float MAX_POWER = 1500.0f;
const float V_Floor = 0.5f; // needs to be tuned?
const float scalingFactor = MAX_POWER / V_REF;  // 1500 / 5 = 300

float interpretVolt(float someVolt) {
    float power = someVolt * scalingFactor;
    if (power < V_Floor){ 
        power = 0.0f;
    } 
    else if (power > MAX_POWER){
        power = MAX_POWER;
    }
    return power;
};

static float read_Pot_V(){
    static int16_t buf;
    static struct adc_sequence sequence = {
        .buffer = &buf,
        .buffer_size = sizeof(buf),
    };
    static bool seq_ready = false;

    if (!seq_ready) {
        if (adc_sequence_init_dt(&pot_dev, &sequence) < 0) {
            return -1.0f;
        }
        seq_ready = true;
    }

    if (adc_read_dt(&pot_dev, &sequence) < 0) {
        return -1.0f;
    }

    int32_t mv = buf;
    if (adc_raw_to_millivolts_dt(&pot_dev, &mv) < 0) {
        return -1.0f;
    }
    return mv / 1000.0f;
}


void periodic() {

    DJIMotor::getCanHandler(CANHandler::CANBUS_1)->readAllCan();

    if(curr_time > SWAP_TIME) {
        motorM2006.setPower(current_motor_power);
        motorM3508.setPower(current_motor_power);
        current_motor_power *= -1;
        DJIMotor::sendValues(true);
        curr_time = 0;
        printf("current time is : %d, swapping power to %d\n", curr_time, current_motor_power);

    } else {
        curr_time += 200;
        // printf("current time is : %d\n", curr_time);
    }
}

int main(void)
{
    printk("HERROoooo\n");

    
    printk("CAN SET MODE %d\n", can_set_mode(canbus1_dev, CAN_MODE_NORMAL));
    printk("EBUSY %d, EIO %d, ENOTSUP %d\n", -EBUSY, -EIO, -ENOTSUP);
    
    
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

    if (!adc_is_ready_dt(&pot_dev)) {
        printf("ADC not ready\n");
        return 0;
    }
    if (adc_channel_setup_dt(&pot_dev) < 0) {
        printf("ADC channel setup failed\n");
        return 0;
    }

    while (true) {
        periodic();
        
        float voltage = read_Pot_V();
        if (voltage < 0.0f) {
            k_sleep(K_MSEC(100));
            continue;
        }
        float updated_motor_power = interpretVolt(voltage);
        printf("power is %f\n", updated_motor_power);

        motorM2006.setPower(updated_motor_power);
        motorM3508.setPower(updated_motor_power);
        DJIMotor::sendValues(true);

        k_sleep(K_MSEC(100));
    }
}
