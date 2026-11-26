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

short constexpr MOTOR_ID = 2;
int constexpr SWAP_TIME = 1000;
int curr_time = 0;
short current_motor_power = 1000;

DJIMotor::config motorConfig = {canbus1_dev, MOTOR_ID, CANHandler::CANBUS_2, M3508};
DJIMotor motor(motorConfig);


double AG[6];

void periodic() {

    DJIMotor::getCanHandler(CANHandler::CANBUS_2)->readAllCan();

    if(curr_time > SWAP_TIME) {
        motor.setPower(current_motor_power);
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
    static Referee ref(DEVICE_DT_GET(DT_NODELABEL(usart3)), USART3);  // PC11 RX, PC10 TX

    int sec = 0;
    while (true) {
        k_sleep(K_MSEC(1000));
        sec++;
        printf("[%d s] id=%u  hp=%u/%u  power_limit=%u  buffer=%u\n", sec,
               ref.robot_status.robot_id , ref.robot_status.current_HP, ref.robot_status.maximum_HP,
               ref.robot_status.chassis_power_limit, ref.power_heat_data.buffer_energy);

        // Every 5 s toggle the UI flags so writeThread sends UI packets
        if (sec % 5 == 0) {
            ref.is_spinning = !ref.is_spinning;
            ref.is_flywheel_on = !ref.is_flywheel_on;
        }
    }
}
