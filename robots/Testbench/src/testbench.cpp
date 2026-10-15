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
const struct pwm_dt_spec encoderSpec = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_encoder_ch));




MA4 encoder_(&encoderSpec);


void periodic() {

    printk("Encoder: %.2f\n", static_cast<double>(encoder_.getEncoderYawPosition()));
    printk("cb_count=%u status=%d\n", encoder_.cbCount(), encoder_.lastStatus());
}

int main(void)
{
    printk("HERROoooo\n");

    int ret = encoder_.init();
    if (ret) {
        printk("Failed to initialize encoder: %d\n", ret);
        return ret;
    }

    while (true) {
        periodic();
        k_sleep(K_MSEC(200));
    }
}
