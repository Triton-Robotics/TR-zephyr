#include "stm32f446xx.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>
#include <util/communications/DJIRemote2.h>
#include <util/communications/CANHandler.h>
#include <util/peripherals/imu/ISM330.h>


const struct device *gpio_devb = DEVICE_DT_GET(DT_NODELABEL(gpiob));
const struct device *gpio_devc= DEVICE_DT_GET(DT_NODELABEL(gpioc));

const struct device *controllerUart = DEVICE_DT_GET(DT_NODELABEL(usart1));

DJIRemote2 controller(controllerUart);

// IMU Constucting
static const struct i2c_dt_spec imu_spec = I2C_DT_SPEC_GET(DT_NODELABEL(imu));
ISM330 imu(imu_spec); //Lowk beautiful, 1 year ago Dil would not believe his eyes



int main(void)
{
    if (!device_is_ready(gpio_devb) || !device_is_ready(gpio_devc) || (!device_is_ready(controllerUart))) {
        printf("GPIO device not ready!\n");
        return 0;
    }

    gpio_pin_configure(gpio_devb, 0, GPIO_OUTPUT);
    gpio_pin_configure(gpio_devc, 1, GPIO_OUTPUT);
    gpio_pin_configure(gpio_devc, 0, GPIO_OUTPUT);
    int timer = 0;

    while(1) {
        timer++;
        if (timer % 50 == 0) {
            gpio_pin_toggle(gpio_devb, 0);
        }

        if (timer % 200 == 0) {
            gpio_pin_toggle(gpio_devc, 1);
        }

        if (timer % 500 == 0) {
            gpio_pin_toggle(gpio_devc, 0);
        }

        k_usleep(1000);
    }
}