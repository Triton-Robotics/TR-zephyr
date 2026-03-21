#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>


const struct device *gpio_devb = DEVICE_DT_GET(DT_NODELABEL(gpiob));
const struct device *gpio_devc= DEVICE_DT_GET(DT_NODELABEL(gpioc));

int main(void)
{
    if (!device_is_ready(gpio_devb) || !device_is_ready(gpio_devc)) {
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