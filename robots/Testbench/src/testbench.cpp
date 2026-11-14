#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

int count;

LOG_MODULE_REGISTER(swo_test, LOG_LEVEL_INF);

int main(void)
{
    count = 0;

    while (1) {
        // Goes to SWO through the logging backend if CONFIG_LOG_BACKEND_SWO=y
        LOG_INF("hello from LOG_INF %d", count);
        printk("hello from printk %d\n", count);

        // Does not go through SWO because printf can only go through UART, can only be seen in serial monitor
        printf("hello from printf %d\n", count);        

        count++;
        k_msleep(1000);
    }

    return 0;
}