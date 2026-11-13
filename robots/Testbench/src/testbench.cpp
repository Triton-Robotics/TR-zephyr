#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

int count;

LOG_MODULE_REGISTER(swo_test, LOG_LEVEL_INF);

int main(void)
{
    count = 0;

    while (1) {
        // Goes to SWO through the logging backend
        LOG_INF("hello from LOG_INF %d", count);

        // Also goes to SWO, because CONFIG_LOG_PRINTK=y routes printk through logging
        printk("hello from printk %d\n", count);

        count++;
        k_msleep(500);
    }

    return 0;
}