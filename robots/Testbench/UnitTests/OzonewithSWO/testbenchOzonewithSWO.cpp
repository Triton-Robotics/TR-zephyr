#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(swo_test, LOG_LEVEL_INF);

int main(void)
{
    int count = 0;
    while (1) {
        LOG_INF("hello from SWO %d", count++); // Using SWO
        printk("hello from UART %d\n", count); // Using default UART
        k_msleep(500);
    }
    return 0;
}