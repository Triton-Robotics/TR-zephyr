#include <zephyr/kernel.h>
#include <stdio.h>

int main(void)
{
    printf("starting...\n");

    while(1) {
        printf("looping\n");
        k_usleep(10000);
    }
}