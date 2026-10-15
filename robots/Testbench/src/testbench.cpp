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


float encodersYaw = 0.0f;

MA4 encoder_(&encoderSpec);


static void dump_tim3() {
    uint32_t c1 = TIM3->CNT;
    k_busy_wait(200);
    uint32_t c2 = TIM3->CNT;

    // count edges on PB4 by polling the input data register for ~100 ms
    int edges = 0;
    uint32_t last = (GPIOB->IDR >> 4) & 1;
    for (int i = 0; i < 10000; i++) {
        k_busy_wait(10);
        uint32_t now = (GPIOB->IDR >> 4) & 1;
        if (now != last) { edges++; last = now; }
    }

    printk("TIM3 CNT %u->%u CR1=%08x SMCR=%08x DIER=%08x SR=%08x CCER=%08x CCR1=%u CCR2=%u\n",
           c1, c2, (unsigned)TIM3->CR1, (unsigned)TIM3->SMCR, (unsigned)TIM3->DIER,
           (unsigned)TIM3->SR, (unsigned)TIM3->CCER, (unsigned)TIM3->CCR1, (unsigned)TIM3->CCR2);
    printk("PB4 edges in 100ms: %d | NVIC TIM3 enabled=%d pending=%d\n",
           edges, (int)NVIC_GetEnableIRQ(TIM3_IRQn), (int)NVIC_GetPendingIRQ(TIM3_IRQn));
    printk("PB4 MODER=%u AFRL=%u PUPDR=%u | CCMR1=%08x\n",
       (unsigned)((GPIOB->MODER >> 8) & 3),
       (unsigned)((GPIOB->AFR[0] >> 16) & 0xF),
       (unsigned)((GPIOB->PUPDR >> 8) & 3),
       (unsigned)TIM3->CCMR1);
}

void periodic() {

    printk("Encoder: %.2f\n", static_cast<double>(encoder_.getEncoderYawPosition()));
    // printk("cb_count=%u status=%d\n", encoder_.cbCount(), encoder_.lastStatus());

    encodersYaw = encoder_.encoderMovingAverage();
    // dump_tim3();
    
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
        k_sleep(K_MSEC(20));
    }
}
