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

constexpr PID::config FLYWHEEL_PID = {7.1849f, 0.000042634f, 0};
constexpr PID::config FEEDER_PID = {4, 0, 1};
constexpr PID::config INDEXER_VEL_PID = {2.7f, 0.001f, 0};
constexpr PID::config INDEXER_POS_PID = {0.1f, 0, 0.001f};

DJIMotor::config leftflywheelcfg {
    .can_dev = canbus1_dev,
    .motorID = 4,
    .canBus = CANHandler::CANBUS_1,
    .type = M3508,
    .vel_cfg = FLYWHEEL_PID
};

DJIMotor::config rightflywheelcfg{
    .can_dev = canbus1_dev,
    .motorID = 3,
    .canBus = CANHandler::CANBUS_1,
    .type = M3508,
    .vel_cfg = FLYWHEEL_PID
};

DJIMotor::config feederCFG{
    .can_dev = canbus1_dev,
    .motorID = 8,
    .canBus = CANHandler::CANBUS_1,
    .type = M2006,
    // .vel_cfg = FEEDER_PID,
};

MA4 encoder_(&encoderSpec);

DJIMotor leftflywheelmotor(leftflywheelcfg);
DJIMotor rightflywheelmotor(rightflywheelcfg);

DJIMotor feedermotor(feederCFG);

// static void dump_tim3() {
//     uint32_t c1 = TIM3->CNT;
//     k_busy_wait(200);
//     uint32_t c2 = TIM3->CNT;

//     // count edges on PB4 by polling the input data register for ~100 ms
//     int edges = 0;
//     uint32_t last = (GPIOB->IDR >> 4) & 1;
//     for (int i = 0; i < 10000; i++) {
//         k_busy_wait(10);
//         uint32_t now = (GPIOB->IDR >> 4) & 1;
//         if (now != last) { edges++; last = now; }
//     }

//     printk("TIM3 CNT %u->%u CR1=%08x SMCR=%08x DIER=%08x SR=%08x CCER=%08x CCR1=%u CCR2=%u\n",
//            c1, c2, (unsigned)TIM3->CR1, (unsigned)TIM3->SMCR, (unsigned)TIM3->DIER,
//            (unsigned)TIM3->SR, (unsigned)TIM3->CCER, (unsigned)TIM3->CCR1, (unsigned)TIM3->CCR2);
//     printk("PB4 edges in 100ms: %d | NVIC TIM3 enabled=%d pending=%d\n",
//            edges, (int)NVIC_GetEnableIRQ(TIM3_IRQn), (int)NVIC_GetPendingIRQ(TIM3_IRQn));
//     printk("PB4 MODER=%u AFRL=%u PUPDR=%u | CCMR1=%08x\n",
//        (unsigned)((GPIOB->MODER >> 8) & 3),
//        (unsigned)((GPIOB->AFR[0] >> 16) & 0xF),
//        (unsigned)((GPIOB->PUPDR >> 8) & 3),
//        (unsigned)TIM3->CCMR1);
// }

void periodic() {

    // printk("Encoder: %.2f\n", static_cast<double>(encoder_.getEncoderYawPosition()));
    // // printk("cb_count=%u status=%d\n", encoder_.cbCount(), encoder_.lastStatus());

    // encodersYaw = encoder_.encoderMovingAverage();
    // printk("cb=%u st=%d period=%u pulse=%u SR=%08x\n",
    //    encoder_.cbCount(), encoder_.lastStatus(),
    //    encoder_.periodCycles(), encoder_.pulseCycles(),
    //    (unsigned)TIM3->SR);
    // // dump_tim3();
    
    DJIMotor::getCanHandler(CANHandler::CANBUS_1)->readAllCan();

    leftflywheelmotor.setSpeed(-50);
    rightflywheelmotor.setSpeed(50);

    feedermotor.setPower(800);

     DJIMotor::sendValues();


    static int64_t last_log = 0;
if (k_uptime_get() - last_log >= 500) {
    last_log = k_uptime_get();
    printk("Feeder connected=%d velocity=%d output=%d\n",
           feedermotor.isConnected(),
           feedermotor >> VELOCITY,
           feedermotor.powerOut);
}
}

int main(void)
{
    printk("HERROoooo\n");

    DJIMotor::setCanHandlers();

    CANHandler *can1 = DJIMotor::getCanHandler(CANHandler::CANBUS_1);
    CANHandler *can2 = DJIMotor::getCanHandler(CANHandler::CANBUS_2);

    can1->registerCallback(0x201, 0x208, DJIMotor::getCanOneFeedback);



    if (!can1 || !can2) {
        printk("CAN initialization failed\n");
        return 0;
    }


    while (true) {
        periodic();
        k_sleep(K_MSEC(2));
    }

    return 0;
}