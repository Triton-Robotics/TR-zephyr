#include "subsystems/ChassisSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include <zephyr/drivers/i2c.h>
#include <stdio.h>
#include <util/communications/CANHandler.h>
#include <util/algorithms/PID.h>
#include <base_robot/BaseRobot.h>
#include <util/motor/DJIMotor.h>

// Devices from DT | Leaving these here for now, but these are debugging LEDs on the coverboard
const struct gpio_dt_spec led0_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
const struct gpio_dt_spec led1_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
const struct gpio_dt_spec led2_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

static const struct device *canbus1 = DEVICE_DT_GET(DT_NODELABEL(can1));
CANHandler* can1;

DJIMotor::config indexerConfig = {
    canbus1,
    1,
    CANHandler::CANBUS_1,
    M2006,
    "indexer",
    {3.0, 0.001, 0.0},
    {0.032, 0.001, 0.0015}
};

DJIMotor::config feederConfig = {
    canbus1,
    6,
    CANHandler::CANBUS_1,
    M2006,
    "feeder",
    {3.3, 0.001, 0.0},
    {0.04, 0.001, 0.0015}
};


DJIMotor indexer(indexerConfig);
DJIMotor feeder(feederConfig);

constexpr double TICKS_PER_OUTPUT_REV = static_cast<double>(TICKS_PER_ROTATION) * static_cast<double>(M2006_GEAR_RATIO);
constexpr double TICKS_PER_SLOT = static_cast<double>(TICKS_PER_ROTATION * (M2006_GEAR_RATIO) / NUM_INDEXER_SLOTS);

int target;
float INDEXER_ANGLE = -1.0f;
float INDEXER_SPEED = -1.0f;
float INDEXER_TORQUE = -1.0f;
float INDEXER_POWEROUT = -1.0f;
double start_angle = -1.0;

// void updateOzone() {
//     double ticks_from_start = static_cast<double>(indexer >> MULTITURNANGLE) - start_angle;
//     INDEXER_ANGLE = ticks_from_start / TICKS_PER_OUTPUT_REV * 360.0;  // degrees
//     INDEXER_SPEED = indexer.getData(VELOCITY) / M2006_GEAR_RATIO; // rad/s
//     INDEXER_TORQUE = indexer.getData(TORQUE);
//     INDEXER_POWEROUT = indexer.getData(POWEROUT);
// }

int main(void)
{
    uint64_t loop_clock_us = now_us();
    uint64_t prev_loop_time_us = loop_clock_us;


    if (!device_is_ready(canbus1)) {
        printf("fail to init can1!\n");
        return -1;
    }

    DJIMotor::setCanHandlers();
    can1 = DJIMotor::getCanHandler(CANHandler::CANBUS_1);
    if (!can1) {
        printk("CAN bus 1 initialization failed\n");
        return -1;
    }

    can1->registerCallback(0x201, 0x208, DJIMotor::getCanOneFeedback);


    start_angle = indexer >> MULTITURNANGLE;

    INDEXER_ANGLE = 0;
    target = static_cast<int>(start_angle + (TICKS_PER_SLOT * NUM_BALLS_SHOT));

    uint64_t startTime = now_us();

    printf("warmup angle=%d\n", (int)(indexer >> MULTITURNANGLE));

    bool stopped = false;

    while (true) {
        loop_clock_us = now_us();

        if (!stopped && (loop_clock_us - startTime >= 10'000'000ULL)) {
            indexer.setPower(0);
            feeder.setPower(0);
            DJIMotor::sendValues(true);

            stopped = true;
            printf("Stopped motors\n");
        }

        if ((loop_clock_us - prev_loop_time_us) >= 2000) {
            can1->readAllCan();
            printf("Indexer: angle=%d velocity=%d torque=%d\n",
                (int)indexer.getData(ANGLE),
                (int)indexer.getData(VELOCITY),
                (int)indexer.getData(TORQUE));

            printf("Feeder: angle=%d velocity=%d torque=%d\n",
                (int)feeder.getData(ANGLE),
                (int)feeder.getData(VELOCITY),
                (int)feeder.getData(TORQUE));

            if (!stopped) {
                // indexer.setPower(500);
                // feeder.setPower(500);
                indexer.setPosition(target);
                feeder.setPosition(target);
            }

            DJIMotor::sendValues(false);

            prev_loop_time_us = loop_clock_us;
        }

        k_sleep(K_MSEC(10));
    }
}