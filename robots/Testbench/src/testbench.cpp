#include <zephyr/drivers/i2c.h>
#include <stdio.h>
#include <util/communications/CANHandler.h>
#include <util/algorithms/PID.h>
#include <base_robot/BaseRobot.h>
#include <util/motor/DJIMotor.h>



// Devices from DT
static const struct device *canbus2_dev = DEVICE_DT_GET(DT_NODELABEL(can2));
static const struct device *canbus1_dev = DEVICE_DT_GET(DT_NODELABEL(can1));

short constexpr MOTOR_ID = 1;

DJIMotor::config indexerConfig = {
    canbus1_dev,
    MOTOR_ID,
    CANHandler::CANBUS_1,
    M2006,
    "indexer",
    {0.5, 0, 0, 10000, 500},
    {0.1, 0, 0.001, 10000, 500}
};


DJIMotor indexer(indexerConfig);


constexpr double TICKS_PER_SLOT = static_cast<double>(TICKS_PER_ROTATION * (M2006_GEAR_RATIO) / NUM_INDEXER_SLOTS);

int target;
float INDEXER_ANGLE = -1.0f;
float start_angle = -1.0f;

void periodic(const unsigned long dt_us) {
    INDEXER_ANGLE = (indexer >> MULTITURNANGLE) - start_angle;
    indexer.setPosition(target);
}

int main(void)
{
    unsigned long loop_clock_us = now_us();
    unsigned long prev_loop_time_us = loop_clock_us;

    unsigned long main_loop_dt_ms = 2;

    // CAN2 is the bxCAN slave, so CAN1 must be up too
    if (!device_is_ready(canbus1_dev) || !device_is_ready(canbus2_dev)) {
        printk("CAN device not ready!\n");
        return -1;
    }

    DJIMotor::setCanHandlers();

    CANHandler *can1 = DJIMotor::getCanHandler(CANHandler::CANBUS_1);
    CANHandler *can2 = DJIMotor::getCanHandler(CANHandler::CANBUS_2);
    if (can1) can1->registerCallback(0x201, 0x20D, DJIMotor::getCanOneFeedback);
    if (can2) can2->registerCallback(0x201, 0x20D, DJIMotor::getCanTwoFeedback);

    for (int i = 0; i < 20; i++) {
        can1->readAllCan();
        can2->readAllCan();
        k_sleep(K_MSEC(5));
    }

    start_angle = indexer >> MULTITURNANGLE;
    INDEXER_ANGLE = 0;
    target = static_cast<double>((TICKS_PER_ROTATION * M2006_GEAR_RATIO / 4.0f) + (indexer>>MULTITURNANGLE));

    
    printf("multiturn start: %d\n", indexer>>MULTITURNANGLE);
    printf("target: %d\n", target);

    uint64_t startTime = now_us();

    while (true) {
        loop_clock_us = now_us();

        if (loop_clock_us - startTime > 5e6) {
            indexer.setPower(0.0f);
            DJIMotor::sendValues(false);
            break;
        }

        if ((loop_clock_us - prev_loop_time_us) / 1000 >= main_loop_dt_ms) {
            periodic(loop_clock_us - prev_loop_time_us);
            prev_loop_time_us = loop_clock_us;

            // Motor updates
            DJIMotor::sendValues();
        }
        if (can1) can1->readAllCan();
        if (can2) can2->readAllCan();

        k_sleep(K_MSEC(10));
    }
    
    printf("multiturn end: %d\n", indexer>>MULTITURNANGLE);
}