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
    {2.0, 0.1, 0, 5000, 500},
    {0.03, 0, 0, 3000, 0}
};


DJIMotor indexer(indexerConfig);


constexpr double TICKS_PER_SLOT = static_cast<double>(TICKS_PER_ROTATION * (M2006_GEAR_RATIO) / NUM_INDEXER_SLOTS);

int target;
float INDEXER_ANGLE = -1.0f;
float start_angle = -1.0f;

void periodic(const uint64_t dt_us) {
    INDEXER_ANGLE = (indexer >> MULTITURNANGLE) - start_angle;
    indexer.setPosition(target);
    // indexer.setPower(500);
}

int main(void)
{
    uint64_t loop_clock_us = now_us();
    uint64_t prev_loop_time_us = loop_clock_us;

    uint64_t main_loop_dt_ms = 2;

    // CAN2 is the bxCAN slave, so CAN1 must be up too
    if (!device_is_ready(canbus1_dev) || !device_is_ready(canbus2_dev)) {
        printk("CAN device not ready!\n");
        return -1;
    }

    DJIMotor::setCanHandlers();

    CANHandler *can1 = DJIMotor::getCanHandler(CANHandler::CANBUS_1);
    CANHandler *can2 = DJIMotor::getCanHandler(CANHandler::CANBUS_2);
    if (can1) can1->registerCallback(0x201, 0x208, DJIMotor::getCanOneFeedback);
    if (can2) can2->registerCallback(0x201, 0x208, DJIMotor::getCanTwoFeedback);

    // enum can_state st; struct can_bus_err_cnt cnt;
    // can_get_state(canbus1_dev, &st, &cnt);
    // printf("can1 state=%d tx_err=%u rx_err=%u\n", st, cnt.tx_err_cnt, cnt.rx_err_cnt);


    for (int i = 0; i < 50; i++) {
        can1->readAllCan();
        k_sleep(K_MSEC(2));
    }


    start_angle = indexer >> MULTITURNANGLE;

    INDEXER_ANGLE = 0;
    target = static_cast<int>(start_angle + TICKS_PER_SLOT);

    indexer.setPower(0);
    // indexer.printAngle = true;

    uint64_t startTime = now_us();

    printf("warmup angle=%d\n", (int)(indexer >> MULTITURNANGLE));

    while (true) {
        loop_clock_us = now_us();

        if (loop_clock_us - startTime > 10e6) {
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

        k_sleep(K_MSEC(10));
    }
}