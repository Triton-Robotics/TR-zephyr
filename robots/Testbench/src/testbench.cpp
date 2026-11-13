#include <util/peripherals/imu/ISM330.h>
#include "zephyr/drivers/i2c.h"
#include <cmath>
#include <cstdint>
#include <util/algorithms/general_functions.h>

#define ISM330_NODE DT_NODELABEL(imu)

float EKF_yaw;
float EKF_pitch;
float EKF_roll;

float MAH_yaw;
float MAH_pitch;
float MAH_roll;

const int64_t period_ms = 10; //100 Hz

static const struct i2c_dt_spec ism_i2c = I2C_DT_SPEC_GET(ISM330_NODE);
ISM330 imu(ism_i2c);

static IMU::EulerAngles mahAngles;
static IMU::EulerAngles ekfAngles;

void periodic() {
    // Mahony
    imu.mahonyUpdateIMU(period_ms / 1000.0f);
    imu.computeAnglesMah(mahAngles);
    MAH_roll  = mahAngles.roll;
    MAH_pitch = mahAngles.pitch;
    MAH_yaw   = mahAngles.yaw;

    // EKF
    imu.computeAnglesEKF(ekfAngles);
    EKF_roll = ekfAngles.roll;
    EKF_pitch = ekfAngles.pitch;
    EKF_yaw = ekfAngles.yaw;

    printf("Mahony: Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", MAH_roll, MAH_pitch, MAH_yaw);
    printf("EKF: Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", EKF_roll, EKF_pitch, EKF_yaw);
}


int main(void){
    if (!i2c_is_ready_dt(&ism_i2c)) {
        printf("I2C device not ready.");
        return -1;
    }

    i2c_recover_bus(ism_i2c.bus);

    if (!imu.begin(0.1f, 0.0f)) {
        printf("ISM330 initialization failed.");
        return -1;
    }
    printf("ISM330 initialized.");

    imu.ekfStart();

    int64_t next_wake = k_uptime_get(); 

    uint32_t i = 0;

    while (true) {
        periodic();

        i++;

        next_wake += period_ms;
        int64_t sleep_ms = next_wake - k_uptime_get();
        if (sleep_ms > 0) {
            k_sleep(K_MSEC(sleep_ms));
        } else {
            next_wake = k_uptime_get();
        }
    }

    return 0;
}