#include "ISM330EKF.h"
#include "zephyr/drivers/i2c.h"
#include <cmath>
#include <cstdint>
#include <util/algorithms/general_functions.h>

// Scale factors for calculations 
static constexpr float accelRangeScale = 0.122f;  // mg/LSB @ ±4g; see datasheet for other ranges
static constexpr float gyroRangeScale  = 70.0 * 1.5;   // mdps/LSB @ ±2000 dps * 1.5 for scaling error

static constexpr float g = 9.80665f; 
static constexpr float dpsToRad = PI / 180.0f;
static constexpr float radToDeg = 180.0f / PI;

static constexpr float accelConversion = accelRangeScale * g / 1000.0f; // The actual conversion factor from raw to m/s^2
static constexpr float gyroConversion = gyroRangeScale * dpsToRad / 1000.0f;   // The conversion factor from raw to rad/s


static constexpr uint8_t ISM330_CHIP_ID = 0x6B;   // Expected WHO_AM_I
static const uint8_t WhoAmIReg[] = {0x0F};

//Reading and Write Addresses
static constexpr int readAddr = 0b1101010;
static constexpr int writeAddr = 0b1101010;

static const uint8_t SWRST[] = {0x12,0x01}; // Software Reset command
static const uint8_t xEnable[] = {0x10, 0x7A}; // Enable accelerometer at 833Hz, +/-4g
//static const uint8_t xEnableMax[] = {0x10, 0xAA}; // Enable accelerometer at 
static const uint8_t gEnable[] = {0x11, 0x7C}; // Enable gyroscope at 833Hz, 2000dps
//static const uint8_t gEnableMax[] = {0x11, 0xAC}; // Enable gyroscope at 833Hz, 2000dps, with low pass filter

// Accelerometer and gyro register definitions
static constexpr uint8_t CTRL1_XL = 0x10;
static constexpr uint8_t CTRL2_G  = 0x11;
static constexpr uint8_t CTRL3_C  = 0x12;
static constexpr uint8_t CTRL4_C  = 0x13;
static constexpr uint8_t CTRL6_C  = 0x15;
static constexpr uint8_t STATUS_REG = 0x1E;
static const uint8_t gReg[] = {0x22,0x23,0x24,0x25,0x26,0x27}; //Gyro Register values
static const uint8_t xReg[] = {0x28,0x29,0x2A, 0x2B, 0x2C, 0x2D,}; //Accel Register Values
static const uint8_t tReg[] = {0x20, 0x21}; //Temperature register values
static const uint8_t gyroLowPassFilter[] = {CTRL6_C, 0x03}; // Low pass filter
static const uint8_t blockUpdate[] = {CTRL3_C, 0x44}; // Block update for reading consistent data


// Constructor
ISM330::ISM330(const struct i2c_dt_spec &i2c) noexcept : i2c_(i2c) {}; // Just makes the i2c_ attribute store the i2c_dt_spec

// Helper functions

void ISM330::writeReg(const uint8_t *cmd, size_t len) noexcept { // Should hopefully make the swap to SPI easier
    i2c_write_dt(&i2c_, cmd, len);
}

void ISM330::readReg(uint8_t reg, uint8_t *out, size_t len) noexcept {
    i2c_burst_read_dt(&i2c_, reg, out, len);
}

// ------------------- INITIALIZATION -------------------

bool ISM330::begin(float prop_gain, float int_gain) noexcept //TODO: Currently written in explicit i2c_writes so that ret is an actual test; change in future to writeReg/readReg
{
    
    int ret = i2c_write_dt(&i2c_, SWRST, sizeof(SWRST));
    if (ret != 0) {
        printf("SWRST write failed: %d\r\n", ret);
        return false;
    }
    k_sleep(K_MSEC(100));

    ret = i2c_write_dt(&i2c_, xEnable, sizeof(xEnable));
    ret |= i2c_write_dt(&i2c_, gEnable, sizeof(gEnable));
    ret |= i2c_write_dt(&i2c_, gyroLowPassFilter, sizeof(gyroLowPassFilter));
    ret |= i2c_write_dt(&i2c_, blockUpdate, sizeof(blockUpdate));
    if (ret != 0) {
        printf("IMU config write failed\r\n");
        return false;
    }


    // WHO_AM_I: write register addr, repeated-start, read back one byte.
    // i2c_reg_read_byte_dt does exactly this in a single transaction.
    ret = i2c_reg_read_byte_dt(&i2c_, WhoAmIReg[0], &whoAmIReading);
    if (ret != 0) {
        printf("WHO_AM_I read failed: %d\r\n", ret);
        return false;
    }

    // mahonyStart(prop_gain, int_gain); ---do i need this for ekf?
    calibrate();
    // mahonyStart(prop_gain, int_gain);
    // madgwickStart(1.0f);

    return true;
}

void ISM330::calibrate() noexcept
{
    printk("Calibrating IMU...\n");
    // Throw out first few readings
    for (int i = 0; i < 500; i++) {
        readAGraw();
        k_sleep(K_MSEC(1));
    }

    // Simple calibration routine, just takes 1000 readings and averages them to find bias values for accel and gyro
    axBias = 0;
    ayBias = 0;
    azBias = 0;
    wxBias = 0;
    wyBias = 0;
    wzBias = 0;

    for (int i = 0; i < 5000; i++) {
        ISM330_RAW_DATA_TypeDef raw_data = readAGraw();
        axBias += raw_data.ax;
        ayBias += raw_data.ay;
        azBias += raw_data.az;
        wxBias += raw_data.gx;
        wyBias += raw_data.gy;
        wzBias += raw_data.gz;
        k_sleep(K_MSEC(1));
    }

    axBias /= 5000.0f;
    ayBias /= 5000.0f;
    azBias /= 5000.0f;
    wxBias /= 5000.0f;
    wyBias /= 5000.0f;
    wzBias /= 5000.0f;

    printk("IMU Calibration complete.\n");
}




// ------------------- BASIC DEBUG FUNCTIONS -------------------
// uint8_t ISM330::whoAmI() noexcept
// {
//     i2c.write(writeAddr, WhoAmIReg, 1, false);
//     char whoAmIReading;
//     i2c.read(readAddr, reinterpret_cast<char*>(&whoAmIReading), 1, true);
//     return whoAmIReading;

// }

void ISM330::reset() noexcept
{
    writeReg(SWRST, sizeof(SWRST));
    k_sleep(K_MSEC(100));
}

// ------------------- SENSOR READOUT -------------------

//Helper functions to convert accel readings to real values

//Raw values to Acceleration (m/s^2)
ISM330::ISM330_VECTOR_TypeDef ISM330::readingToAccel(const uint8_t *readings, float temp) 
{
    int16_t rawX = (int16_t)(((uint16_t)readings[1] << 8) | readings[0]);
    int16_t rawY = (int16_t)(((uint16_t)readings[3] << 8) | readings[2]);
    int16_t rawZ = (int16_t)(((uint16_t)readings[5] << 8) | readings[4]);

    float rawXfloat = static_cast<float>(rawX);
    float rawYfloat = static_cast<float>(rawY);
    float rawZfloat = static_cast<float>(rawZ);

    float x = (((rawXfloat) * accelConversion) + accelTempOffset(temp)) *(1 + accelTempScale(temp));
    float y = (((rawYfloat) * accelConversion) + accelTempOffset(temp)) *(1 + accelTempScale(temp));
    float z = (((rawZfloat) * accelConversion) + accelTempOffset(temp)) *(1 + accelTempScale(temp));


    return {x, y, z};
}

//Raw to Gyro (Radians/second)
ISM330::ISM330_VECTOR_TypeDef ISM330::readingToGyro(const uint8_t *readings, float temp) 
{
    int16_t rawX = (int16_t)(((uint16_t)readings[1] << 8) | readings[0]);
    int16_t rawY = (int16_t)(((uint16_t)readings[3] << 8) | readings[2]);
    int16_t rawZ = (int16_t)(((uint16_t)readings[5] << 8) | readings[4]);

    float rawXfloat = static_cast<float>(rawX);
    float rawYfloat = static_cast<float>(rawY);
    float rawZfloat = static_cast<float>(rawZ);

    float x = (((rawXfloat-wxBias) * gyroConversion) + gyroTempOffset(temp)) *(1 + gyroTempScale(temp));
    float y = (((rawYfloat-wyBias) * gyroConversion) + gyroTempOffset(temp)) *(1 + gyroTempScale(temp));
    float z = (((rawZfloat-wzBias) * gyroConversion) + gyroTempOffset(temp)) *(1 + gyroTempScale(temp));

    return {x, y, z};
}

// Actual Reading functions
ISM330::ISM330_VECTOR_TypeDef ISM330::readAccel() noexcept
{
    readReg(xReg[0], xReadings,6);
    
    ISM330_VECTOR_TypeDef accel_reading = readingToAccel(xReadings, 0);
    return accel_reading;
}

ISM330::ISM330_VECTOR_TypeDef ISM330::readGyro() noexcept
{
    readReg(gReg[0], gReadings,6);

    ISM330_VECTOR_TypeDef gyro_reading = readingToGyro(gReadings, 0);
    return gyro_reading;
}


// Combined Read function, should be more efficient because less I2C transactions
ISM330::ISM330_RAW_DATA_TypeDef ISM330::readAGraw() noexcept 
{
    // i2c.write(writeAddr, gReg, 1, false); 
    // i2c.read(readAddr, reinterpret_cast<char*>(agReadings), 12, true);

    readReg(gReg[0],agReadings,12);

    int16_t rawGX = (int16_t)((agReadings[1] << 8) | agReadings[0]);
    int16_t rawGY = (int16_t)((agReadings[3] << 8) | agReadings[2]);
    int16_t rawGZ = (int16_t)((agReadings[5] << 8) | agReadings[4]);

    int16_t rawAX = (int16_t)((agReadings[7] << 8) | agReadings[6]);
    int16_t rawAY = (int16_t)((agReadings[9] << 8) | agReadings[8]);
    int16_t rawAZ = (int16_t)((agReadings[11] << 8) | agReadings[10]);

    return {rawAX, rawAY, rawAZ, rawGX, rawGY, rawGZ};
}



ISM330::ISM330_DATA_TypeDef ISM330::readAG() noexcept //
{
    // i2c.write(writeAddr, tReg, 1, false); 
    // i2c.read(readAddr, reinterpret_cast<char*>(agReadings), 14, true);

    readReg(tReg[0],agReadings, 14);

    int16_t rawTemp = (int16_t)((agReadings[1] << 8) | agReadings[0]);
    float temp = rawTemp / 256.0f;
    temperature = temp;

    //printf("Temperature: %.2f \n", temp);

    auto [gx,gy,gz] = readingToGyro(&agReadings[2], temp);
    auto [ax,ay,az] = readingToAccel(&agReadings[8], temp);

    // ISM330_VECTOR_TypeDef accelVector = {ax, ay, az};
    // ISM330_VECTOR_TypeDef gyroVector = {gx, gy, gz};

    return ISM330_DATA_TypeDef{ax, ay, az, gx, gy, gz};
}

void ISM330::getAGVectors(ISM330_VECTOR_TypeDef& accel, ISM330_VECTOR_TypeDef& gyro)
{
    ISM330_DATA_TypeDef imu_data = readAG();
    accel.x = imu_data.ax;
    accel.y = imu_data.ay;
    accel.z = imu_data.az;
    gyro.x = imu_data.gx;
    gyro.y = imu_data.gy;
    gyro.z = imu_data.gz;
}

float ISM330::readTemperature() noexcept // This is here for Debugging mostly, the actual temperature is already a part of the accel and gyro
{
    return temperature;
}

float ISM330::accelTempScale(float temp) 
{
    return temp * 0.00005f; // Datasheet says 0.005% per C
}

float ISM330::accelTempOffset(float temp) {
    return temp * 0.0001f * g; // Datasheet says 0.1mg per C
}

float ISM330::gyroTempScale(float temp) {
    return temp * 0.00007f; // Datasheet says 0.007% per C
}

float ISM330::gyroTempOffset(float temp) {
    return temp * 0.005f * dpsToRad; // Datasheet says 0.005 dps/C, but we're working in rad
}

//----? Do I need this

// The original Mahony algorithm used a fast inverse square root approx, but for accuracy we can just do the real thing I think
// If we need speed later, we can always switch to the approx
float ISM330::invSqrt(float x) 
{ 
    return 1/sqrt(x);

    //For easy access later, here's the approx

//     float halfx = 0.5f * x;
//   union {
//     float f;
//     long i;
//   } conv = {x};
//   conv.i = 0x5f3759df - (conv.i >> 1);
//   conv.f *= 1.5f - (halfx * conv.f * conv.f);
//   conv.f *= 1.5f - (halfx * conv.f * conv.f);
//   return conv.f;
}


IMU::EulerAngles ISM330::getImuAngles() 
{
    IMU::EulerAngles angles;
    // computeAnglesMah(angles);
    computeAnglesEKF(angles);
    return angles;
}

// EKF Sensor fusion

void ISM330::quatNormalize(float q[4])
{
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 1e-6f) { q[0]=1.0f; q[1]=q[2]=q[3]=0.0f; return; }
    float inv = 1.0f / n;
    q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv;
}

void ISM330::quatMul(float out[4], const float a[4], const float b[4])
{
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

void ISM330::QuatFromRotVec(float q[4], const float rv[3])
{
    float angle = sqrtf(rv[0]*rv[0] + rv[1]*rv[1] + rv[2]*rv[2]);
    if (angle < 1e-8f) { q[0]=1.0f; q[1]=q[2]=q[3]=0.0f; return; }
    float half = 0.5f * angle;
    float s = sinf(half) / angle;
    q[0] = cosf(half);
    q[1] = rv[0] * s;
    q[2] = rv[1] * s;
    q[3] = rv[2] * s;
}

void ISM330::rotMatFromQuat(float R[3][3], const float q[4])
{
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    R[0][0] = 1.0f - 2.0f*(q2*q2 + q3*q3);
    R[0][1] = 2.0f*(q1*q2 - q0*q3);
    R[0][2] = 2.0f*(q1*q3 + q0*q2);
    R[1][0] = 2.0f*(q1*q2 + q0*q3);
    R[1][1] = 1.0f - 2.0f*(q1*q1 + q3*q3);
    R[1][2] = 2.0f*(q2*q3 - q0*q1);
    R[2][0] = 2.0f*(q1*q3 - q0*q2);
    R[2][1] = 2.0f*(q2*q3 + q0*q1);
    R[2][2] = 1.0f - 2.0f*(q1*q1 + q2*q2);
}

void ISM330::ekfReset(float dtheta[3], float dbias[3])
{
    // Apply small rotation correction to quaternion
    float qd[4], qnew[4];
    QuatFromRotVec(qd, dtheta);
    float q[4] = { ekfq0, ekfq1, ekfq2, ekfq3 };
    quatMul(qnew, q, qd);
    quatNormalize(qnew);
    ekfq0 = qnew[0]; ekfq1 = qnew[1]; ekfq2 = qnew[2]; ekfq3 = qnew[3];

    // Apply bias correction
    for (int i = 0; i < 3; i++) gyro_bias[i] += dbias[i];

    // Reset covariance
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            P[i][j] = (i == j) ? 1e-6f : 0.0f;
}

void ISM330::ekfPredict(float gx, float gy, float gz, float dt)
{
    // Remove estimated bias
    float wx = gx - gyro_bias[0];
    float wy = gy - gyro_bias[1];
    float wz = gz - gyro_bias[2];

    // ---- State transition: quaternion integration ----
    float q[4] = { ekfq0, ekfq1, ekfq2, ekfq3 };
    float omega[4] = { 0.0f, wx, wy, wz };
    float qdot[4], qdelta[4];

    // qdot = 0.5 * q ⊗ omega
    quatMul(qdot, q, omega);
    qdot[0] *= 0.5f; qdot[1] *= 0.5f; qdot[2] *= 0.5f; qdot[3] *= 0.5f;

    for (int i = 0; i < 4; i++) qdelta[i] = q[i] + qdot[i] * dt;
    quatNormalize(qdelta);
    ekfq0 = qdelta[0]; ekfq1 = qdelta[1]; ekfq2 = qdelta[2]; ekfq3 = qdelta[3];

    // ---- Covariance propagation: P = F * P * F^T + Q ----
    // F ≈ I + dt * J, where J is the quaternion error Jacobian
    // Simplified error-state F for (dtheta, dbias):
    //   dtheta_dot = -[omega]x * dtheta - dbias
    //   dbias_dot  = 0
    float F[6][6];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            F[i][j] = 0.0f;

    // Cross-product matrix of (wx, wy, wz) negated
    float C[3][3] = {
        {  0.0f,   wz,    -wy  },
        { -wz,    0.0f,   wx   },
        {  wy,   -wx,     0.0f }
    };

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            F[i][j] = (i == j) ? 1.0f : 0.0f;
            F[i][j] += C[i][j] * dt;
            F[i+3][j] = 0.0f;
            F[i][j+3] = -dt * ((i == j) ? 1.0f : 0.0f);
            F[i+3][j+3] = (i == j) ? 1.0f : 0.0f;
        }
    }

    // Compute F*P
    float FP[6][6] = {0};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            for (int k = 0; k < 6; k++)
                FP[i][j] += F[i][k] * P[k][j];

    // Compute F*P*F^T
    float FPFt[6][6] = {0};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            for (int k = 0; k < 6; k++)
                FPFt[i][j] += FP[i][k] * F[j][k];  // F^T[j][k] = F[k][j]

    // Add process noise Q (diagonal)
    float qGyro  = sigma_gyro  * sigma_gyro  * dt;
    float qBias  = sigma_bias  * sigma_bias  * dt;
    for (int i = 0; i < 3; i++) {
        FPFt[i][i]     += qGyro;
        FPFt[i+3][i+3] += qBias;
    }

    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            P[i][j] = FPFt[i][j];
}

void ISM330::ekfUpdate(float ax, float ay, float az)
{
    // Normalize accelerometer (g to unit vector)
    float an = sqrtf(ax*ax + ay*ay + az*az);
    if (an < 1e-3f) return;
    ax /= an; ay /= an; az /= an;

    // Predicted gravity direction from quaternion
    float q[4] = { ekfq0, ekfq1, ekfq2, ekfq3 };
    float R[3][3];
    rotMatFromQuat(R, q);

    // Predicted gravity in body frame: g_pred = R^T * [0,0,1]
    float gpred[3] = { R[2][0], R[2][1], R[2][2] };

    // Innovation y = measured - predicted
    float y[3] = { ax - gpred[0], ay - gpred[1], az - gpred[2] };

    // Measurement Jacobian H (3x6): d(g_pred)/d(dtheta) = -[g_pred]x ; /d(dbias) = 0
    float H[3][6] = {0};
    H[0][0] =  0.0f;      H[0][1] =  gpred[2]; H[0][2] = -gpred[1];
    H[1][0] = -gpred[2];  H[1][1] =  0.0f;     H[1][2] =  gpred[0];
    H[2][0] =  gpred[1];  H[2][1] = -gpred[0]; H[2][2] =  0.0f;

    // S = H*P*H^T + Rm
    float HP[3][6] = {0};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 6; j++)
            for (int k = 0; k < 6; k++)
                HP[i][j] += H[i][k] * P[k][j];

    float S[3][3] = {0};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 6; k++)
                S[i][j] += HP[i][k] * H[j][k];
    for (int i = 0; i < 3; i++) S[i][i] += sigma_accel * sigma_accel;

    // Invert S (3x3) via adjugate / determinant
    float det = S[0][0]*(S[1][1]*S[2][2] - S[1][2]*S[2][1])
              - S[0][1]*(S[1][0]*S[2][2] - S[1][2]*S[2][0])
              + S[0][2]*(S[1][0]*S[2][1] - S[1][1]*S[2][0]);
    if (fabsf(det) < 1e-9f) return;
    float invDet = 1.0f / det;
    float Sinv[3][3];
    Sinv[0][0] =  (S[1][1]*S[2][2] - S[1][2]*S[2][1]) * invDet;
    Sinv[0][1] = -(S[0][1]*S[2][2] - S[0][2]*S[2][1]) * invDet;
    Sinv[0][2] =  (S[0][1]*S[1][2] - S[0][2]*S[1][1]) * invDet;
    Sinv[1][0] = -(S[1][0]*S[2][2] - S[1][2]*S[2][0]) * invDet;
    Sinv[1][1] =  (S[0][0]*S[2][2] - S[0][2]*S[2][0]) * invDet;
    Sinv[1][2] = -(S[0][0]*S[1][2] - S[0][2]*S[1][0]) * invDet;
    Sinv[2][0] =  (S[1][0]*S[2][1] - S[1][1]*S[2][0]) * invDet;
    Sinv[2][1] = -(S[0][0]*S[2][1] - S[0][1]*S[2][0]) * invDet;
    Sinv[2][2] =  (S[0][0]*S[1][1] - S[0][1]*S[1][0]) * invDet;

    // K = P * H^T * S^-1  (6x3)
    float PHt[6][3] = {0};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 6; k++)
                PHt[i][j] += P[i][k] * H[j][k];

    float K[6][3] = {0};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                K[i][j] += PHt[i][k] * Sinv[k][j];

    // dx = K * y  (6x1)
    float dx[6] = {0};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++)
            dx[i] += K[i][j] * y[j];

    // Apply correction
    float dtheta[3] = { dx[0], dx[1], dx[2] };
    float dbias[3]  = { dx[3], dx[4], dx[5] };
    ekfReset(dtheta, dbias);

    // P = (I - K*H) * P
    float KH[6][6] = {0};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            for (int k = 0; k < 3; k++)
                KH[i][j] += K[i][k] * H[k][j];

    float IKH[6][6];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            IKH[i][j] = ((i == j) ? 1.0f : 0.0f) - KH[i][j];

    float Pnew[6][6] = {0};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            for (int k = 0; k < 6; k++)
                Pnew[i][j] += IKH[i][k] * P[k][j];

    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            P[i][j] = Pnew[i][j];
}

void ISM330::computeAnglesEKF(IMU::EulerAngles& angles)
{
    // Read sensor and run one predict + update step
    static uint32_t last_ms = 0;
    uint32_t now_ms = k_uptime_get_32();
    float dt = (last_ms == 0) ? 0.001f : (now_ms - last_ms) * 0.001f;
    last_ms = now_ms;
    if (dt <= 0.0f) dt = 0.001f;
    if (dt > 0.1f)  dt = 0.1f;

    auto [ax, ay, az, gx, gy, gz] = readAG();

    ekfPredict(gx, gy, gz, dt);
    ekfUpdate(ax, ay, az);

    float q0 = ekfq0, q1 = ekfq1, q2 = ekfq2, q3 = ekfq3;

    angles.roll  = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * radToDeg;
    angles.pitch = asinf(-2.0f * (q1*q3 - q0*q2)) * radToDeg;
    angles.yaw   = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) * radToDeg;

    grav[0] = 2.0f * (q1*q3 - q0*q2);
    grav[1] = 2.0f * (q0*q1 + q2*q3);
    grav[2] = 2.0f * (q0*q0 - 0.5f + q3*q3);
    anglesComputed = 1;
}


void ISM330::initQuaternionFromAccel() 
{
    float axSum = 0, aySum = 0, azSum = 0;
    const int samples = 50;

    for (int i = 0; i < samples; i++) {
        ISM330_RAW_DATA_TypeDef raw = readAGraw();
        // Convert to m/s² but DO NOT subtract biases
        axSum += raw.ax * accelConversion;
        aySum += raw.ay * accelConversion;
        azSum += raw.az * accelConversion;
        k_sleep(K_MSEC(1));
    }

    float ax = axSum / samples;
    float ay = aySum / samples;
    float az = azSum / samples;

    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.001f) {
        mahq0 = 1.0f; mahq1 = 0.0f; mahq2 = 0.0f; mahq3 = 0.0f;
        return;
    }
    ax /= norm; ay /= norm; az /= norm;

    float pitch = asinf(-ax);
    float roll  = atan2f(ay, az);
    float yaw   = 0.0f;

    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll  * 0.5f);
    float sr = sinf(roll  * 0.5f);
    float cy = cosf(yaw   * 0.5f);
    float sy = sinf(yaw   * 0.5f);

    mahq0 = cr*cp*cy + sr*sp*sy;
    mahq1 = sr*cp*cy - cr*sp*sy;
    mahq2 = cr*sp*cy + sr*cp*sy;
    mahq3 = cr*cp*sy - sr*sp*cy;

    q0 = cr*cp*cy + sr*sp*sy;
    q1 = sr*cp*cy - cr*sp*sy;
    q2 = cr*sp*cy + sr*cp*sy;
    q3 = cr*cp*sy - sr*sp*cy;
}




void ISM330::computeAnglesEKF(IMU::EulerAngles& angles) 
{
    angles.roll = atan2f(ekfq0 * ekfq1 + ekfq2 * ekfq3, 0.5f - ekfq1 * ekfq1 - ekfq2 * ekfq2)*radToDeg;
    angles.pitch = asinf(-2.0f * (ekfq1 * ekfq3 - ekfq0 * ekfq2))*radToDeg;
    angles.yaw = atan2f(ekfq1 * ekfq2 + ekfq0 * ekfq3, 0.5f - ekfq2 * ekfq2 - ekfq3 * ekfq3)*radToDeg;
    anglesComputed = 1;
}


