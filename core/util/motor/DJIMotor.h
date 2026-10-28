#pragma once
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include "util/algorithms/PID.h"
#include "util/algorithms/general_functions.h"
#include "util/communications/CANHandler.h"
#include <cmath>
#include <string>
#include <optional>

constexpr float M3508_GEAR_RATIO = 3591.0f / 187.0f;
constexpr float M2006_GEAR_RATIO = 36.0f;

constexpr int TICKS_REVOLUTION = 8192;
constexpr int TIMEOUT_MS = 400;
constexpr int CAN_HANDLER_NUMBER = 2;

constexpr int INT16_T_MAX = 32767;
constexpr int INT15_T_MAX = 16383;

constexpr int s_sendIDs[3] = {0x200, 0x1FF, 0x2FF};                //IDs to send data



enum motorDataType {
    ANGLE,
    VELOCITY,
    TORQUE,
    TEMPERATURE,
    MULTITURNANGLE,
    POWEROUT,
    VALUE
};


//keep in mind that in the constructor, this is only used to
//set the default pid values and gear ratio. The motorType will
//be changed to STANDARD, because that's what the code uses.

enum motorType {
    NONE = 0,
    STANDARD = 1,       //identifier for all motors that use the standard can protocol, used by the C610 and C620
    GM6020 = 2,
    M3508 = 3,
    M2006 = 4,
    M3508_FLYWHEEL = 5,
};

class DJIMotor {
public:
    enum motorMoveMode{
        OFF,
        POS,
        SPD,
        POW,
        ERR
    };

    struct config
    {
        const struct device *can_dev; 
        short motorID;
        CANHandler::CANBus canBus; 
        motorType type = STANDARD;
        std::string name = "NO_NAME";
        PID::config vel_cfg;
        PID::config pos_cfg = PID::config{0,0,0, 0, 0};
    };


    static int motorCount;
    static bool initializedWarning;
    std::string name = "NO_NAME";
    int16_t powerOut = 0;

    PID pidSpeed;
    PID pidPosition;

    int multiTurn = 0;
    int multiTurnLowSpeeds = 0;
    int lastMotorSpeedLowSpeeds = 0;
    long lastMotorTimeLowSpeeds = 0;
    long lastMotorAngleLowSpeeds = 0;

    // ----------------------- DEBUGGING VARS ----------------------- 

    long curr_time = 0; // current time of now_us()
    long function_call_counter = 0; // records how many times the multiturn angle has been called

    int high_speeds_case_one_false = 0; // records cases in which a specific if statementa should NOT be running
    int high_speeds_case_two_false = 0; // records cases in which a specific if statementa should NOT be running
    int high_speeds_case_three_false = 0; // records cases in which a specific if statementa should NOT be running
    int high_speeds_neg_skip_log = 0;
    int high_speeds_pos_skip_log = 0;
    int high_speeds_default_log = 0;
    int high_speeds_last_angle = 0;
    int high_speeds_curr_angle = 0;
    int high_speeds_velocity = 0;
    int high_speeds_delta_angle = 0;
    int high_speeds_multiturn_angle = 0;

    int low_speeds_neg_skip_log = 0;
    int low_speeds_pos_skip_log = 0;
    int low_speeds_default_log = 0;
    int low_speeds_case_one_false = 0; // records cases in which a specific if statementa should NOT be running 
    int low_speeds_case_two_false = 0; // records cases in which a specific if statementa should NOT be running
    int low_speeds_case_three_false = 0; // records cases in which a specific if statementa should NOT be running
    int low_speeds_last_angle = 0;
    int low_speeds_curr_angle = 0;
    int low_speeds_velocity = 0;
    int low_speeds_delta_angle = 0;
    int low_speeds_multiturn_angle = 0;
    int log_size = 15; // size of record
    int log_index = 0; // next index to log data to (LAST RECORDED DATA IS STORED IN curr_index - 1!!!)
    int last_multiturn_angles_log[15]; // last 15 records for angles
    int last_encoder_angles_log[15]; // last 15 record for raw encoder value
    int last_time_log[15]; // last 15 record for time
    

    int outputCap = INT16_T_MAX;
    bool useAbsEncoder = false; // false for everything except gm6020s
    bool printAngle = false;

    motorType type = NONE;


    explicit DJIMotor(bool isErroneousMotor = false);
    DJIMotor(short motorID, CANHandler::CANBus canBus, motorType type = STANDARD, const std::string& name = "NO_NAME");
    DJIMotor(config cfg);
    ~DJIMotor();

    static int calculateDeltaPhase(int target, int current, int max = TICKS_REVOLUTION);
    static float calculateDeltaPhase(float target, float current, float max);

    static void setCanHandlers();
    static CANHandler* getCanHandler(CANHandler::CANBus bus) {
    return s_canHandlers[bus];
}
    static void getCanRxFeedback(const struct device *dev, struct can_frame *frame);
    static void getCanOneFeedback(const can_frame *frame);
    static void getCanTwoFeedback(const can_frame *frame);

    // static void getCan1Feedback(const CANMsg * msg);
    // static void getCan2Feedback(const CANMsg * msg);
    static void sendValues(bool debug = false);

    float getData(motorDataType data);

    inline void setMotorOutput(int val, motorMoveMode mod)
    {
        value = val;
        mode = mod;
    }

    inline void setPower(int power)
    {
        value = (float)power;
        mode = POW;
        powerOut = power;
    }

    inline void setSpeed(float speed){
        value = speed;
        mode = SPD;
    }

    inline void setPosition(float position){
        value = position;
        mode = POS;
    }

    inline motorMoveMode getMode(){
        return mode;
    }

    __attribute__((unused)) inline bool isConnected() const {
        return now_us() / 1000 - timeOfLastFeedback <= TIMEOUT_MS;
    }

    inline int operator>>(motorDataType data)                                                       { return getData(data); }
    DJIMotor(const DJIMotor&) = delete;
    DJIMotor& operator=(const DJIMotor&) = delete;


    inline void setPositionPID(float kP, float kI, float kD)                                        { pidPosition.setPID(kP, kI, kD); }
    inline void setPositionIntegralCap(double cap)                                                  { pidPosition.setIntegralCap((float)cap); }
    inline void setPositionOutputCap(double cap)                                                    { pidPosition.setOutputCap((float)cap); }

    inline void setSpeedPID(float kP, float kI, float kD)                                           { pidSpeed.setPID(kP, kI, kD); }
    inline void setSpeedIntegralCap(double cap)                                                     { pidSpeed.setIntegralCap((float)cap); }
    inline void setSpeedDerivativeCap(double cap)                                                   { pidSpeed.setDerivativeCap((float)cap); }
    inline void setSpeedOutputCap(double cap)                                                       { pidSpeed.setOutputCap((float)cap); }

    inline float calculateSpeedPID(float desired, float current, double dt)                               { return pidSpeed.calculate(desired, current, dt); }
    inline float calculatePositionPID(float desired, float current, double dt)                            { return pidSpeed.calculate(pidPosition.calculate(desired, current, dt), getData(VELOCITY), dt); }
    inline float calculatePeriodicPosition(float dE, double dt)                                       { return pidSpeed.calculate(pidPosition.calculatePeriodic(dE, dt), getData(VELOCITY), dt); }

private:
    static DJIMotor* s_allMotors  [CAN_HANDLER_NUMBER][3][4];
    static bool s_motorsExist     [CAN_HANDLER_NUMBER][3][4];

    static CANHandler* s_canHandlers[CAN_HANDLER_NUMBER];
    CANHandler::CANBus canBus = CANHandler::NOBUS;

    short canID_0;              // canID - 1, because canID is 1-8, arrays are 0-7
    short motorID_0;            // physical motorID - 1
    motorMoveMode mode = OFF;   // mode of the motor

    //  angle | velocity | torque | temperature
    int16_t motorData[4] = {};
    float value = 0;

    int lastMotorAngle = 0;
    int integratedAngle = 0;
    long lastIntegrationTime = -1;

    uint32_t timeOfLastFeedback = 0;
    uint32_t timeOfLastPID = 0;

    static void updateOneMultiTurn(int canBus, int can_line, int motor_id);
    static void sendOneID(CANHandler::CANBus canBus, short sendIDindex, bool debug = false);
    void setOutput();


    /** 
    @brief These are static instances of CANHandler; Set at runtime with DJIMotor::setCanHandlers().
    Use s_canHandlerInstances[i].value() to get a pointer to the CANHandler instance for bus i.


    @details We have these as static optional buffers because at compile time, we don't have the actual pointers, since we need the specific device tree pointers
    This could be done thru static function-scope instances, but that means accessing them could be a hassle. 
    More specifically, its a struct(?) that contains a buffer "value" for a CANHandler instance, and a bool 'has_value' that encodes whether or not 
    something is stored in the buffer

    **/
    static std::optional<CANHandler> s_canHandlerInstances[2]; 
};
