
#include "DJIMotor.h"
#include "CANHandler.h"
#include <cstdint>


// Here to prevent any linker errors, since these are static variables that need to be defined somewhere in the cpp
int DJIMotor::motorCount = 0;
DJIMotor* DJIMotor::s_allMotors[2][3][4];
bool DJIMotor::s_motorsExist[2][3][4];
CANHandler* DJIMotor::s_canHandlers[2];
bool DJIMotor::initializedWarning = false;
std::optional<CANHandler> DJIMotor::s_canHandlerInstances[2];

DJIMotor::DJIMotor(bool isErroneousMotor) {
    canID_0 = -1;
    motorID_0 = -1;
    canBus = CANHandler::NOBUS;
    value = 0;
    mode = OFF;

    if (isErroneousMotor)
        mode = ERR;

    powerOut = 0;
}

DJIMotor::DJIMotor(short motorID, CANHandler::CANBus canBus, motorType type,
                   const std::string &name) {
    DJIMotor::motorCount++;
    canID_0 = static_cast<short>(motorID - 1);
    motorID_0 = canID_0;
    this->canBus = canBus;
    this->type = type;
    this->name = name;

    value = 0;

    // Adding 4 to move to the next ID line
    if (type == GM6020) {
        canID_0 += 4;
        useAbsEncoder = true;
    }

    int can_line =
        canID_0 /
        4; // The can line depends on the motor id and type: M3508 1-4 on line
           // 0, 5-8 on line 1, GM6020 1-4 on line 1, 5-8 on line 2
    int index = canID_0 % 4; // The index within the can line

    if (!s_motorsExist[canBus][can_line][index]) {
        s_allMotors[canBus][can_line][index] = this;
        s_motorsExist[canBus][can_line][index] = true;

    }else {
       printf("[ERROR] THERES A CONFLICT ON BUS [%d] MOTOR [%d], \"%s\". YOU "
              "WILL HAVE ERRORS.\n", canBus + 1, motorID, name.c_str());
       // do NOT touch s_allMotors here — leave the original valid pointer alone
   }
    // } else {
    //     DJIMotor mot(true);
    //     s_allMotors[canBus][can_line][index] = &mot;
    //     printf("[ERROR] THERES A CONFLICT ON BUS [%d] MOTOR [%d], \"%s\". YOU "
    //            "WILL HAVE ERRORS.\n",
    //            canBus + 1, motorID + 1, name.c_str());
    // }

    if (motorID > 8 || motorID < 1) {
        printf("[ERROR] The canID [%d] not within correct bounds\n", motorID);
    }

    powerOut = 0;
}

DJIMotor::DJIMotor(config cfg)
    : DJIMotor(cfg.motorID, cfg.canBus, cfg.type, cfg.name) {
    pidSpeed = PID(cfg.vel_cfg);
    pidPosition = PID(cfg.pos_cfg);
}

DJIMotor::~DJIMotor() {
    mode = OFF;
    s_motorsExist[canBus][canID_0 / 4][canID_0 % 4] = false;
    canID_0 = -1;
    motorID_0 = -1;
    canBus = CANHandler::NOBUS;
}

void DJIMotor::setOutput() {
    uint32_t time = now_us();

    if (mode == SPD)
        powerOut = (short) calculateSpeedPID(value, getData(VELOCITY),
                                     static_cast<double>(time - timeOfLastPID));

    else if (mode == POS)
        if (!useAbsEncoder)
            powerOut = (short) calculatePositionPID(value, getData(MULTITURNANGLE),
                                     static_cast<double>(time - timeOfLastPID));

        else
            powerOut = (short) calculatePeriodicPosition(
                static_cast<float>(calculateDeltaPhase(value, getData(ANGLE))),
                static_cast<double>(time - timeOfLastPID));

    else if (mode == POW)
        powerOut = (short) value;

    else if (mode == OFF)
        powerOut = 0;

    else
        printf("[ERROR] THIS IS AN ERRONEOUS MOTOR. ON BUS [%d] MOTOR [%d], "
               "\"%s\". YOU WILL HAVE ERRORS.\n. DO NOT ATTEMPT TO SEND IT "
               "DATA, DO NOT PASS GO, DO NOT COLLECT $200, FIX THIS!\n",
               canBus + 1, motorID_0 + 1, name.c_str());

    if (type == M3508_FLYWHEEL || type == M3508 || type == M2006) {
        if (powerOut > outputCap || powerOut > INT15_T_MAX)
            powerOut = (short) std::min(outputCap, INT15_T_MAX);

        else if (powerOut < -outputCap || powerOut < -INT15_T_MAX)
            powerOut = (short) std::max(-outputCap, -INT15_T_MAX);

    } else {
        if (powerOut > outputCap || powerOut > INT16_T_MAX)
            powerOut = (short) std::min(outputCap, INT16_T_MAX);

        else if (powerOut < -outputCap || powerOut < -INT16_T_MAX)
            powerOut = (short) std::max(-outputCap, -INT16_T_MAX);
    }

    this->powerOut = static_cast<int16_t>(powerOut);
    timeOfLastPID = time;
}

void DJIMotor::sendValues(bool debug) {
    // printf("Inside send\n");
    for (short canBus = 0; canBus < CAN_HANDLER_NUMBER; canBus++)
        for (short sendIDindex = 0; sendIDindex < 3; sendIDindex++)
            sendOneID((CANHandler::CANBus)canBus, sendIDindex, debug);
}

void DJIMotor::sendOneID(CANHandler::CANBus canBus, short sendIDindex,
                         bool debug) {
    uint8_t bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    bool anyMotors = false;

    if (debug)
        printf("0x%x:\t", s_sendIDs[sendIDindex]);

    for (int i = 0; i < 4; i++) {
        if (s_motorsExist[canBus][sendIDindex][i]) {
            s_allMotors[canBus][sendIDindex][i]->setOutput();
            int16_t powerOut = s_allMotors[canBus][sendIDindex][i]->powerOut;

            bytes[2 * i] = static_cast<uint8_t>(powerOut >> 8);
            bytes[2 * i + 1] = static_cast<uint8_t>(powerOut);

            anyMotors = true;

            if (debug)
                printf("%d\t", powerOut);

        } else if (debug)
            printf("NA\t");
    }
    
    if (anyMotors) {
        if (s_canHandlers[canBus] != nullptr){
            s_canHandlers[canBus]->rawSend(s_sendIDs[sendIDindex], bytes);
        } else {
            printf("\n[ERROR] YOUR CANHANDLERS ARE NOT DEFINED YET. DO THIS "
                   "BEFORE YOU CALL ANY MOTORS,\n USING "
                   "[(DJIMotor::s_setCANHandlers(PA_11,PA_12,PB_12,PB_13)], "
                   "WHERE PA_11, PA_12 ARE TX, RX\n");
            }
    }

    if (debug) {
        printf("\n");
    }
}

static const struct device *s_canDevices[2]; // 2 Canbuses

void DJIMotor::setCanHandlers() { //This sets the devices and the canhandlers
    static bool already_initialized = false;
    if (already_initialized) {
        printf("[WARNING] setCanHandlers() called more than once — ignoring.\n");
        return;
    }
    already_initialized = true;
    
    
    s_canDevices[CANHandler::CANBUS_1] = DEVICE_DT_GET(DT_NODELABEL(can1));
    s_canDevices[CANHandler::CANBUS_2] = DEVICE_DT_GET(DT_NODELABEL(can2));

    for (int i = 0; i < 2; i++) {
        if (!device_is_ready(s_canDevices[i])) {
            printf("ERROR, CANbus %d not ready", i);
        }
    }

    for (int i = 0; i < 2; i++) {
        if (!device_is_ready(s_canDevices[i])) {
            printf("ERROR, CANbus %d not ready", i);
            continue;
        }
        s_canHandlerInstances[i].emplace(s_canDevices[i]); // emplace is used to construct std::optional objects, in this case the CANHandler instance
        int ret = s_canHandlerInstances[i]->init();
        if (ret < 0) {
            printf("[ERROR] CANHandler %d init failed: %d\n", i, ret);
        } else {
            s_canHandlers[i] = &s_canHandlerInstances[i].value();
        }
    }
}


// TODO: Figure out if this can even be implemented cause it's kinda weird - Dil
void DJIMotor::getCanRxFeedback(const struct device *dev, struct can_frame *msg) { 
    int canBus = -1; //Initialize here so clangd doesn't warn
    if (dev == DEVICE_DT_GET(DT_NODELABEL(can1))) canBus = CANHandler::CANBUS_1;
    else if (dev == DEVICE_DT_GET(DT_NODELABEL(can2))) canBus = CANHandler::CANBUS_2;
    int canID_0 = msg->id - 0x201;

    // Find which motor line we are in (0x200, 0x1FF, 0x2FF)
    int can_line = canID_0 / 4;

    // Find which motor it is
    int motor_id = canID_0 % 4;

    // Update motor specific data
    if (s_motorsExist[canBus][can_line][motor_id]) {
        DJIMotor *motor = s_allMotors[canBus][canID_0 / 4][canID_0 % 4];

        motor->motorData[ANGLE] =
            static_cast<int16_t>(msg->data[0] << 8 | msg->data[1]);
        motor->motorData[VELOCITY] =
            static_cast<int16_t>(msg->data[2] << 8 | msg->data[3]);
        motor->motorData[TORQUE] =
            static_cast<int16_t>(msg->data[4] << 8 | msg->data[5]);
        motor->motorData[TEMPERATURE] = static_cast<int16_t>(msg->data[6]);
        motor->timeOfLastFeedback = now_us() / 1000;

        if (motor->motorData[TEMPERATURE] > 80)
            printf("[WARNING] YOU HAVE A MOTOR [0x%x] ATTACHED THAT IS %d "
                   "DEGREES CELSIUS ON BUS [%d] ID [%d], \"%s\" \n",
                   msg->id, motor->motorData[TEMPERATURE], canBus + 1,
                   motor->motorID_0 + 1, motor->name.c_str());

    } else if (initializedWarning) {
        printf("[WARNING] YOU HAVE A MOTOR [0x%x] {%d}{%d}{%d} ATTACHED THAT "
               "IS NOT INITIALIZED.. WHY: \n",
               msg->id, canBus, can_line, motor_id);
    }
    updateOneMultiTurn(canBus, can_line, motor_id);
}

void DJIMotor::getCanOneFeedback(const struct can_frame *msg) {
    int canBus = CANHandler::CANBUS_1; //Initialize here so clangd doesn't warn
    int canID_0 = msg->id - 0x201;

    // Find which motor line we are in (0x200, 0x1FF, 0x2FF)
    int can_line = canID_0 / 4;

    // Find which motor it is
    int motor_id = canID_0 % 4;

    // Update motor specific data
    if (s_motorsExist[canBus][can_line][motor_id]) {
        DJIMotor *motor = s_allMotors[canBus][canID_0 / 4][canID_0 % 4];

        motor->motorData[ANGLE] =
            static_cast<int16_t>(msg->data[0] << 8 | msg->data[1]);
        motor->motorData[VELOCITY] =
            static_cast<int16_t>(msg->data[2] << 8 | msg->data[3]);
        motor->motorData[TORQUE] =
            static_cast<int16_t>(msg->data[4] << 8 | msg->data[5]);
        motor->motorData[TEMPERATURE] = static_cast<int16_t>(msg->data[6]);
        motor->timeOfLastFeedback = now_us() / 1000;

        if (motor->motorData[TEMPERATURE] > 80)
            printf("[WARNING] YOU HAVE A MOTOR [0x%x] ATTACHED THAT IS %d "
                   "DEGREES CELSIUS ON BUS [%d] ID [%d], \"%s\" \n",
                   msg->id, motor->motorData[TEMPERATURE], canBus + 1,
                   motor->motorID_0 + 1, motor->name.c_str());

    } else if (initializedWarning) {
        printf("[WARNING] YOU HAVE A MOTOR [0x%x] {%d}{%d}{%d} ATTACHED THAT "
               "IS NOT INITIALIZED.. WHY: \n",
               msg->id, canBus, can_line, motor_id);
    }
    updateOneMultiTurn(canBus, can_line, motor_id);
}

void DJIMotor::getCanTwoFeedback(const struct can_frame *msg) {
    int canBus = CANHandler::CANBUS_2; //Initialize here so clangd doesn't warn
    int canID_0 = msg->id - 0x201;

    // Find which motor line we are in (0x200, 0x1FF, 0x2FF)
    int can_line = canID_0 / 4;

    // Find which motor it is
    int motor_id = canID_0 % 4;

    // Update motor specific data
    if (s_motorsExist[canBus][can_line][motor_id]) {
        DJIMotor *motor = s_allMotors[canBus][canID_0 / 4][canID_0 % 4];

        motor->motorData[ANGLE] =
            static_cast<int16_t>(msg->data[0] << 8 | msg->data[1]);
        motor->motorData[VELOCITY] =
            static_cast<int16_t>(msg->data[2] << 8 | msg->data[3]);
        motor->motorData[TORQUE] =
            static_cast<int16_t>(msg->data[4] << 8 | msg->data[5]);
        motor->motorData[TEMPERATURE] = static_cast<int16_t>(msg->data[6]);
        motor->timeOfLastFeedback = now_us() / 1000;

        if (motor->motorData[TEMPERATURE] > 80)
            printf("[WARNING] YOU HAVE A MOTOR [0x%x] ATTACHED THAT IS %d "
                   "DEGREES CELSIUS ON BUS [%d] ID [%d], \"%s\" \n",
                   msg->id, motor->motorData[TEMPERATURE], canBus + 1,
                   motor->motorID_0 + 1, motor->name.c_str());

    } else if (initializedWarning) {
        printf("[WARNING] YOU HAVE A MOTOR [0x%x] {%d}{%d}{%d} ATTACHED THAT "
               "IS NOT INITIALIZED.. WHY: \n",
               msg->id, canBus, can_line, motor_id);
    }
    updateOneMultiTurn(canBus, can_line, motor_id);
}


// Samples angle and updates multiturn angle to match angle and accounts for any
//      wrap-arounds to keep a consistent count of multiturn angle 
void DJIMotor::updateOneMultiTurn(int canBus, int can_line, int motor_id) {
    /** 
        8192 is encoder reading
        1/1000 is microsec to millisec
        integratedAngle = (-4096, 4096)
    **/

    constexpr int Threshold = 4096; // From 0 to 8191
    constexpr double time_threshold_ms = 1; 

    float raw_speed, raw_rps, cur_angle, last_angle, delta_angle;
    double dt_ms = 0;
    DJIMotor *curMotor;

    unsigned long time = now_us();


    if (s_motorsExist[canBus][can_line][motor_id]) {
        curMotor = s_allMotors[canBus][can_line][motor_id];
        
        // calculates delta time and handles first itteration setup
        if (curMotor->lastIntegrationTime != -1) {
            dt_ms = static_cast<double>(time - curMotor->lastIntegrationTime) / 1000.0d; // ms
        } else {
            curMotor->lastRps = curMotor->getData(VELOCITY_RAW) / 60.0f;
            curMotor->lastMotorAngle = curMotor->getData(ANGLE);
            curMotor->lastIntegrationTime = static_cast<long>(time);
            return;
        }

        // Forces to sample at consistent intervals/frequencies
        if(dt_ms < time_threshold_ms) {
            return;
        }
        
        cur_angle = curMotor->getData(ANGLE);
        last_angle = curMotor->lastMotorAngle;
        delta_angle = cur_angle - last_angle;
        
        raw_speed = curMotor->getData(VELOCITY_RAW);
        raw_rps = curMotor->getData(VELOCITY_RAW) / 60.0f;


        /*
            Slow speed mode where we assume that inbetween intervals, our motor
            does not spin more than one full rotation
            This allows us to base our multiturn adjustement to be more accurate
            by using pure encoder (low speeds have some trouble with velocity
            being accurate)
        */
        if (fabsf(raw_rps) < 100.0f) {
            // negative wrap (pos -> neg)
            if (cur_angle > (ENCODER_WRAP - Threshold) && last_angle < Threshold) {
                delta_angle -= ENCODER_WRAP;
                curMotor->low_speeds_neg_skip_log++;
            } 
            // positive wrap (neg -> pos)
            else if (cur_angle < Threshold && last_angle > (ENCODER_WRAP - Threshold)) {
                delta_angle += ENCODER_WRAP;
                curMotor->low_speeds_pos_skip_log++;
            }
            // no wrap (just for logging rn) 
            else {
                curMotor->low_speeds_default_log++;
            }
            curMotor->multiTurn += delta_angle;
        } 
        /*
            High speeds logic where we assume that inbetween intervals, our motor
            has a change to spin more than one full rotation
            Here, we use velocity and encoder values to calculate when we do a jump
            (in order to have our percentage of rotation value be accurate to motor
            encoder) as well as estimate the amount of rotation we need to add onto
            via velocity * dt
        */
        else {
            float predictedRev = (curMotor->lastRps + raw_rps)/2.0f * (dt_ms / 1000);
            float actualRev = static_cast<float>(cur_angle)/ENCODER_WRAP;
            float lastRev = static_cast<float>(last_angle)/ENCODER_WRAP;       

            // speed is backwards while our deltaAngle =ispositive
            //  thus we know for certain we have done a neg wrap (pos -> neg)
            if (predictedRev < 0 && delta_angle > 0) { 
                delta_angle -= ENCODER_WRAP;
                curMotor->high_speeds_neg_skip_log++;
            } 
            // speed is forwards while our deltaAngle is negative
            //  thus we know for certain we have done a pow wrap (neg -> pos)
            else if (predictedRev > 0 && delta_angle < 0) { 
                delta_angle += ENCODER_WRAP;
                curMotor->high_speeds_pos_skip_log++;
            } else {
                curMotor->high_speeds_default_log++;
            }   
            
            // shifts the predicted rev to have it's zero centered at last rev, range: [0,1]
            float predictedRevCenteredAtLast = (predictedRev - static_cast<int>(predictedRev) + 1);
            predictedRevCenteredAtLast -= static_cast<int>(predictedRevCenteredAtLast);

            // shifts the actual rev to have it's zero centered at last rev, range: [0,1]
            float actualRevCenteredAtLast = actualRev + 2 - lastRev;
            actualRevCenteredAtLast -= static_cast<int>(actualRevCenteredAtLast);
            
            // amount of revolutions we think we will jump by
            int revolutionJump = static_cast<int>(predictedRev);

            /*
                Split by speed direction (based on predicted rev)
                For positive:
                    If our predicted revoliution's angle is ahead of last 
                    recorded angle but our actual rev is behind last recored angle
                    our predicted amount of revolution is too high

                    If our predicted rev is behind our last recorded angle
                    but our actual rev is ahead, that means our predicted amount of
                    revolution is too low and needs to be increased

                For Negative:
                    Same as positive but reversed
                     
            */
            if(predictedRev > 0) {
                if(predictedRevCenteredAtLast < 0.25f && actualRevCenteredAtLast > 0.75f) {
                    // overshoot by one revolution
                    revolutionJump--;
                } else if(predictedRevCenteredAtLast > 0.75f && actualRevCenteredAtLast < 0.25f) {
                    // undershoot by one revolution
                    revolutionJump++;
                }
            } else if(predictedRev < 0){
                if(predictedRevCenteredAtLast > 0.75f && actualRevCenteredAtLast < 0.25f) {
                    // overshoot by one revolution
                    revolutionJump++;
                } else if(predictedRevCenteredAtLast < 0.25f && actualRevCenteredAtLast > 0.75f) {
                    // undershoot by one revolution
                    revolutionJump--;
                }
            }

            delta_angle += revolutionJump*ENCODER_WRAP;
            curMotor->multiTurn += delta_angle; 
        }
        
        // Update new multiturn
        curMotor->lastMotorAngle = cur_angle;
        curMotor->lastRps = raw_rps;
        curMotor->lastIntegrationTime = static_cast<long>(time);

        curMotor->buffer1[curMotor->log_index] = curMotor->multiTurn;
        curMotor->buffer2[curMotor->log_index] = cur_angle;
        curMotor->buffer3[curMotor->log_index] = static_cast<long>(time);
        curMotor->buffer4[curMotor->log_index] = raw_speed;
        curMotor->log_index = (curMotor->log_index + 1) % curMotor->log_size;
        
    }
}

void DJIMotor::zeroTicks() {
    tickOffset = static_cast<int16_t>(getData(ANGLE_RAW));
}

float DJIMotor::getData(motorDataType data) {
    if (data <= 3) {
        if (data == VELOCITY && type == M3508) {
            return ((float)(motorData[data]) / M3508_GEAR_RATIO) * 2 * PI / 60;
        }
        else if (data == VELOCITY) {
            return (float)(motorData[data]) * 2 * PI / 60;
        } else if(data == ANGLE) {
            return (float)((motorData[data] - tickOffset + ENCODER_WRAP) % ENCODER_WRAP);
        }
        return (float)(motorData[data]);
    }

    else if (data == MULTITURNANGLE)
        return (float)multiTurn;

    else if (data == POWEROUT)
        return (float)powerOut;

    else if (data == VALUE)
        return (float)value;

    else if(data == VELOCITY_RAW) {
        return (float)(motorData[VELOCITY]);  
    } 

    return 0;
}

int DJIMotor::calculateDeltaPhase(int target, int current, int max) {
    target %= max;

    int deltaPhase = target - current;

    if (abs(deltaPhase) > max / 2) {
        if (deltaPhase > 0)
            deltaPhase -= max;
        else
            deltaPhase += max;
    }
    return deltaPhase;
}

float DJIMotor::calculateDeltaPhase(float target, float current, float max) {
    // target %= max;

    float deltaPhase = target - current;

    if (fabsf(deltaPhase) > max / 2) {
        if (deltaPhase > 0)
            deltaPhase -= max;

        else
            deltaPhase += max;
    }
    return deltaPhase;
}
