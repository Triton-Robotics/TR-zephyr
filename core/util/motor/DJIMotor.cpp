
#include "DJIMotor.h"
#include "CANHandler.h"


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


// TODO: Fix again
void DJIMotor::updateOneMultiTurn(int canBus, int can_line, int motor_id) {
    /** 
        8192 is encoder reading
        1/1000 is microsec to millisec
        integratedAngle = (-4096, 4096)
    **/

    int Threshold = 3000; // From 0 to 8191
    int ENCODER_WRAP = 8192;
    int curAngle, lastAngle, deltaAngle;
    float speed, raw_speed;
    DJIMotor *curMotor;

    unsigned long time = now_us();

    if (s_motorsExist[canBus][can_line][motor_id]) {
        curMotor = s_allMotors[canBus][can_line][motor_id];

        // Update data from most recent frame
        curAngle = curMotor->getData(ANGLE);
        lastAngle = curMotor->lastMotorAngle;
        speed = curMotor->getData(VELOCITY);
        raw_speed = curMotor->getData(VELOCITY_RAW);
        deltaAngle = curAngle - lastAngle;

        // Update multiturn data using latest data
        double dt = 0;
        // gets delta time
        if (curMotor->lastIntegrationTime != -1) {
            dt = (double)(time - curMotor->lastIntegrationTime) / 1000.0; // ms
        }

        // rpm * dt => rads travled * 19 * 6 = outer shell traveled
        curMotor->integratedAngle += (int)(speed * dt * 19 * 6 / 1000.0);
        // gets difference in current angle and calculated angle 
        // integratedAngle % 8192 is just to keep in bounds
        int positiveDiff = curAngle - curMotor->integratedAngle % 8192;

        if (positiveDiff >= 0 && positiveDiff < 4096) {
            // integrated angle is behind but does not need to loop
            // set integrated = positiveDiff
            curMotor->integratedAngle += positiveDiff;
        } else if (positiveDiff < 0 && positiveDiff > -4096) {
            // integrated angle is ahead and does not need to loop
            // set integrated = positiveDiff
            curMotor->integratedAngle += positiveDiff;
        } else if (positiveDiff >= 4096) { 
            // integrated angle is ahead and does need to loop
            // loop around
            curMotor->integratedAngle += positiveDiff - 8192;
        } else if (positiveDiff <= -4096) {
            // integrated angle is ahead and does need to loop
            // loop around
            curMotor->integratedAngle += positiveDiff + 8192;
        } else {
            // printf("Don't know what to do with this %i\n", positiveDiff);
        }

        // "slow speed mode"
        if (abs(speed) < 0.5) {
            // speed is negative, deltaAngle = positive, so make negative to do neg skip
            // if (curAngle > (ENCODER_WRAP - Threshold) && lastAngle < Threshold) {
            //     curMotor->multiTurn += deltaAngle - ENCODER_WRAP; // 
            //     curMotor->high_speeds_case_one_false++;
            // }
            // else if (curAngle < Threshold && lastAngle > (ENCODER_WRAP - Threshold)) {
            //     curMotor->multiTurn += deltaAngle + ENCODER_WRAP; // speed is positive, deltaAngle = negative, so make angle do pos skip
            //     curMotor->high_speeds_case_two_false++;
            // }
            // else {
            //     curMotor->multiTurn += deltaAngle; // Regular
            //     curMotor->high_speeds_case_three_false++;
            // }

            if (speed < 0 && deltaAngle > Threshold) { 
                // speed is backwards, deltaAngle = positive, so make do neg skip
                // Only do neg skip if delta angle is greater than a threshold
                // that way can eliminate small
                curMotor->multiTurn += deltaAngle - ENCODER_WRAP;
                curMotor->high_speeds_neg_skip_log++;
            } else if (speed > 0 && deltaAngle < Threshold) { 
                // speed is forwards, deltaAngle is negative, so make do pos skip
                curMotor->multiTurn += deltaAngle + ENCODER_WRAP;
                curMotor->high_speeds_pos_skip_log++;
            } else {
                curMotor->multiTurn += deltaAngle; // Regular
                curMotor->high_speeds_default_log++;
            }
        } else {
            if (speed < 0 && deltaAngle > 0) { 
                // speed is backwards, deltaAngle = positive, so make do neg skip
                // printf("positive overclock: %i\n", (int) speed);
                curMotor->multiTurn += deltaAngle - ENCODER_WRAP;
                curMotor->high_speeds_neg_skip_log++;
            } else if (speed > 0 && deltaAngle < 0) { 
                // speed is forwards, deltaAngle is negative, so make do pos skip
                // printf("negative overclock: %i\n", (int) speed);
                curMotor->multiTurn += deltaAngle + ENCODER_WRAP;
                curMotor->high_speeds_pos_skip_log++;
            } else {
                curMotor->multiTurn += deltaAngle; // Regular
                curMotor->high_speeds_default_log++;
            }
        }

        curMotor->function_call_counter++;

        curMotor->high_speeds_last_angle = lastAngle;
        curMotor->high_speeds_curr_angle = curAngle;
        curMotor->high_speeds_velocity = speed;
        curMotor->high_speeds_velocity_raw = raw_speed;
        curMotor->high_speeds_delta_angle = deltaAngle;
        curMotor->high_speeds_multiturn_angle = curMotor->multiTurn;
        curMotor->curr_time = time;

        // log current data into data
        curMotor->last_multiturn_angles_log[curMotor->log_index] = curMotor->high_speeds_multiturn_angle;
        curMotor->last_encoder_angles_log[curMotor->log_index] = curAngle;
        curMotor->last_time_log[curMotor->log_index] = raw_speed; //now_us();

        curMotor->log_index = (curMotor->log_index + 1) % curMotor->log_size;
        
        // Update new multiturn
        curMotor->lastMotorAngle = curAngle;
        curMotor->lastIntegrationTime = long(time);

        if (abs(speed) < 100) {
            if (curAngle > (ENCODER_WRAP - Threshold) && lastAngle < Threshold) {
                curMotor->multiTurnLowSpeeds += deltaAngle - ENCODER_WRAP;
                curMotor->low_speeds_neg_skip_log++;
            }
            else if (curAngle < Threshold && lastAngle > (ENCODER_WRAP - Threshold)) {
                curMotor->multiTurnLowSpeeds += deltaAngle + ENCODER_WRAP; // speed is positive, deltaAngle = negative, so make angle do pos skip
                curMotor->low_speeds_pos_skip_log++;
            }
            else {
                curMotor->multiTurnLowSpeeds += deltaAngle; // Regular
                curMotor->low_speeds_default_log++;
            }
        } else {
            if (speed < 0 && deltaAngle > 0) { 
                // speed is backwards, deltaAngle = positive, so make do neg skip
                // printf("positive overclock: %i\n", (int) speed);
                curMotor->multiTurnLowSpeeds += deltaAngle - ENCODER_WRAP;
                curMotor->low_speeds_case_one_false++;
            } else if (speed > 0 && deltaAngle < 0) { 
                // speed is forwards, deltaAngle is negative, so make do pos skip
                // printf("negative overclock: %i\n", (int) speed);
                curMotor->multiTurnLowSpeeds += deltaAngle + ENCODER_WRAP;
                curMotor->low_speeds_case_two_false++;
            } else {
                curMotor->multiTurnLowSpeeds += deltaAngle; // Regular
                curMotor->low_speeds_case_three_false++;
            }
        }

        curMotor->low_speeds_last_angle = lastAngle;
        curMotor->low_speeds_curr_angle = curAngle;
        curMotor->low_speeds_velocity = speed;
        curMotor->low_speeds_delta_angle = deltaAngle;
        curMotor->low_speeds_multiturn_angle = curMotor->multiTurnLowSpeeds;

        // // revolution catch
        // float avgSpeed = (speed - curMotor->lastMotorSpeedFancy) / 2.0;
        // float calculatedRotation = fabs(avgSpeed) * (time - curMotor->lastMotorTimeFancy);
        // // If our calculated rotation is 
        // if((abs(deltaAngle) < Threshold && calculatedRotation > (8191 - Threshold))) {
        //     if(speed < 0 && deltaAngle < 0) {
        //         // speed is backwards and deltaAngle shows backwards even though we went a rotation, then we missed a rotation with our prev add
        //         curMotor->multiTurnFancy -= 8191;
        //     } else if (speed > 0 && deltaAngle > 0){
        //         // speed is forwards and deltaAngle shows forwards even though we went a rotation, then we missed a rotation with our prev add
        //         curMotor->multiTurnFancy += 8191;
        //     }
        // }

        // ensures we only update what last valid time as long as angle has moved OR we have not moved
        if(curMotor->lastMotorAngleLowSpeeds != curAngle || speed == 0) {
            curMotor->lastMotorTimeLowSpeeds = long(time);
        }

        curMotor->lastMotorSpeedLowSpeeds = speed;
        curMotor->lastMotorAngleLowSpeeds = curAngle;
    }
}

float DJIMotor::getData(motorDataType data) {
    if (data <= 3) {
        if (data == VELOCITY && type == M3508) {
            return ((float)(motorData[data]) / M3508_GEAR_RATIO) * 2 * PI / 60;
        }
        else if (data == VELOCITY) {
            return (float)(motorData[data]) * 2 * PI / 60;
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
