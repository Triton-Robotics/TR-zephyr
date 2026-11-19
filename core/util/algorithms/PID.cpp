#include "PID.h"

#ifndef FUNNY_PID

PID::PID()
{
    kP = 0;
    kI = 0;
    kD = 0;
    integralCap = 0;
    outputCap = 0;
    feedForward = 0;
}

PID::PID(float kP, float kI, float kD, float integralCap, float outputCap)
{
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->integralCap = integralCap;
    this->outputCap = outputCap;
}

void PID::setPID(float kP, float kI, float kD, float integralCap, float outputCap)
{
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->integralCap = integralCap;
    this->outputCap = outputCap;
}

void PID::resetPID(float kP, float kI, float kD, float integralCap, float outputCap)
{
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->integralCap = integralCap;
    this->outputCap = outputCap;

    lastError = 0;
    errorIntegral = 0;
}

void PID::resetErrorIntegral()
{
    lastError = 0;
    errorIntegral = 0;
}

int PID::calculate(int desired, int current, float dt)
{
    float error = static_cast<float>(desired - current);
    return calculatePeriodic(error, dt);
}

int PID::calculatePeriodic(float error, float dt)
{

    dt /= 1000;
    errorIntegral += kI * dt * (error + lastError) / 2;
    limitErrorIntegral();

    iC = errorIntegral;

    float PIDCalc = (kP * error) + (errorIntegral) + feedForward;
    pC = kP * error;

    if (dt > 0)
    {
        PIDCalc += (kD * (error - lastError) / dt);
    }

    dC = (kD * (error - lastError) / dt);

    lastError = error;
    limitOutput(PIDCalc);
    return static_cast<int>(PIDCalc);
}

void PID::limitErrorIntegral()
{
    if (integralCap != 0)
    {
        if (errorIntegral > integralCap)
            errorIntegral = integralCap;

        else if (errorIntegral < -integralCap)
            errorIntegral = -integralCap;
    }
}

void PID::limitOutput(float &PIDCalc) const
{

    if (outputCap != 0)
    {
        if (PIDCalc > outputCap)
            PIDCalc = outputCap;

        else if (PIDCalc < -outputCap)
            PIDCalc = -outputCap;
    }
}

void PID::setIntegralCap(float integralCap)
{
    this->integralCap = integralCap;
}

void PID::setOutputCap(float outputCap)
{
    this->outputCap = outputCap;
}

#else

#include "PID.h"

//constructor default
//why there is no this->
PID::PID()
{
    kP = 0;
    kI = 0;
    kD = 0;
    integralCap = 0;
    outputCap = 0;
    feedForward = 0;
}

//constructor
PID::PID(float kP, float kI, float kD, float integralCap, float outputCap)
{
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->integralCap = integralCap;
    this->outputCap = outputCap;
}

//constructor config
PID::PID(PID::config cfg)
{
    kP = cfg.p;
    kI = cfg.i;
    kD = cfg.d;
    integralCap = cfg.integralCap;
    outputCap = cfg.outCap;
}

void PID::setPID(float kP, float kI, float kD, float integralCap, float outputCap)
{
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->integralCap = integralCap;
    this->outputCap = outputCap;
}

void PID::resetPID(float kP, float kI, float kD, float integralCap, float outputCap)
{
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->integralCap = integralCap;
    this->outputCap = outputCap;

    lastError = 0;
    errorIntegral = 0;
}

void PID::resetErrorIntegral()
{
    lastError = 0;
    errorIntegral = 0;
}

float PID::calculate(float desired, float current, float dt_ms)
{
    float error = static_cast<float>(desired - current);//the current difference
    return calculatePeriodic(error, dt_ms);
}

float PID::calculatePeriodic(float error, float dt_ms)//The output is actually the power output
{
    dt_ms /= 1000; // Convert to seconds
    //This is the method to calculate the integration
    errorIntegral += kI * dt_ms * (error + lastError) / 2;
    limitErrorIntegral();

    iC = errorIntegral;//integral conponent

    float PIDCalc = (kP * error) + (errorIntegral) + feedForward;// what does feedForward do?
    pC = kP * error; //proportional component

    float dTerm = 0;//the derivative term
    if (dt_ms > 0)
    {
        dTerm = (kD * (error - lastError) / dt_ms);//calculate the rate of change within dt
        //dBuffer is a lastfew struct
        dBuffer.add(dTerm);
        dTerm = dBuffer.time();//neutralized with the previous derivative
        //see line110 the lastY_ which sets the amount of derivative to neutralize
        PIDCalc += dTerm;
    }

    dC = dTerm;

    lastError = error;
    limitOutput(PIDCalc);
    return static_cast<int>(PIDCalc);
}

void PID::limitErrorIntegral()//it tests the two ends of the limit.
//errorIntegral is the integral of the error
{
    if (integralCap != 0)
    {
        if (errorIntegral > integralCap)
            errorIntegral = integralCap;

        else if (errorIntegral < -integralCap)
            errorIntegral = -integralCap;
    }
}

void PID::limitOutput(float &PIDCalc) const
{

    if (outputCap != 0)
    {
        if (PIDCalc > outputCap)
            PIDCalc = outputCap;

        else if (PIDCalc < -outputCap)
            PIDCalc = -outputCap;
    }
}

void PID::setIntegralCap(float integralCap)
{
    this->integralCap = integralCap;
}

void PID::setOutputCap(float outputCap)
{
    this->outputCap = outputCap;
}

#endif