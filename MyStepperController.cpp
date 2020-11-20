#include "MyStepperController.h"

MyStepperController::MyStepperController(UINT stepsPerRev, SHORT pinStep, SHORT pinDir)
    : stepsPerRev(stepsPerRev),pinStep(pinStep), pinDir(pinDir)
{
    pinMode(pinStep, OUTPUT);
    pinMode(pinDir, OUTPUT);
    digitalWrite(pinDir, LOW);
    digitalWrite(pinStep, LOW);
}

/* main method for calculating the outputs needed for driving to stepper 
   * should be called cyclic as often as possible */
void MyStepperController::run()
{
    calcSpeed();
    calcDirection();
    runSpeed();
}

void MyStepperController::moveTo(LONG targetStep)
{
    ctrlMode = POSITIONING;
    targetPos = targetStep;

    Serial.print("\n at pos: ");
    Serial.print(stepCount);
    Serial.print("\n goto pos: ");
    Serial.print(targetStep);
}

/* set target speed (steps/s) 
   * positive valies result in cw rotation, negative ones in ccw */
void MyStepperController::setTargetSpeed(FLOAT ts)
{
    targetSpeed = ts;
}

/* set max allowed speed (steps/s) */
void MyStepperController::setMaxSpeed(FLOAT maxSpeed)
{
    this->maxSpeed = maxSpeed;
}

/* set acceleration (steps/s^2) */
void MyStepperController::setAcceleration(UINT accel)
{
    acceleration = accel;
}

/* returns current position */
LONG MyStepperController::getStepCount(void)
{
    return stepCount;
}

/* returns current speed */
FLOAT MyStepperController::getCurrentSpeed(void)
{
    return currSpeed;
}
/* returns control mode speed */
CTRL_MODE  MyStepperController::getCtrlMode(void)
{
    return ctrlMode;
}

/* returns rotation direction */
DIRECTION MyStepperController::getRotDir(void)
{
    return rotDir;
}

void MyStepperController::setLowerLimit(LONG pos)
{
    lowerLimit = pos;
}

void MyStepperController::setUpperLimit(LONG pos)
{
    upperLimit = pos;
}

void MyStepperController::setStepCount(LONG pos)
{
    stepCount = pos;
}

void MyStepperController::reset(void)
{
    stepCount = 0;
}

void MyStepperController::doStep(DIRECTION direction)
{
    switch (direction)
    {
    case STOP:
        digitalWrite(pinStep, LOW);
        return;
    case CLOCKWISE:
        // write DIR pin
        digitalWrite(pinDir, LOW);
        stepCount++;
        break;
    case COUNTERCLOCKWISE:
        // write DIR pin
        digitalWrite(pinDir, HIGH);
        stepCount--;
        break;
    }

    // create pulse on STEP
    digitalWrite(pinStep, HIGH);
    delayMicroseconds(PULSE_DURATION);
    digitalWrite(pinStep, LOW);
    

}

void MyStepperController::runSpeed(void)
{
    ULONG tCurrent = micros();
    tPeriod = getNewPeriod();

    if (tCurrent - tPulseBegin >= tPeriod)
    {
        doStep(rotDir);        
        tPulseBegin = tCurrent;
    }
}

ULONG MyStepperController::getNewPeriod()
{
    // convert frequency into period
    if (rotDir != STOP)
        return ((FLOAT) SECOND / (FLOAT)currfrequency);
    else
        // avoid very high numbers if stopped
        return 0;
}

/* calculates the current speed depending on the user given target speed
   * considering the acceleration */
void MyStepperController::calcSpeed(void)
{
    /*  distance to get to Stop = curretSpeed^2 / 2*a */
    // LONG brakeDist = currSpeed * currSpeed / (2 * acceleration) + 1;
    if (POSITIONING == ctrlMode)
    {
        if (stepCount < targetPos)
        {
            targetSpeed = maxSpeed;
        }
        else if (stepCount > targetPos)
        {
            targetSpeed = -maxSpeed;
        }
        else // stepCount == targetPos
        {
            ctrlMode = END_PROTECTION;
        }
    }
    else
    {
        /* limit target speed to max speed */
        targetSpeed = constrain(targetSpeed, -maxSpeed, maxSpeed);
    }

    /* ramp current speed to targetspeed */
    rampSpeed(targetSpeed);

    /* end protection with upper/lower limit: limit current speed*/
    if (UNLIMED != ctrlMode)
    {
        if(currSpeed > 0.0f)
            currSpeed = limitSpeed(currSpeed, upperLimit - stepCount);
        else
            currSpeed = limitSpeed(currSpeed, lowerLimit - stepCount);
    }

    if (POSITIONING == ctrlMode)
    {
        currSpeed = limitSpeed(currSpeed, targetPos - stepCount);
    }

    // absolute value of frequency is passed to STEP pin impuls generator
    currfrequency = abs(currSpeed);
}

void MyStepperController::rampSpeed(FLOAT targetSpeed)
{
    /* generate a linar ramp up/down */
    static ULONG tlastRun = 0;
    ULONG tCurrent = micros();
    
    /* v = v0 + a*dt */
    FLOAT dSpeed= (FLOAT)((tCurrent - tlastRun) * acceleration) / SECOND;

    if (currSpeed < targetSpeed)
    {
        currSpeed += dSpeed;
        // limit to targetspeed
        currSpeed = min(currSpeed, targetSpeed);
    }
    else
    {
        currSpeed -= dSpeed;
        // limit to targetspeed
        currSpeed = max(currSpeed, targetSpeed);
    }

    tlastRun = tCurrent;
}

FLOAT MyStepperController::limitSpeed(FLOAT currSpeed, LONG distToGo)
{
    FLOAT retVal;
    FLOAT maxSpeed;
    
    if (distToGo != 0)
    {
        maxSpeed = sqrt(2 * abs(distToGo) * acceleration);
        /* if speed and distance have same sign then speed needs to be limited */
        if( currSpeed > 0.0 && distToGo > 0)
            retVal = min(maxSpeed, currSpeed);
        else if ( currSpeed < 0.0 && distToGo < 0)
            retVal = max(-maxSpeed, currSpeed);
        else
            retVal = currSpeed;
    }
    else
    {
        retVal = 0.0f;
    }
    
    return retVal;
    
}

void MyStepperController::calcDirection(void)
{
    // since speed variables are FLOAT we need to have a threshold below
    // the motor speed shall be considered as zero

    if (abs(currSpeed) < (FLOAT)STOP_THRESHOLD)
    {
        rotDir = STOP;
    }
    else if (currSpeed > 0.0f)
    {
        rotDir = CLOCKWISE;
        digitalWrite(pinDir, LOW);
    }
    else
    {
        rotDir = COUNTERCLOCKWISE;
        digitalWrite(pinDir, HIGH);
    }
}