#include "MyStepperController.h"

MyStepperController::MyStepperController(UINT stepsPerRev, SHORT pinStep, SHORT pinDir)
    : stepsPerRev(stepsPerRev),pinStep(pinStep), pinDir(pinDir)
{
    pinMode(pinStep, OUTPUT);
    pinMode(pinDir, OUTPUT);
    digitalWrite(pinDir, LOW);
    digitalWrite(pinStep, LOW);
}

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

    // Serial.print("\n at pos: ");
    // Serial.print(stepCount);
    // Serial.print("\n goto pos: ");
    // Serial.print(targetStep);
}

void MyStepperController::setTargetSpeed(FLOAT ts)
{
    if (ctrlMode != POSITIONING)
        targetSpeed = ts;
}

void MyStepperController::setMaxSpeed(FLOAT speed)
{
    maxSpeed = abs(speed);
}

void MyStepperController::setAcceleration(UINT accel)
{
    acceleration = accel;
}

LONG MyStepperController::getStepCount(void)
{
    return stepCount;
}

FLOAT MyStepperController::getCurrentSpeed(void)
{
    return currSpeed;
}

BOOL MyStepperController::isRunning(void)
{
    return (STOP != rotDir);
}

void MyStepperController::stop(void)
{
    setTargetSpeed(0.0);
    ctrlMode = SPEED_CTRL;
}

CTRL_MODE  MyStepperController::getCtrlMode(void)
{
    return ctrlMode;
}

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

void MyStepperController::setLimitProtectionEnabled(BOOL enbl)
{
    limitProtectionEnabled = enbl;
}

void MyStepperController::doStep(DIRECTION direction)
{
    switch (direction)
    {
        case STOP:
            return;
        case CLOCKWISE:
            /* write DIR pin */
            digitalWrite(pinDir, LOW);
            stepCount++;
            break;
        case COUNTERCLOCKWISE:
            /* write DIR pin */
            digitalWrite(pinDir, HIGH);
            stepCount--;
            break;
    }

    /* create pulse on STEP */
    digitalWrite(pinStep, HIGH);
    delayMicroseconds(PULSE_DURATION);
    digitalWrite(pinStep, LOW);
    

}

void MyStepperController::runSpeed(void)
{
    static ULONG tPulseBegin; // time when the last step pulse has been done
    ULONG time = micros();

    if(rotDir == STOP)
        n = 0; // used for speed calculation of the first step
    if (time - tPulseBegin >= tPeriod)
    {
        doStep(rotDir);        
        tPeriod = getNewPeriod();
        tPulseBegin = time;
    }    
}

ULONG MyStepperController::getNewPeriod()
{
    static ULONG rest;
    FLOAT fnew, f;
    ULONG tPeriodNew;
    INT i;

    ULONG currfrequency = abs(currSpeed);

    if (rotDir != STOP)
    {
        if(n==0)
        {
            /* calculate the first period of acceleration according to the approach introduced in
             * http://ww1.microchip.com/downloads/en/AppNotes/doc8017.pdf because otherwise the
             * first period will be very long if accerlation is slow in the first moment.
             * However I will not follow the whole approach since I want to change speed more
             * frequent and a the simple period calculation T= 1s/f works sufficient */
            tPeriodNew = (FLOAT)SECOND_US*sqrt(2.0f/(FLOAT)acceleration);
        }
        else
        {
            /* use simple period calculation T= 1s/f after first period */ 
            tPeriodNew = ((FLOAT)SECOND_US / (FLOAT)currfrequency);
        }

        n++;
    }
    else
    {
        tPeriodNew = 0;
    }

    return tPeriodNew;
}

/* calculates the current speed depending on the user given target speed
 * considering the acceleration */
void MyStepperController::calcSpeed(void)
{
    /* 1. if we want to run to a position we just modify the target speed
     * to the max allowed speed */
    if (POSITIONING == ctrlMode)
    {
        if (stepCount < targetPos)
            targetSpeed = maxSpeed;
        else if (stepCount > targetPos)
            targetSpeed = -maxSpeed;
        else /* target position reached, go back to SPEED_CTRL */
            ctrlMode = SPEED_CTRL;
    }
    
    /* limit target speed to max speed */
    targetSpeed = constrain(targetSpeed, -maxSpeed, maxSpeed);

    /* 2. next step is to gradually ramp the current speed to targetspeed */
    rampSpeed();

    /* 3. after that we reduce the speed in case we get near the limits if 
     * limit pretection is  enabled */
    if (true == limitProtectionEnabled)
    {
        if(currSpeed > 0.0f)
            currSpeed = limitSpeed(upperLimit - stepCount);
        else
            currSpeed = limitSpeed(lowerLimit - stepCount);
    }
    /* 4. and if we are running towards a position we do the same with the
     * target position instead of the limit */
    if (POSITIONING == ctrlMode)
        currSpeed = limitSpeed(targetPos - stepCount);

}

void MyStepperController::rampSpeed(void)
{
    /* generate a linar ramp up/down */
    static ULONG tlastRun = 0;
    ULONG time = millis();
    ULONG dt = time - tlastRun;

    if (dt != 0)
    {
        if (currSpeed < targetSpeed)
        {
            /* accelerate: v = v0 + a*dt */
            currSpeed += (dt/SECOND_MS * acceleration) ;
            /* limit to targetspeed */
            currSpeed = min(currSpeed, targetSpeed);
        }
        else
        {
            /* decellerate: v = v0 - a*dt */
            currSpeed -= (dt/SECOND_MS * acceleration);
            /* limit to targetspeed */
            currSpeed = max(currSpeed, targetSpeed);
        }

        tlastRun = time;
    }

}

FLOAT MyStepperController::limitSpeed(LONG distToGo)
{
    FLOAT retVal;
    FLOAT limitedSpeed;
    
    if (distToGo != 0)
    {
        limitedSpeed = sqrt(2 * abs(distToGo) * acceleration);
        /* if speed and distance have same sign then speed needs to be limited */
        if( currSpeed > 0.0 && distToGo > 0)
            retVal = min(limitedSpeed, currSpeed);
        else if ( currSpeed < 0.0 && distToGo < 0)
            retVal = max(-limitedSpeed, currSpeed);
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
    /* since speed variables are FLOAT which is never exactly zero we need to have a threshold
     * below the motor speed shall be considered as zero */
    if (abs(currSpeed) < (FLOAT)STOP_THRESHOLD) 
    {
        rotDir = STOP;
    }
    else if (currSpeed > 0.0f)
    {
        rotDir = CLOCKWISE;
    }
    else
    {
        rotDir = COUNTERCLOCKWISE;
    }
}