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
    if (ctrlMode != POSITIONING)
        targetSpeed = ts;
}

/* set max allowed speed (steps/s) */
void MyStepperController::setMaxSpeed(FLOAT maxSpeed)
{
    maxSpeed = abs(maxSpeed);
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

BOOL MyStepperController::isRunning(void)
{
    return (STOP != rotDir);
}

void MyStepperController::stop(void)
{
    setTargetSpeed(0.0);
    ctrlMode = SPEED_CTRL;
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
    static ULONG tPulseBegin;   // time when the last step pulse has been done
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
        if (currSpeed < targetSpeed)
        {
            if(n==0)
            {
                tPeriodNew = (FLOAT)SECOND_US*sqrt(2.0f/(FLOAT)acceleration);
            }
            else
            {
                // tPeriodNew = tPeriod - (2*tPeriod+rest)/(4*n+1);
                // rest = (2*tPeriod+rest)%(4*n+1);
                tPeriodNew = ((FLOAT)SECOND_US / (FLOAT)currfrequency);
            }
            n++;
        }
        else if (currSpeed == targetSpeed)
        {
            tPeriodNew = tPeriod;
        }
        else /*decellerate*/
        {
            if(n==0)
            {
                tPeriodNew = (FLOAT)SECOND_US*sqrt(2.0f/(FLOAT)acceleration);
            }
            else
            {
                // tPeriodNew = tPeriod + (2*tPeriod+rest)/(4*n+1);
                // rest = (2*tPeriod+rest)%(4*n+1);

                tPeriodNew = ((FLOAT)SECOND_US / (FLOAT)currfrequency);
            }
            n--;
        }
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
    else
    {
        /* limit target speed to max speed */
        targetSpeed = constrain(targetSpeed, -maxSpeed, maxSpeed);
    }

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
    }
    else
    {
        rotDir = COUNTERCLOCKWISE;
    }
}