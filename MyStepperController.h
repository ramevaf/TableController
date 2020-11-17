#ifndef MYSTEPPERCONTROLLER_H
#define MYSTEPPERCONTROLLER_H

#include "types.h"

#define PULSE_DURATION 50 // 50 µs pulse on step 
#define STOP_THRESHOLD 0.05
#define SECOND 1000000

enum direction
{
  STOP,
  CLOCKWISE,
  COUNTERCLOCKWISE
};

class MyStepperController
{

public:
  MyStepperController(UINT stepsPerRev, SHORT pinStep, SHORT pinDir)
  {
    this->pinStep = pinStep;
    this->pinDir = pinDir;
    this->stepsPerRev = stepsPerRev;
    pinMode(pinStep, OUTPUT);
    pinMode(pinDir, OUTPUT);

    digitalWrite(pinStep, LOW);
  }

  void run()
  {
    tCurrent = micros();
    calcSpeed();
    calcDirection();
    createStepPulse();
    // Serial.print("\n T: ");
    // Serial.print(T);    
    // Serial.print("\n currTimeStamp: ");
    // Serial.print(currTimeStamp);
    // Serial.print("\n tPulseBegin: ");
    // Serial.print(tPulseBegin);
    tlastRun = tCurrent;
  }

  void calcSpeed(void)
  {
    ULONG dt = tCurrent-tlastRun;
    FLOAT dSpeed;
    
    if (currSpeed < targetSpeed)
    {
      dSpeed =  (((FLOAT)dt)/SECOND)*acceleration;
      currSpeed += (((FLOAT)dt)/SECOND)*acceleration;
    }
    else
    {
      currSpeed -= (((FLOAT)dt)/SECOND)*acceleration;
    }
    
    currfrequency = abs(currSpeed);

    // Serial.print("\n currSpeed: ");
    // Serial.print(currSpeed);
    // Serial.print("\n currfrequency: ");
    // Serial.print(currfrequency);
    
  }

  void calcDirection(void)
  {
    // speed threshold below the motor shall be considered as stopped
    FLOAT stopTshld = STOP_THRESHOLD*acceleration;

    if (abs(currSpeed) < stopTshld)
    {
      rotDir = STOP;
      digitalWrite(pinDir, LOW);
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

    // Serial.print("\n stopTshld: ");
    // Serial.print(stopTshld);
    
  }

  /* gets a frequency and creates pulses on the step
   * pin. Needs to be called cyclic */
  void createStepPulse()
  {
    // if motor is stopped we don't want to get any random pulses
    if (rotDir != STOP)
    {
      // convert frequency into period
      ULONG T = SECOND/(currfrequency);

      if (LOW == pinStepSts) 
      {
        if (tCurrent - this->tPulseBegin >= T) 
        {
          // new period begins with HIGH
          digitalWrite(pinStep, HIGH);
          pinStepSts = HIGH;
          // save current time as period start
          this->tPulseBegin = tCurrent;

          // count steps
          if(CLOCKWISE == rotDir) stepCount++;
          else if(COUNTERCLOCKWISE == rotDir) stepCount--;
        }
      }
      else
      {
        // set level to LOW after PULSE_DURATION
        if (tCurrent - (this->tPulseBegin) >= PULSE_DURATION)
        {
          digitalWrite(pinStep, LOW);
          pinStepSts = LOW;
        }
      }
    }
    else
    {
      // set to LOW if motor shall be stopped
      digitalWrite(pinStep, LOW);
      pinStepSts = LOW;
    }
    
  }

  void setTargetSpeed(FLOAT ts)
  {
    targetSpeed = ts;
  }

  void setAcceleration(FLOAT accel)
  {
    acceleration = accel;
  }


  LONG getStepCount(void)
  {
    return stepCount;
  }

  void setStepCount(LONG pos)
  {
    stepCount = pos;
  }

  void reset(void)
  {
    stepCount = 0;
  }

  void stepClockwise(void)
  {
  }

  void stepCounterClockwise(void)
  {
  }

  void setStepperSpeed(UINT motSpeed)
  {
  }

  void gotoPosition(LONG targetStep)
  {
    while (stepCount != targetStep)
    {
      if (stepCount > targetStep)
      {
        stepCounterClockwise();
      }
      else
      {
        stepClockwise();
      }      
    }
  }

private:
  LONG stepCount = 0;
  USHORT pinStep;
  USHORT pinDir;
  UINT stepsPerRev;
  FLOAT currSpeed; // speed in 1/s
  FLOAT targetSpeed; // speed in 1/s
  FLOAT currfrequency; // frequency in 1/s ONLY POSITIVE!
  UINT acceleration = 50;
  USHORT pinStepSts = LOW;
  
  ULONG tCurrent; // local variable for temp saving micros()
  // saved timestamps
  ULONG tlastRun;
  ULONG tPulseBegin;
  direction rotDir;
  
  
};

#endif
