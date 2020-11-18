#include "MyStepperController.h"


  MyStepperController::MyStepperController(UINT stepsPerRev, SHORT pinStep, SHORT pinDir) 
            : stepsPerRev(stepsPerRev), pinStep(pinStep), pinDir(pinDir)
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
    LONG distToTarget;

    tCurrent = micros(); 

    if (POSITIONING == controlMode)
    {
      distToTarget = targetPos - stepCount;
      calcSpeedPositioning(distToTarget);
    }
    else // (SPEED_CTRL == controlMode)
    {
      calcSpeedFreeRun();
    }    
    calcDirection();
    createStepPulse();

    // if target positiona is reached return to SPEED_CTRL mode
    if (stepCount == targetPos) controlMode = SPEED_CTRL;

    tlastRun = tCurrent;
  }

  void MyStepperController::moveTo(LONG targetStep)
  {
    controlMode = POSITIONING;
    targetPos = targetStep;
    startingPos = stepCount;
    
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

  void MyStepperController::stepClockwise(void)
  {
  }

  void MyStepperController::stepCounterClockwise(void)
  {
  }

  /* calculates the current speed depending on the user given target speed
   * considering the acceleration */
  void MyStepperController::calcSpeedFreeRun(void)
  {
    LONG distToLimit;
    // speed variables needs to be float since dSpeed could be below
    // 1 step/s for very slow speeds
    FLOAT dt = tCurrent-tlastRun;
    FLOAT dSpeed =  (dt*acceleration)/SECOND;
    // distance to get to Stop = curreSpeed^2 / 2*a
    LONG brakeDist = currSpeed*currSpeed/(2*acceleration)+1;
    // limit target speed to max speed
    targetSpeed = constrain(targetSpeed, -maxSpeed, maxSpeed);

    // generate speed ramp considering accleration
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

    // calculate distance to upper or lower limit depending on direction
    if (currSpeed > 0.0)
    {
      distToLimit = upperLimit - stepCount;
      // reduce current speed if getting near the lower/upper limit
      if (distToLimit == 0)
      {
        currSpeed = 0.0;
      }
      else if (distToLimit < brakeDist)
      {
        currSpeed = sqrt(2*abs(distToLimit)*acceleration);
      }
    }
    else
    {
      distToLimit = stepCount - lowerLimit;
      // reduce current speed if getting near the lower/upper limit
      if (distToLimit == 0)
      {
        currSpeed = 0.0;
      }
      else if (distToLimit < brakeDist)
      {
        currSpeed = -sqrt(2*abs(distToLimit)*acceleration);
      }
    }

    // absolute value of frequency is passed to STEP pin impuls generator
    currfrequency = abs(currSpeed);
    
    // save for next cycle
    tlastRun = tCurrent;
  }

  /* when the stepper runs to a certain position then it is better to
   * consider the reamining distance to the target for the speed 
   * calculation */
  void MyStepperController::calcSpeedPositioning(LONG distToTarget)
  {
    FLOAT dt = tCurrent-tlastRun;
    // distance to get to Stop = curreSpeed^2 / 2*a
    LONG brakeDist = currSpeed*currSpeed/(2*acceleration)+1;
    // speed change
    FLOAT dSpeed =  (dt*acceleration)/SECOND; 
    
    // no braking needed yet, go to max speed with ramp
    if (abs(brakeDist) < abs(distToTarget))
    {
      if (distToTarget > 0)
      {
        if (currSpeed < maxSpeed)
        {
          currSpeed += dSpeed;
          // limit to maxSpeed
          currSpeed = min(currSpeed, maxSpeed);
        }
      }
      else
      {
        if (currSpeed > -maxSpeed)
        {
          currSpeed -= dSpeed;
          // limit to maxSpeed
          currSpeed = max(currSpeed, -maxSpeed);
          // Serial.print("\n acclerating neg: ");
        }
      }      
    }
    // braking
    else
    {
      currSpeed = sqrt(2*abs(distToTarget)*acceleration);
      if (distToTarget < 0)
      {
        currSpeed = -currSpeed;
      }
    }

    // Serial.print("\n currSpeed: ");
    // Serial.print(currSpeed);
    
    currfrequency = abs(currSpeed);
    
    tlastRun = tCurrent;
  }

  void MyStepperController::calcDirection(void)
  {
    // since speed variables are FLOAT we need to have a threshold below 
    // the motor speed shall be considered as zero
    INT stopTshld = (INT)(STOP_THRESHOLD*acceleration);

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
    // Serial.print("\n rotDir: ");
    // Serial.print(rotDir);
    
  }

  /* gets a frequency and creates pulses on the step
   * pin. Needs to be called cyclic */
  void MyStepperController::createStepPulse()
  {
    // ULONG tCurrent = micros();

    // if motor is stopped we don't want to get any random pulses
    if (rotDir != STOP)
    {
      // convert frequency into period
      ULONG T = SECOND/currfrequency;

      if (LOW == stepPinLvl) 
      {
        if (tCurrent - this->tPulseBegin >= T) 
        {
          // new period begins with HIGH
          digitalWrite(pinStep, HIGH);
          stepPinLvl = HIGH;
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
          stepPinLvl = LOW;
        }
      }
    }
    else
    {
      // set to LOW if motor shall be stopped
      digitalWrite(pinStep, LOW);
      stepPinLvl = LOW;
    }
  }


