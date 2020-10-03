#ifndef MYSTEPPERCONTROLLER_H
#define MYSTEPPERCONTROLLER_H

#include "types.h"

class MyStepperController
{

public:
  MyStepperController(UINT stepsPerRev, SHORT pinAPlus, SHORT pinAMinus, SHORT pinBplus, SHORT pinBMinus)
  {
    myStepper = new Stepper(stepsPerRev, pinAPlus, pinAMinus, pinBplus, pinBMinus);
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
    myStepper->step(-1);
    stepCount++;
  }

  void stepCounterClockwise(void)
  {
    myStepper->step(1);
    stepCount--;
  }

  void setStepperSpeed(UINT motSpeed)
  {
    myStepper->setSpeed(motSpeed);
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
  Stepper *myStepper;
};

#endif
