#ifndef MYSTEPPERCONTROLLER_H
#define MYSTEPPERCONTROLLER_H

#include "types.h"
#include "Arduino.h"

#define PULSE_DURATION 1  // microseconds
#define SECOND 1000000

const FLOAT STOP_THRESHOLD = 0.5;  // steps/s

enum DIRECTION
{
  STOP,
  CLOCKWISE,
  COUNTERCLOCKWISE
};

enum CTRL_MODE
{
  END_PROTECTION,
  POSITIONING,
  UNLIMED
};

class MyStepperController
{
public:
  MyStepperController(UINT stepsPerRev, SHORT pinStep, SHORT pinDir);

  /* main method for calculating the outputs needed for driving to stepper 
   * should be called cyclic as often as possible */
  void run();

  void moveTo(LONG targetStep);

  /* set target speed (steps/s) 
   * positive valies result in cw rotation, negative ones in ccw */
  void setTargetSpeed(FLOAT ts);

  /* set max allowed speed (steps/s) */
  void setMaxSpeed(FLOAT maxSpeed);

  /* set acceleration (steps/s^2) */
  void setAcceleration(UINT accel);

  /* returns current position */
  LONG getStepCount(void);

  /* returns current speed */
  FLOAT getCurrentSpeed(void);
  /* returns current speed */
  CTRL_MODE getCtrlMode(void);

  DIRECTION getRotDir(void);

  void setLowerLimit(LONG pos);

  void setUpperLimit(LONG pos);

  void setStepCount(LONG pos);

  void reset(void);

  void doStep(DIRECTION direction);

  void runSpeed(void);

private:
  const USHORT pinStep;     // pin number of STEP pin
  const USHORT pinDir;      // pin number of DIR pin
  const UINT stepsPerRev;   // number of steps per rev
  
  LONG stepCount;           // stepper position
  FLOAT currSpeed;          // current speed in steps/s
  FLOAT targetSpeed;        // desired speed in steps/s in speed control mode
  FLOAT currfrequency;      // absolute value of motor frequency in steps/s
  FLOAT maxSpeed = SECOND/(PULSE_DURATION*2); // maximum allowed speed in steps/s
  UINT acceleration;        // acceleration in steps/s^2
  USHORT stepPinLvl = LOW;  // current level of the STEP pin
  // ULONG tCurrent;           // current timestamp in micros of the call of run() method
  // ULONG tlastRun;           // timestamp in micros of the last call of run() method
  ULONG tPulseBegin;        // timestamp in begining of the current period for the STEP pin
  DIRECTION rotDir;         // current motor direction (cw/ccw/stop)

  ULONG tPeriod;

  CTRL_MODE ctrlMode;
  LONG startingPos;
  LONG targetPos;

  LONG upperLimit = LONG_MAX;
  LONG lowerLimit = 0;


  /* calculates the current speed depending on the user given target speed
   * considering the acceleration */
  void calcSpeed(void);

  ULONG getNewPeriod(void);

  /* when the stepper runs to a certain position then it is better to
   * consider the reamining distance to the target for the speed 
   * calculation */
  void calcSpeedPositioning(LONG distToTarget);

  void calcDirection(void);

  void rampSpeed(FLOAT targetSpeed);

  FLOAT limitSpeed(FLOAT currSpeed, LONG distToGo);

};

#endif
