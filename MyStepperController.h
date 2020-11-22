#ifndef MYSTEPPERCONTROLLER_H
#define MYSTEPPERCONTROLLER_H

#include "types.h"
#include "Arduino.h"

#define PULSE_DURATION 1  // microseconds
const FLOAT SECOND_US = 1000000.0;  // 1s = 10e6 µs
const FLOAT SECOND_MS = 1000.0; // 1s = 1000 ms

const FLOAT STOP_THRESHOLD = 0.5;  // steps/s

enum DIRECTION
{
  STOP,
  CLOCKWISE,
  COUNTERCLOCKWISE
};

enum CTRL_MODE
{
  SPEED_CTRL,
  POSITIONING,
};

class MyStepperController
{
public:
  MyStepperController(UINT stepsPerRev, SHORT pinStep, SHORT pinDir);

  /* main method for calculating the outputs needed for driving to stepper 
   * should be called cyclic as often as possible */
  void run();

   /* makes the stepper run to a given position
    * \param[in] targetStep The dirsired postion (pos/neg) */ 
  void moveTo(LONG targetStep);

  /* stops the motor if running */
  void stop(void);

  /* set target speed (steps/s) 
   * \param[in] ts target speed in steps/second, positive valies result in clockwise rotation, 
   * negative ones in counterclockwise */
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

  void setLimitProtectionEnabled(BOOL enbl);
  /* returns TRUE if stepper is running  */
  BOOL isRunning(void);

  void reset(void);

  void doStep(DIRECTION direction);

  void runSpeed(void);

  /* calculates the current speed depending on the user given target speed
   * considering the acceleration */
  void calcSpeed(void);

private:

  const USHORT pinStep;       // pin number of STEP pin
  const USHORT pinDir;        // pin number of DIR pin
  const UINT stepsPerRev;     // number of steps per rev
  
  LONG stepCount;             // stepper position
  FLOAT currSpeed;            // current speed in steps/s
  FLOAT targetSpeed;          // desired speed in steps/s in speed control mode
  FLOAT maxSpeed = SECOND_US/(PULSE_DURATION*2); // maximum allowed speed in steps/s
  UINT acceleration;          // acceleration in steps/s^2, also used for decelleration
  ULONG tPulseBegin;          // timestamp in begining of the current period for the STEP pin
  DIRECTION rotDir;           // current motor direction (cw/ccw/stop)
  ULONG tPeriod;              // period time for the current period of the step pin
  CTRL_MODE ctrlMode;         // whether the stepper is running to a position or in speed control
  LONG targetPos;             // target position if stepper does positioning
  ULONG n;                    // used to indentify the step count of a ramp

  BOOL limitProtectionEnabled = true; // allows to limit the range where the stepper runs
  LONG upperLimit = LONG_MAX; // upper end point
  LONG lowerLimit = 0;        // lower end point



  ULONG getNewPeriod(void);

  void rampSpeed(FLOAT targetSpeed);

  FLOAT limitSpeed(LONG distToGo);

  void calcDirection(void);
};

#endif
