#ifndef MYSTEPPERCONTROLLER_H
#define MYSTEPPERCONTROLLER_H

#include "types.h"
#include "Arduino.h"

const ULONG PULSE_DURATION = 1;     // pulse duration on step pulse in microseconds
const FLOAT SECOND_US = 1000000.0;  // 1s = 10e6 µs
const FLOAT SECOND_MS = 1000.0;     // 1s = 1000 ms
const FLOAT STOP_THRESHOLD = 0.5;   // threshold where the stepper is considered stopped in steps/s

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

  /* set max allowed speed (steps/s)
   * \param[in] maxSpeed maximum alled speed (absolute value used) */
  void setMaxSpeed(FLOAT speed);

  /* set acceleration (steps/s^2) 
   * \param[in] accel acceleration in steps/second^2*/
  void setAcceleration(UINT accel);

  /* set lower end point 
   * \param[in] pos position of lower limit in steps */
  void setLowerLimit(LONG pos);

  /* set upper end point 
   * \param[in] pos position of upper limit in steps */
  void setUpperLimit(LONG pos);

  /* reset the step count of motor
   * \param[in] pos new position of the motor in steps */
  void setStepCount(LONG pos);

  /* enable/siable end point protection
   * \param[in] enbl if true end point protection is ON */
  void setLimitProtectionEnabled(BOOL enbl);

  /* returns current position
   * \return current motor position in steps */
  LONG getStepCount(void);

  /* returns current speed 
   * \return current motor speed in steps/s */
  FLOAT getCurrentSpeed(void);

  /* returns currently active control mode 
   * \return currently active control mode  */
  CTRL_MODE getCtrlMode(void);

  /* returns direction of rotation 
   * \return direction of rotation or STOP if stopped  */
  DIRECTION getRotDir(void);

  /* returns whether stepper is running
   * \return true if motor is running */
  BOOL isRunning(void);

  /* makes to motor turn one step
   * \param[in] direction direction of desired rotation */
  void doStep(DIRECTION direction);

private:

  const USHORT pinStep;       // pin number of STEP pin
  const USHORT pinDir;        // pin number of DIR pin
  const UINT stepsPerRev;     // number of steps per rev
  
  LONG stepCount;             // stepper position
  FLOAT currSpeed;            // current speed in steps/s
  FLOAT targetSpeed;          // desired speed in steps/s in speed control mode
  FLOAT maxSpeed = SECOND_US/(PULSE_DURATION*2); // maximum allowed speed in steps/s
  UINT acceleration;          // acceleration in steps/s^2, also used for decelleration
  DIRECTION rotDir;           // current motor direction (cw/ccw/stop)
  ULONG tPeriod;              // period time for the current period of the step pin
  CTRL_MODE ctrlMode;         // whether the stepper is running to a position or in speed control
  LONG targetPos;             // target position if stepper does positioning
  ULONG n;                    // used to indentify the step count of a ramp

  BOOL limitProtectionEnabled = true; // allows to limit the range where the stepper runs
  LONG upperLimit = LONG_MAX; // upper end point
  LONG lowerLimit = 0;        // lower end point

  /* calculates the current speed depending on the user given target speed
   * considering the acceleration */
  void calcSpeed(void);

  /* converts the current speed into pulses on STEP and DIR */
  void runSpeed(void);

  /* calculates new period time for STEP timer depending on speed
   * \returns new period time */
  ULONG getNewPeriod(void);

  /* calculates ramp from current speed to target speed */
  void rampSpeed(void);

  /* limits speed to target speed if near limit or position
   * param[in] distToGo remaining distance in steps 
   * \returns new speed in steps/s */
  FLOAT limitSpeed(LONG distToGo);

  /* calculates current direction of rotation */
  void calcDirection(void);

};

#endif
