/*
  This file contains a small programm for controlling a self build adjustable-height-desk. The desk
  is beeing lifted by a stepper motor which is controlled by an Arduino Uno(R) There are five 
  buttons foreseen:

    - drive up: if pressed the desk goes up.
    - drive down: if pressed the desk goes down.
    - goto zero: if pressed short the desk motor drives to the lowest position. If pressed long the 
      control switches into calibration mode.
    - goto pos1: if pressed short the desk motor drives to the stored position 1. If pressed long the
      current position is stored as position 1.
    - goto pos2: if pressed short the desk motor drives to the stored position 2. If pressed long the
      current position is stored as position 2.

  Calibration mode: in this mode you can lift the table with the up/down button as normal but the end
  switch protection is turned off. If you press the goto zero button short the current position will
  be stored as lowest possible position.

  Author: Daniel Ramonat
  Date: 07.10.2020
*/

#include <EEPROM.h>
#include "types.h"
#include "myLED.h"
#include "MyButton.h"
#include "MyStepperController.h"


// SERIAL INTERFACE
#define SERIAL_BAUD 9600
// TASK SCHEDULER
#define TASK_TIME_20MS 20
// STEPPER
#define STEPPER_STEPS_PER_REV 400
#define STEPPER_MAX_SPEED 3
#define STEPPER_ACCEL 2
#define STEPPER_MIN_POS 0
#define STEPPER_MAX_POS 45000
#define STEPPER_CALIBMODE_POS_EXTENSION 40000
// PINS
#define PIN_STEPPER_DIR           2
#define PIN_STEPPER_STEP          3
#define PIN_STEPPER_ENBL          4
#define PIN_CALIBRATION_MODE_LED  7
#define PIN_DRIVE_UP              8
#define PIN_DRIVE_DOWN            9
#define PIN_POS_RETURN_PIN        10
#define PIN_POS1_PIN              12
#define PIN_POS2_PIN              13
// EEPROM
#define EE_ENABLED 0          // allows to disable EEPROM writing for testing
#define EE_TIME_AFTER_LAST_MOVE 5000
#define EE_START_ADDR 0

/* ---------- USER DEFINED DATATYPES ------------ */
struct POSSTORAGE
{
  LONG currentPos;
  LONG pos1;
  LONG pos2;
};


/* ---------- GLOBAL VARIABLES ------------ */
// AccelStepper stepper(1 , PIN_STEPPER_STEP, PIN_STEPPER_DIR);
MyStepperController stepper(STEPPER_STEPS_PER_REV, PIN_STEPPER_STEP, PIN_STEPPER_DIR);
MyButton driveUpButton(PIN_DRIVE_UP);
MyButton driveDownButton(PIN_DRIVE_DOWN);
MyButton returnButton(PIN_POS_RETURN_PIN);
MyButton pos1Button(PIN_POS1_PIN);
MyButton pos2Button(PIN_POS2_PIN);
MyLED calibLed(PIN_CALIBRATION_MODE_LED);
POSSTORAGE posStorage;
bool calibModeEnabled = false;
ULONG tLastT20call;
ULONG tLastMotorRunning;
bool posWritten = true;
// K1 values:
BUTTON_STS returnButtonStsK1;
BUTTON_STS pos1ButtonStsK1;
BUTTON_STS pos2ButtonStsK1;

void savePosToEE(void)
{
  // print for debug
  // Serial.print("\n currentPos: ");
  // Serial.print(posStorage.currentPos);
  // Serial.print("\n pos1: ");
  // Serial.print(posStorage.pos1);
  // Serial.print("\n pos2: ");
  // Serial.print(posStorage.pos2);

  // position has changed. Update posStorage
  if (EE_ENABLED) EEPROM.put(EE_START_ADDR, posStorage);
}

/* setup task of the Arduino: called once at startup */
void setup()
{
  // initialize the serial port:
  Serial.begin(SERIAL_BAUD);
  
  // get position values from EEPROM and pass to stepper control
  if (EE_ENABLED) EEPROM.get(EE_START_ADDR, posStorage);
  stepper.setStepCount(posStorage.currentPos);

  /* set stepper speed */
  stepper.setMaxSpeed(STEPPER_MAX_SPEED*STEPPER_STEPS_PER_REV);
  stepper.setAcceleration(STEPPER_ACCEL*STEPPER_STEPS_PER_REV);
  stepper.setLowerLimit(0);
  stepper.setUpperLimit(5000);
}

/* called every 20 ms. All the slow stuff gets in here */
void task20ms(void)
{
  // Serial.print("\n speed: ");
  // Serial.print(stepper.getCurrentSpeed());
  // Serial.print("\t position: ");
  // Serial.print(stepper.getStepCount());

  // read input of the buttons (rather slow)  
  driveUpButton.updateStatus();
  driveDownButton.updateStatus();
  returnButton.updateStatus();
  pos1Button.updateStatus();
  pos2Button.updateStatus();
  // write LED output
  calibLed.writeOut();

  // if (true == stepper.isRunning())
  // {
  //   // motor stopped
  //   tLastMotorRunning = millis();
  //   posWritten = false;
  // }
  // else
  // {
  //   if (  (millis() - tLastMotorRunning >= EE_TIME_AFTER_LAST_MOVE)
  //      && (false == posWritten)
  //      )
  //   {
  //     savePosToEE();
  //     posWritten = true;
  //   }
  // }

}

/* fast task for driving the motor. This is called as often as possible */
void taskFast(void)
{
  // only read status but not calculate
  BUTTON_STS driveUpButtonSt = driveUpButton.getStatus();
  BUTTON_STS driveDownButtonSt = driveDownButton.getStatus();
  BUTTON_STS returnButtonSts = returnButton.getStatus();
  BUTTON_STS pos1ButtonSts = pos1Button.getStatus();
  BUTTON_STS pos2ButtonSts = pos2Button.getStatus();

  


  // ----------------------------------------------------------------------------
  // drive up button logic:
  //    pressed: drive up
  // ----------------------------------------------------------------------------
  if (  (PRESSED == driveUpButtonSt.status)
     || (FALLING_EDGE == driveUpButtonSt.status)
     )
  {
    if (true == calibModeEnabled)
    {
      stepper.setTargetSpeed(STEPPER_MAX_SPEED*STEPPER_STEPS_PER_REV);
    }
    else
    {
      stepper.setTargetSpeed(STEPPER_MAX_SPEED*STEPPER_STEPS_PER_REV);
      // stepper.moveTo(STEPPER_MAX_POS);
    }    
  }
  
  // ----------------------------------------------------------------------------
  // drive down button logic:
  //    pressed: drive down until zero position (or below in calibration mode)
  // ----------------------------------------------------------------------------
  else if (  (PRESSED == driveDownButtonSt.status)
          || (FALLING_EDGE == driveDownButtonSt.status)
          )
  {
    if (true == calibModeEnabled)
    {
      stepper.setTargetSpeed(-STEPPER_MAX_SPEED*STEPPER_STEPS_PER_REV);
    }
    else
    {
      stepper.setTargetSpeed(-STEPPER_MAX_SPEED*STEPPER_STEPS_PER_REV);
      // stepper.moveTo(STEPPER_MIN_POS);
    }    
  }
  else
  {
    // stop stepper
    stepper.setTargetSpeed(0.0);
  }

  // ----------------------------------------------------------------------------
  // return button logic:
  //    short tip: goto zero position (save zero and reset memory in calibMode)
  //    long tip: toggle calibration mode
  // ----------------------------------------------------------------------------
  if (  (FALLING_EDGE == returnButtonSts.status)
     && (FALLING_EDGE != returnButtonStsK1.status)
     && (SHORT_TIP == returnButtonSts.tipType)
     )
  {
    if (false == calibModeEnabled)
    {
      // go to zero position if not in calibration mode
      stepper.moveTo(0);
    }
    else
    {
      // save current position as zero position and reset saved positions
      stepper.setStepCount(STEPPER_MIN_POS);
      posStorage.currentPos = stepper.getStepCount();
      posStorage.pos1 = STEPPER_MIN_POS;
      posStorage.pos2 = STEPPER_MIN_POS;
      // blink LED
      calibLed.blink(300);
    }
  }

  // Serial.print("\n stepperPos: ");
  // Serial.print(stepper.getStepCount());
  

  // toggle caliibration mode on/off on return button long tip 
  if (  (LONG_TIP != returnButtonStsK1.tipType)
     && (LONG_TIP == returnButtonSts.tipType)
     )
  {
    calibModeEnabled = !calibModeEnabled;
    calibLed.toggle();
  }


  // ----------------------------------------------------------------------------
  // pos1 button logic:
  //    short tip: goto pos1 position
  //    long tip: save current position as pos1
  // ----------------------------------------------------------------------------
  if (  (FALLING_EDGE == pos1ButtonSts.status)
     && (FALLING_EDGE != pos1ButtonStsK1.status)
     && (SHORT_TIP == pos1ButtonSts.tipType)
     )
  {
    // go to pos1 position
    stepper.moveTo(posStorage.pos1);
  }
  // save current position as pos1
  if (  (LONG_TIP != pos1ButtonStsK1.tipType)
     && (LONG_TIP == pos1ButtonSts.tipType)
     )
  {
    posStorage.pos1 = stepper.getStepCount();
    savePosToEE();
    // blink LED
    calibLed.blink(300);
  }

  // ----------------------------------------------------------------------------
  // pos2 button logic:
  //    short tip: goto pos2 position
  //    long tip: save current position as pos2
  // ----------------------------------------------------------------------------
  if (  (FALLING_EDGE == pos2ButtonSts.status)
     && (FALLING_EDGE != pos2ButtonStsK1.status)
     && (SHORT_TIP == pos2ButtonSts.tipType)
     )
  {
    // go to pos2 position
    stepper.moveTo(posStorage.pos2);
  }
  // save current position as pos2Button
  if (  (LONG_TIP != pos2ButtonStsK1.tipType)
     && (LONG_TIP == pos2ButtonSts.tipType)
     )
  {
    posStorage.pos2 = stepper.getStepCount();
    savePosToEE();
    // blink LED
    calibLed.blink(300);
  }


  stepper.run();

  posStorage.currentPos = stepper.getStepCount();

  returnButtonStsK1 = returnButtonSts;
  pos1ButtonStsK1 = pos1ButtonSts;
  pos2ButtonStsK1 = pos2ButtonSts;
}

/* this is the main loop of the Arduino. It contains a very simple
  task sheduler to have a fast task for the motor controll and a slow
  on for calculating the inputs and outputs */
void loop()
{
  ULONG tCurrent = millis();
  
  if(tCurrent - tLastT20call >= TASK_TIME_20MS)
  {
    task20ms();
    tLastT20call = tCurrent;
  }
  taskFast();
}
