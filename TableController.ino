/*
  This file contains a small programm for controlling a self build adjustable-height-desk. The desk
  is beeing lifted by a stepper motor which is controlled by an Arduino Uno(R) There are five 
  buttons foreseen:

    - drive up: if pressed the desk goes up.
    - drive down: if pressed the desk goes down.
    - goto zero: if pressed short the desk motor drives to the lowest position. If pressed long the
      control switches into calibration mode.
    - goto pos1: if pressed short the desk motor drives to the stored position 1. If pressed long 
      the current position is stored as position 1.
    - goto pos2: if pressed short the desk motor drives to the stored position 2. If pressed long
      the current position is stored as position 2.

  Calibration mode: in this mode you can lift the table with the up/down button as normal but the 
  end switch protection is turned off. If you press the goto zero button short the current position 
  will be stored as lowest possible position.

  Author: Daniel Ramonat
  Date: 07.10.2020
*/

#include <EEPROM.h>
#include "types.h"
#include "myLED.h"
#include "MyButton.h"
#include "MyStepperController.h"


/* SERIAL INTERFACE */
#define SERIAL_BAUD               9600
/* TASK SCHEDULER */
#define TASK_TIME_20MS            20
/* STEPPER */
#define STEPPER_STEPS_PER_REV     400
#define STEPPER_MAX_SPEED         7
#define STEPPER_ACCEL             4
#define STEPPER_MIN_POS           0
#define STEPPER_MAX_POS           45000
/* PINS */
#define PIN_STEPPER_DIR           2
#define PIN_STEPPER_STEP          3
#define PIN_STEPPER_ENBL          4
#define PIN_CALIBRATION_MODE_LED  7
#define PIN_DRIVE_UP              8
#define PIN_DRIVE_DOWN            9
#define PIN_POS_RETURN_PIN        10
#define PIN_POS1_PIN              12
#define PIN_POS2_PIN              13
/* EEPROM */
#define EE_ENABLED                0 // allows to disable EEPROM writing for testing
#define EE_TIME_AFTER_LAST_MOVE   5000
#define EE_START_ADDR             0

/* ---------- USER DEFINED DATATYPES ------------ */
struct POSSTORAGE
{
  LONG currentPos;
  LONG pos1;
  LONG pos2;
};


/* ---------- FUNCTION DECLARATIONS ------------- */
void onEvent_buttonReturnShortTip(void);
void onEvent_buttonReturnLongTip(void);
void onEvent_buttonPos1ShortTip(void);
void onEvent_buttonPos1LongTip(void);
void onEvent_buttonPos2ShortTip(void);
void onEvent_buttonPos2LongTip(void);

/* ---------- GLOBAL VARIABLES ------------------ */
/* timestamps */
ULONG tLastT20call;
ULONG tLastMotorRunning;

MyStepperController stepper(STEPPER_STEPS_PER_REV, PIN_STEPPER_STEP, PIN_STEPPER_DIR);
MyButton driveUpButton(PIN_DRIVE_UP);
MyButton driveDownButton(PIN_DRIVE_DOWN);
MyButton returnButton(PIN_POS_RETURN_PIN, &onEvent_buttonReturnShortTip, &onEvent_buttonReturnLongTip);
MyButton pos1Button(PIN_POS1_PIN, &onEvent_buttonPos1ShortTip, &onEvent_buttonPos1LongTip);
MyButton pos2Button(PIN_POS2_PIN, &onEvent_buttonPos2ShortTip, &onEvent_buttonPos2LongTip);
MyLED calibLed(PIN_CALIBRATION_MODE_LED);
POSSTORAGE posStorage;
bool calibModeEnabled;
bool posWritten = true;

/* motor speed curve to take the nonlinear force to stroke curve of the scissor lift into 
 * constideration*/
const ULONG speedCurveRanges[] = {0, 2000, 4000, 6000, 8000, 999999};
const FLOAT speedCurveSpeeds[] = {0.75, 1.5, 3.0, 5.0, STEPPER_MAX_SPEED, STEPPER_MAX_SPEED};

/* ---------- FUNCTIONS ------------------- */

/* RETURN button event
 * on short tip: goto zero position (save zero and reset memory in calibMode) */
void onEvent_buttonReturnShortTip(void)
{
  if (false == calibModeEnabled)
  {
    /* go to zero position if not in calibration mode */
    stepper.moveTo(0);
  }
  else
  {
    /* save current position as zero position and reset saved positions */
    stepper.setStepCount(STEPPER_MIN_POS);
    posStorage.currentPos = stepper.getStepCount();
    posStorage.pos1 = STEPPER_MIN_POS;
    posStorage.pos2 = STEPPER_MIN_POS;
    /* blink LED */
    calibLed.blink(300);
  }
}

/* RETURN button event
 * on long tip: toggle calibration mode */
void onEvent_buttonReturnLongTip(void)
{
  calibModeEnabled = !calibModeEnabled;
  calibLed.toggle();

  if (true == calibModeEnabled)
    stepper.setLimitProtectionEnabled(false);
  else
    stepper.setLimitProtectionEnabled(true);
}

/* POS1 button event
 * on short tip: goto pos1 position */
void onEvent_buttonPos1ShortTip(void)
{
  stepper.moveTo(posStorage.pos1);
}

/* POS1 button event
 * on long tip: save current position as pos1 and blink LED*/
void onEvent_buttonPos1LongTip(void)
{
  posStorage.pos1 = stepper.getStepCount();
  savePosToEE();
  calibLed.blink(300);

}

/* POS2 button event
 * on short tip: goto pos2Button position */
void onEvent_buttonPos2ShortTip(void)
{
  stepper.moveTo(posStorage.pos2);
}

/* POS2 button event
 * on long tip: save current position as pos2 and blink LED */
void onEvent_buttonPos2LongTip(void)
{
  posStorage.pos2 = stepper.getStepCount();
  savePosToEE();
  calibLed.blink(300);
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
  stepper.setUpperLimit(STEPPER_MAX_POS);
}

/* called every 20 ms. All the slow stuff gets in here */
void task20ms(void)
{
  // read input of the buttons (rather slow)  
  driveUpButton.updateStatus();
  driveDownButton.updateStatus();
  returnButton.updateStatus();
  pos1Button.updateStatus();
  pos2Button.updateStatus();
  // write LED output
  calibLed.writeOut();

  savePosToEE();

  /* modify max speed depending on current motor position to take the nonlinear force to stroke
   * curve of the scissor lift into constideration */
  FLOAT newMaxSpeed = getTargetSpeedLimit(stepper.getStepCount());
  stepper.setMaxSpeed(newMaxSpeed*STEPPER_STEPS_PER_REV);
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

  
  /* process up/down button logic. If UP pressed drive clockwise... */
  if (PRESSED == driveUpButtonSt.status)
  {
    stepper.stop(); // if stepper moves to position it shall stop the order
    stepper.setTargetSpeed(STEPPER_MAX_SPEED*STEPPER_STEPS_PER_REV);
  }
  /* if DOWN pressed drive counterclockwise ... */
  else if (PRESSED == driveDownButtonSt.status)
  {
    stepper.stop(); /// if stepper moves to position it shall stop the order
    stepper.setTargetSpeed(-STEPPER_MAX_SPEED*STEPPER_STEPS_PER_REV);
  }
  /* and if nothing is pressed stop stepper */
  else
  {
    stepper.setTargetSpeed(0.0);
  }


  /* call stepper main loop as often as possible */
  stepper.run();
  /* update posStorage */
  posStorage.currentPos = stepper.getStepCount();
}

/* this is the main loop of the Arduino. It contains a very simple
  task sheduler to have a fast task for the motor controll and slower
  ones for calculating the inputs and outputs */
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

/* returns the allowed speed for the current motor position */
FLOAT getTargetSpeedLimit(ULONG currPosition)
{
    UINT arraySize = sizeof(speedCurveRanges)/sizeof(ULONG);

    FLOAT limitSpeed;

    for(INT i=1; i<arraySize; i++)
    {
        if(currPosition <= speedCurveRanges[i])
        {
            limitSpeed = mapFloat(currPosition, 
                                  speedCurveRanges[i-1], 
                                  speedCurveRanges[i], 
                                  speedCurveSpeeds[i-1], 
                                  speedCurveSpeeds[i]);
            break;
        }
    }

    return limitSpeed;
}

/* does the same as map() but with FLOATs */
FLOAT mapFloat(ULONG x, ULONG in_min, ULONG in_max, FLOAT out_min, FLOAT out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* writes the data about current position and stored positions into EEPROM */
void savePosToEE(void)
{
   /* write position into EEPROM after motor has stopped + delay */
  if (true == stepper.isRunning())
  {
    /* save timestamp when motor is runnin */
    tLastMotorRunning = millis();
    posWritten = false;
  }
  else
  {
    if (  (millis() - tLastMotorRunning >= EE_TIME_AFTER_LAST_MOVE)
       && (false == posWritten)
       )
    {
      /* position has changed. Update posStorage */
      if (EE_ENABLED)
      {
        EEPROM.put(EE_START_ADDR, posStorage);
        Serial.print("\n Position saved in EEPROM");
      }
      posWritten = true;
    }
  }
}

