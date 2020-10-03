/*
 * Author: Daniel Ramonat
 */
#include <Stepper.h>
#include <EEPROM.h>
#include "types.h"
#include "MyStepperController.h"
#include "MyButton.h"


// SERIAL INTERFACE
#define SERIAL_BAUD 9600
// STEPPER
#define STEPPER_STEPS_PER_REV 200
#define STEPPER_MAX_SPEED 40
// PINS
#define PIN_STEPPER_A_PLUS 2
#define PIN_STEPPER_A_MINUS 3
#define PIN_STEPPER_B_PLUS 4
#define PIN_STEPPER_B_MINUS 5
#define PIN_CALIBRATION_MODE_LED 7
#define PIN_DRIVE_UP 8
#define PIN_DRIVE_DOWN 9
#define PIN_POS_RETURN_PIN 10
#define PIN_POS1_PIN 12
#define PIN_POS2_PIN 13
// EEPROM
#define EE_ENABLED 0          // allows to disable EEPROM writing for testing
#define EE_START_ADDR 0

// ---------- USER DEFINED DATATYPES ------------
struct POSSTORAGE
{
  LONG currentPos;
  LONG pos1;
  LONG pos2;
};


// ---------- GLOBAL VARIABLES ------------
MyStepperController StepperControl(STEPPER_STEPS_PER_REV, PIN_STEPPER_A_PLUS, PIN_STEPPER_A_MINUS, PIN_STEPPER_B_PLUS, PIN_STEPPER_B_MINUS);
MyButton driveUpButton(PIN_DRIVE_UP);
MyButton driveDownButton(PIN_DRIVE_DOWN);
MyButton returnButton(PIN_POS_RETURN_PIN);
MyButton pos1Button(PIN_POS1_PIN);
MyButton pos2Button(PIN_POS2_PIN);
POSSTORAGE posStorage;
bool calibModeEnabled = false;

void writePosToEE(void)
{
  // print for debug
  Serial.print("\n currentPos: ");
  Serial.print(posStorage.currentPos);
  Serial.print("\n pos1: ");
  Serial.print(posStorage.pos1);
  Serial.print("\n pos2: ");
  Serial.print(posStorage.pos2);
  // position has changed. Update posStorage
  if (EE_ENABLED) EEPROM.put(EE_START_ADDR, posStorage);
}

void setup()
{
  // initialize the serial port:
  Serial.begin(SERIAL_BAUD);
  // set pin mode
  pinMode(PIN_DRIVE_UP, INPUT);
  pinMode(PIN_DRIVE_DOWN, INPUT);
  pinMode(PIN_POS_RETURN_PIN, INPUT);
  pinMode(PIN_POS1_PIN, INPUT);
  pinMode(PIN_POS2_PIN, INPUT);
  pinMode(PIN_CALIBRATION_MODE_LED, OUTPUT);

  // get position values from EEPROM and pass to stepper control
  if (EE_ENABLED) EEPROM.get(EE_START_ADDR, posStorage);
  StepperControl.setStepCount(posStorage.currentPos);

  // print for debug
  Serial.print("\n currentPos: ");
  Serial.print(posStorage.currentPos);
  Serial.print("\n pos1: ");
  Serial.print(posStorage.pos1);
  Serial.print("\n pos2: ");
  Serial.print(posStorage.pos2);
  
}

void loop()
{

  // read the analog value connected to the potentiometer for controlling the speed
  // and map it to a range from 1 to 100:
  unsigned short sensorReading = analogRead(A0);
  int motorSpeed = map(sensorReading, 0, 1023, 1, STEPPER_MAX_SPEED);
  // read buttons
  BUTTON_STS driveUpButtonSt = driveUpButton.getStatus();
  BUTTON_STS driveDownButtonSt = driveDownButton.getStatus();
  BUTTON_STS returnButtonSts = returnButton.getStatus();
  BUTTON_STS pos1ButtonSts = pos1Button.getStatus();
  BUTTON_STS pos2ButtonSts = pos2Button.getStatus();

  // Serial.print(pos1ButtonTipType);

  StepperControl.setStepperSpeed(motorSpeed);

  // ----------------------------------------------------------------------------
  // drive up button logic:
  //    pressed: drive up
  // ----------------------------------------------------------------------------
  if (PRESSED == driveUpButtonSt.status)
  {
    StepperControl.stepClockwise();
  }
  // ----------------------------------------------------------------------------
  // drive down button logic:
  //    pressed: drive down until zero position (or below in calibration mode)
  // ----------------------------------------------------------------------------
  else if (PRESSED == driveDownButtonSt.status)
  {
    if (  (StepperControl.getStepCount() > 0)
       || (true == calibModeEnabled)
       )
    {
      StepperControl.stepCounterClockwise();
    }
  }
  // update position of finished
  if (  (FALLING_EDGE == driveUpButtonSt.status)
     || (FALLING_EDGE == driveDownButtonSt.status)
     )
  {
    posStorage.currentPos = StepperControl.getStepCount();
    writePosToEE();
  }

  // ----------------------------------------------------------------------------
  // return button logic:
  //    short tip: goto zero position (save zero and reset memory in calibMode)
  //    long tip: activate calibration mode
  // ----------------------------------------------------------------------------
  if (  (FALLING_EDGE == returnButtonSts.status)
     && (SHORT_TIP == returnButtonSts.tipType)
     )
  {
    if (false == calibModeEnabled)
    {
      // go to zero position if not in calibration mode
      StepperControl.gotoPosition(0);
      writePosToEE();
    }
    else
    {
      // save current position as zero position and reset saved positions
      StepperControl.reset();
      posStorage.currentPos = StepperControl.getStepCount();
      posStorage.pos1 = 0;
      posStorage.pos2 = 0;
      // update posStorage in EEPROM
      writePosToEE();
    }
    
    
  }
  // toggle caliibration mode on/off on return button long tip 
  if (  (LONG_TIP != returnButtonSts.tipTypeK1)
     && (LONG_TIP == returnButtonSts.tipType)
     )
  {
    calibModeEnabled = !calibModeEnabled;
  }

  // ----------------------------------------------------------------------------
  // pos1 button logic:
  //    short tip: goto pos1 position
  //    long tip: save current position as pos1
  // ----------------------------------------------------------------------------
  if (  (FALLING_EDGE == pos1ButtonSts.status)
     && (SHORT_TIP == pos1ButtonSts.tipType)
     )
  {
    // go to pos1 position
    StepperControl.gotoPosition(posStorage.pos1);
  }
  // save current position as pos1
  if (  (LONG_TIP != pos1ButtonSts.tipTypeK1)
     && (LONG_TIP == pos1ButtonSts.tipType)
     )
  {
    posStorage.pos1 = StepperControl.getStepCount();
    // update posStorage in EEPROM
    writePosToEE();
    // flicker LED
    digitalWrite(PIN_CALIBRATION_MODE_LED, HIGH);
    delay(100);
    digitalWrite(PIN_CALIBRATION_MODE_LED, LOW);
  }

  // ----------------------------------------------------------------------------
  // pos2 button logic:
  //    short tip: goto pos2 position
  //    long tip: save current position as pos2
  // ----------------------------------------------------------------------------
  if (  (FALLING_EDGE == pos2ButtonSts.status)
     && (SHORT_TIP == pos2ButtonSts.tipType)
     )
  {
    // go to pos2 position
    StepperControl.gotoPosition(posStorage.pos2);
  }
  // save current position as pos2Button
  if (  (LONG_TIP != pos2ButtonSts.tipTypeK1)
     && (LONG_TIP == pos2ButtonSts.tipType)
     )
  {
    posStorage.pos2 = StepperControl.getStepCount();
    // update posStorage in EEPROM
    writePosToEE();
    // flicker LED
    digitalWrite(PIN_CALIBRATION_MODE_LED, HIGH);
    delay(100);
    digitalWrite(PIN_CALIBRATION_MODE_LED, LOW);
  }


  // if calibration mode turn on LED
  if (true == calibModeEnabled)
  {
    digitalWrite(PIN_CALIBRATION_MODE_LED, HIGH);
  }
  else
  {
    digitalWrite(PIN_CALIBRATION_MODE_LED, LOW);
  }

  
  delay(1);
}
