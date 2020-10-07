/*
  This file contains a class deinition for a debounced input button which provides also an 
  interface to distinguish between short and long tips. The calculation of the internal stati and
  grabing the information is split up into different methods so you can controll how often you need
  to update the status.

  Author: Daniel Ramonat
  Date: 07.10.2020
*/

#ifndef MYBUTTON_H
#define MYBUTTON_H

#include "types.h"

/* typdefs */
enum TIPSTS
{
  NOT_PRESSED,
  PRESSED,
  RISING_EDGE,
  FALLING_EDGE
};

enum TIPTYPE
{
  NONE,
  SHORT_TIP,
  LONG_TIP
};

struct BUTTON_STS
{
  TIPSTS status;
  TIPTYPE tipType;
};

/* class definiton */
class MyButton
{
public:
  /* constructor: needs the arduino pin number */
  MyButton(short pinNumber)
  {
    this->pinNumber = pinNumber;
    pinMode(pinNumber, INPUT);
  }

  /* reads in the pin and calculates the new status */
  void updateStatus(void)
  {
    rawButtonSts = digitalRead(pinNumber);
    updateTipStatus(rawButtonSts);
    updateTipType();
  }

  /* getter for the status */
  BUTTON_STS getStatus(void)
  {  
    return btnSts;
  }

  /* set debounce time */
  void setDebounceDelay(ULONG ms)
  {
    debounceDelay = ms;
  }

  /* set time after which the tip is considered a long tip*/
  void setLongTipDelay(ULONG ms)
  {
    longTipDelay = ms;
  }

private:
  // default paramteters
  ULONG debounceDelay = 100;
  ULONG longTipDelay = 2000;
  USHORT pinNumber;
  // internal stati
  ULONG lastDebounceTime;
  ULONG lastRisingEdgeTime;
  USHORT rawButtonSts;
  USHORT rawButtonStsK1;
  BUTTON_STS btnSts;

  /* calculates whether the button is pressed and the edges */
  void updateTipStatus(USHORT readVal)
  {
    // safe timestamp of level change
    if (readVal != rawButtonStsK1)
    {
      lastDebounceTime = millis();
      // if the button transists to HIGH, safe the time for tiptype calculation
      if (HIGH == readVal)
      {
        lastRisingEdgeTime = millis();
      }
    }
    // compare timestamps for debouncing
    if ((millis() - lastDebounceTime) > debounceDelay)
    {
      // calculate status based on debounced signal
      if (readVal == HIGH)
      {
        switch (btnSts.status)
        {
        case PRESSED:
        case RISING_EDGE:
          btnSts.status = PRESSED;
          break;
        case NOT_PRESSED:
        case FALLING_EDGE:
          btnSts.status = RISING_EDGE;
          break;
        }
      }
      else // LOW
      {
        switch (btnSts.status)
        {
        case PRESSED:
        case RISING_EDGE:
          btnSts.status = FALLING_EDGE;
          break;
        case NOT_PRESSED:
        case FALLING_EDGE:
          btnSts.status = NOT_PRESSED;
          break;
        }
      }
    }
    rawButtonStsK1 = readVal;
  }

  /* calculates whether the press is a long/short one */
  void updateTipType(void)
  {
    // even on FALLING_EDGE the tiptype is transmitted to be SHORT_TIP or LONG_TIP. This makes it way easier to check for type
    if (NOT_PRESSED != btnSts.status)
    {
      // time since rising edge
      if ((millis() - lastRisingEdgeTime) > longTipDelay)
      {
        btnSts.tipType = LONG_TIP;
      }
      else
      {
        btnSts.tipType = SHORT_TIP;
      }
    }
    else
    {
      btnSts.tipType = NONE;
    }
  }
};

#endif
