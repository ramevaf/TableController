#ifndef MYBUTTON_H
#define MYBUTTON_H

#include "types.h"

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
  TIPSTS statusK1;
  TIPTYPE tipTypeK1;
};

class MyButton
{
public:
  MyButton(short pinNumber)
  {
    this->pinNumber = pinNumber;
    pinMode(pinNumber, INPUT);
  }

  BUTTON_STS getStatus(void)
  {
    setTipStatus();
    setTipType();

    return btnSts;
  }  

  void setDebounceDelay(ULONG ms)
  {
    debounceDelay = ms;
  }

  void setLongTipDelay(ULONG ms)
  {
    longTipDelay = ms;
  }

private:
  // paramters
  ULONG debounceDelay = 50;
  ULONG longTipDelay = 3000;
  USHORT pinNumber;
  // internal stati
  ULONG lastDebounceTime;
  ULONG lastRisingEdgeTime;
  USHORT rawButtonStsK1;
  BUTTON_STS btnSts;

  void setTipStatus(void)
  {
    int readVal = digitalRead(pinNumber);

    btnSts.statusK1 = btnSts.status;

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

  void setTipType(void)
  {
    btnSts.tipTypeK1 = btnSts.tipType;

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
