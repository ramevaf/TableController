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
#include "Arduino.h"

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
  /* constructor
   * \param[in] pin the arduino pin number where the LED is at */ 
  MyButton(short pin, 
           void (*onShortTip)(void) = nullptr, 
           void (*onLongTip)(void) = nullptr);

  /* reads in the pin and calculates the new status */
  void updateStatus(void);

  /* getter for the status
   * \return current Button status */ 
  BUTTON_STS getStatus(void);

  /* set debounce time
   * \param[in] ms debounce time in ms */ 
  void setDebounceDelay(ULONG ms);

  /* set time after which the tip is considered a long tip
   * \param[in] ms minimum long tip duration in ms */ 
  void setLongTipDelay(ULONG ms);

private:
  /* pointer to event functions */
  void (*fctOnShortTip)(void);
  void (*fctOnLongTip)(void);

  const USHORT pinNumber;
  ULONG debounceDelay = 50;
  ULONG longTipDelay = 2000;
  ULONG tLastDebounce;
  ULONG tLastRisingEdge;
  USHORT rawButtonSts;
  USHORT rawButtonStsK1;
  BUTTON_STS btnSts;

  /* calculates whether the button is pressed and the edges
   * \param[in] readVal raw value of the pin */ 
  void updateTipStatus(USHORT readVal);

  /* calculates whether the press is a long/short one */
  void updateTipType(void);
};

#endif
