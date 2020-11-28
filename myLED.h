/*
  This file contains a class definition for a simple LED which can turned on, off and 
  flicker/blink. The calculation of the internal stati and writing to the pin is split up into 
  different methods so you can controll how often you need update the output.

  Author: Daniel Ramonat
  Date: 07.10.2020
*/

#ifndef MYLED_H
#define MYLED_H

#include "types.h"
#include "Arduino.h"

enum LEDSTAT
{
  OFF,
  ON,
  BLINK
};

enum BLINKTYPE
{
  OFF_ON_OFF,
  ON_OFF_ON,
};


class MyLED
{
public:
    /* constructor
     * \param[in] pin the arduino pin number where the LED is at */ 
    MyLED(USHORT pin);

    /* thurns the LED on */
    void turnOn(void);
    
    /* thurns the LED off */
    void turnOff(void);

    /* what the name says */
    void toggle(void);

    /* lets the LED blink either on->off->on or off->on->off
     * \param[in] duration duration of blink */ 
    void blink(ULONG duration);

    /* writing internal status to actuall pin OUTPUT. This is rather slow */
    void writeOut(void);

private:
    const USHORT pinNumber; // Pin number where the LED is conneted
    LEDSTAT status;         // status see LEDSTAT
    LEDSTAT statusK1;       // status of the last cycle
    ULONG tStartBlink;      // time when the blinking startet (actual output)
    ULONG blinkDur;         // blinking duration
    BLINKTYPE blinkType;    // whether blink startet when LED was on or off, see BLINKTYPE
};

#endif
