/*
  This file contains a class deinition for a simple LED which can turned on, off and flicker/blink.
  The calculation of the internal stati and writing to the pin is   split up into different methods
  so you can controll how often you need update the output.

  Author: Daniel Ramonat
  Date: 07.10.2020
*/

#ifndef MYLED_H
#define MYLED_H

#include "types.h"

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
    MyLED(short pinNumber)
    {
        this->pinNumber = pinNumber;
        pinMode(pinNumber, OUTPUT);
    }

    void turnOn(void)
    {
        status =   ON;
    }
    
    void turnOff(void)
    {
        status = OFF;
    }

    void toggle(void)
    {
        if (status == OFF)
        {
          status = ON;
        }
        else
        {
          status = OFF;
        }
    }

    void blink(ULONG duration)
    {
        status = BLINK;
        blinkDur = duration;
    }

    // writing internal status to actuall pin OUTPUT
    // this is rather slow so call not too regulary
    void writeOut(void)
    {
        // write output only on event
        if (    (statusK1 != ON)
             && (status == ON)
           )
        {
            digitalWrite(pinNumber, HIGH);
        }
        else if (    (statusK1 != OFF)
                  && (status == OFF)
                )
        {
            digitalWrite(pinNumber, LOW);
        }
        // blinking start
        else if (    (statusK1 != BLINK)
                  && (status == BLINK)
                )
        {
            startBlink = millis();
            if (statusK1 == OFF)
            {
                blinkType = OFF_ON_OFF;
                digitalWrite(pinNumber, HIGH);
            }
            else
            {
                blinkType = ON_OFF_ON;
                digitalWrite(pinNumber, LOW);
            }
        }
        // blinking in progress
        else if (   (status == BLINK)
                 && ((millis() - startBlink) > blinkDur)
                )
        {
            if(blinkType == ON_OFF_ON)
            {
                digitalWrite(pinNumber, HIGH);
                status = ON;
            }
            else
            {
                digitalWrite(pinNumber, LOW);
                status = OFF;
            }
        }

        statusK1 = status;
    }

private:
    USHORT pinNumber;       // Pin number where the LED is conneted
    LEDSTAT status;         // status see LEDSTAT
    LEDSTAT statusK1;       // status of the last cycle
    ULONG startBlink;       // time when the blinking startet (actual output)
    ULONG blinkDur;         // blinking duration
    BLINKTYPE blinkType;    // whether blink startet when LED was on or off, see BLINKTYPE
};

#endif
