#include "myLED.h"

MyLED::MyLED(short pin)
    :pinNumber(pin)
{
    pinMode(pin, OUTPUT);
}

void MyLED::turnOn(void)
{
    status = ON;
}

void MyLED::turnOff(void)
{
    status = OFF;
}

void MyLED::toggle(void)
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

void MyLED::blink(ULONG duration)
{
    status = BLINK;
    blinkDur = duration;
}

void MyLED::writeOut(void)
{
    /* write output only on event */
    if ((statusK1 != ON) && (status == ON))
    {
        digitalWrite(pinNumber, HIGH);
    }
    else if ((statusK1 != OFF) && (status == OFF))
    {
        digitalWrite(pinNumber, LOW);
    }
    /* blinking start */
    else if ((statusK1 != BLINK) && (status == BLINK))
    {
        tStartBlink = millis();
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
    /* blinking in progress */
    else if ((status == BLINK) && ((millis() - tStartBlink) > blinkDur))
    {
        if (blinkType == ON_OFF_ON)
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