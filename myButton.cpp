#include "MyButton.h"

MyButton::MyButton(short pin)
    :pinNumber(pin)
{
    pinMode(pinNumber, INPUT);
}

void MyButton::updateStatus(void)
{
    rawButtonSts = digitalRead(pinNumber);
    updateTipStatus(rawButtonSts);
    updateTipType();
}

BUTTON_STS MyButton::getStatus(void)
{
    return btnSts;
}

void MyButton::setDebounceDelay(ULONG ms)
{
    debounceDelay = ms;
}

void MyButton::setLongTipDelay(ULONG ms)
{
    longTipDelay = ms;
}

void MyButton::updateTipStatus(USHORT readVal)
{
    /* safe timestamp of level change */
    if (readVal != rawButtonStsK1)
    {
        lastDebounceTime = millis();
        /* if the button transists to HIGH, safe the time for tiptype calculation */
        if (HIGH == readVal)
        {
            lastRisingEdgeTime = millis();
        }
    }
    /* compare timestamps for debouncing */
    if ((millis() - lastDebounceTime) > debounceDelay)
    {
        /* calculate status based on debounced signal */
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
        else /* LOW */
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

void MyButton::updateTipType(void)
{
    /* even on FALLING_EDGE the tiptype is transmitted to be SHORT_TIP or LONG_TIP. This makes it 
     * way easier to check for type */
    if (NOT_PRESSED != btnSts.status)
    {
        /* time since rising edge */
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