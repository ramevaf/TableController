# TableController

This is a program made for controlling a self-build adjustable-height- desk. The desk is being lifted by a stepper motor which is controlled by an Arduino Uno(R).

I wrote my own little stepper library since the commonly used accelstepper library does not support speed changes during movement of the stepper motor. The table is lifted by a scissors lift which results in a nonlinear lifting-to-motor-position ratio. To maintain a constant lifting speed I need to adapt the stepper motor speed constantly depending on the angle of the scissors.

There are five buttons foreseen to control the desk:
* drive up: if pressed the desk goes up.
* drive down: if pressed the desk goes down.
* goto zero: if pressed short the desk motor drives to the lowest position.
* goto pos1: if pressed short the desk motor drives to the stored position 1. If pressed long the current position is stored as position 1.
* goto pos2: if pressed short the desk motor drives to the stored position 2. If pressed long the current position is stored as position 2.
  
Furthermore there is a calibration mode added. In this mode you can lift the table with the up/down button as normal but the end switch protection is turned off. If you press the goto zero button short the current position will be stored as lowest possible position. To enable/disable the calibartion mode the user must press the goto zero buttion until the calibration mode LED turns on/off.

![rendering_image](https://github.com/ramevaf/TableController/blob/master/Rendering.jpg)
