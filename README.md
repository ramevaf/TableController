# TableController

This is a programm made for controlling a self build heigth-adjustable desk. The desk is beeing lifted by a stepper motor which is controlled by an Arduino Uno(R)
There are five buttons foreseen:
* drive up: if pressed the desk goes up.
* drive down: if pressed the desk goes down.
* goto zero: if pressed short the desk motor drives to the lowest position. If pressed long the control switches into calibration mode.
* goto pos1: if pressed short the desk motor drives to the stored position 1. If pressed long the current position is stored as position 1.
* goto pos2: if pressed short the desk motor drives to the stored position 2. If pressed long the current position is stored as position 2.
  
Calibration mode: in this mode you can lift the table with the up/down button as normal but the end switch protection is turned off. If you press the goto zero button short the current position will be stored as lowest possible position.
