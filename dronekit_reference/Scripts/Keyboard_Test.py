#!/usr/bin/env python

"""

set_attitude_target.py: (Copter Only)

This example shows how to move/direct Copter and send commands
 in GUIDED_NOGPS mode using DroneKit Python.

Caution: A lot of unexpected behaviors may occur in GUIDED_NOGPS mode.
        Always watch the drone movement, and make sure that you are in dangerless environment.
        Land the drone as soon as possible when it shows any unexpected behavior.

Tested in Python 2.7.10
"""
import keyboard
while True:
    key = keyboard.read_key()
    if key == 'enter':
        print('Enter is pressed')
    if key == 'q':
        print('Quitting the program')
        break
    if key == 's':
        print('Skiping the things')

