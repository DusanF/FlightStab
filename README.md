# [WIP] FlightStab
Simple flight stabilisation for RC aircrafts based on SP F3 board.
STM32CubeIDE project, written in C++/C using STM32 HALs.

At this stage, 3 PID regulators are hardcoded, along with mixers for airplane with flaperons. RC receiver is connected using iBus.
One RC channel is used to switch 3 flight modes: manual and 2 stabilised modes.
Buzzer uses queue, works in background
