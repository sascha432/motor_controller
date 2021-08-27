# DC Motor Controller with LED dimmer

Firmware for DC motor controller https://easyeda.com/sascha23095123423/dc-motor-controller

I am using this controller for a 500W drill press with a constant RPM from 300-3500rpm load independent

* Input voltage 13.5-40V
* DC Motor controller 20A continuously, 31.25kHz PWM
* 4A MOSFET driver
* Velocity mode, constant RPM from 300 to 3500rpm/min
* PWM mode, duty cycle 5-100% (like regular drills)
* Opto Interrupter for RPM sensing
* Adjustable current limit 1-40A
* Adjustable auto stop if stalled for 250-5000ms
* Brake function with slow current decay, up to 65A/20ms
* ATmega328P PID controller for constant RPM
* LED driver 14-26V, 350mA, 10W, dimmable
* OLED display and rotary encoder, start/stop button
* Current limit indicator LED

## New Firmware 1.0.3

[Change Log v1.0.3](docs/CHANGELOG.md)

### Change Log PCB

### Rev1.3

* Current limit indicator LED
* Hardware circuit for current limit removed and overcurrent signal added to the MCU
* UCC27517DBVR MOSFET driver
* Kelvin connection to the shunt

### Rev1.1 & 1.2

* Input voltage detection
* PCB layout changes

### Rev1.0

* First prototype

## PCB and schematics

[![PCB Rev1.0](https://github.com/sascha432/motor_controller/blob/master/docs/images/pcb1_tn.jpg)](https://raw.githubusercontent.com/sascha432/motor_controller/master/docs/images/pcb1.jpg)

![PCB](https://github.com/sascha432/motor_controller/blob/master/docs/images/PCB.jpg)

[![Schematics](https://github.com/sascha432/motor_controller/blob/master/docs/images/schematics_tn.png)](https://raw.githubusercontent.com/sascha432/motor_controller/master/docs/images/schematics.png)

## First breadboard prototype

[![Prototype](https://github.com/sascha432/motor_controller/blob/master/docs/images/prototype_tn.jpg)](https://raw.githubusercontent.com/sascha432/motor_controller/master/docs/images/prototype.jpg)

## STL files for the drill press

[Remix of a great drill press](https://www.thingiverse.com/thing:3693804)

[STL download](https://raw.githubusercontent.com/sascha432/motor_controller/master/stl/drill_press_stl.zip)

[![Drill press 1](https://github.com/sascha432/motor_controller/blob/master/docs/images/dill_press_1_tn.jpg)](https://raw.githubusercontent.com/sascha432/motor_controller/master/docs/images/dill_press_1.jpg)

[![Drill press 2](https://github.com/sascha432/motor_controller/blob/master/docs/images/dill_press_2_tn.jpg)](https://raw.githubusercontent.com/sascha432/motor_controller/master/docs/images/dill_press_2.jpg)
