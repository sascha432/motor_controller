# DC Motor Controller with LED dimmer

Firmware for DC motor controller https://easyeda.com/sascha23095123423/dc-motor-controller

I am using this controller for a 500W drill press with variable speed 600-3500rpm.

* Input voltage 13.5-36V
* DC Motor controller 20A continuously, 31.25kHz PWM
* 1.2A MOSFET driver
* RPM or Voltage speed control
* Opto Interrupter for RPM sensing
* Adjustable current limit 1-40A
* Adjustable auto stop if stalled for 250-2500ms
* Brake function with slow current decay, up to 65A/20ms
* ATmega328P PID controller for constant RPM
* LED driver 14-26V, 350mA, 10W, dimmable
* OLED display and rotary encoder, start/stop button

# PCB and schematics

[![PCB Rev1.0](https://github.com/sascha432/motor_controller/blob/master/docs/images/pcb1_tn.jpg)](https://raw.githubusercontent.com/sascha432/motor_controller/master/docs/images/pcb1.jpg)

![PCB](https://github.com/sascha432/motor_controller/blob/master/docs/images/PCB.jpg)

[![Schematics](https://github.com/sascha432/motor_controller/blob/master/docs/images/schematics_tn.png)](https://raw.githubusercontent.com/sascha432/motor_controller/master/docs/images/schematics.png)

# First breadboard prototype

[![Prototype](https://github.com/sascha432/motor_controller/blob/master/docs/images/prototype_tn.jpg)](https://raw.githubusercontent.com/sascha432/motor_controller/master/docs/images/prototype.jpg)

# STL files for the drill press

[Remix of a great drill press](https://www.thingiverse.com/thing:3693804)

[STL download](https://raw.githubusercontent.com/sascha432/motor_controller/master/stl/drill_press_stl.zip)

[![Drill press 1](https://github.com/sascha432/motor_controller/blob/master/docs/images/dill_press_1_tn.jpg)](https://raw.githubusercontent.com/sascha432/motor_controller/master/docs/images/dill_press_1.jpg)

[![Drill press 2](https://github.com/sascha432/motor_controller/blob/master/docs/images/dill_press_2_tn.jpg)](https://raw.githubusercontent.com/sascha432/motor_controller/master/docs/images/dill_press_2.jpg)
