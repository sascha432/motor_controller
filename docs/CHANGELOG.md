# Changelog

## 1.0.4

 - Using size optimized version of Adafruit SSD1306 to free 4.2KB flash memory
 - Using rising and falling edge for RPM detection to work more precisely
 - Reduced clock cycles required for the capture ISR
 - Added inline assembler for time critical parts
 - Motor setting RPM/Volt for minimum duty cycle and pid controller
 - Command to test no load RPM/Volt
 - Reset defaults menu
 - Changed voltage detection analog reference to 1.1V
 - Reading buttons and rotary encoder via pin change interrupt
 - Display LED power
 - Implemented acceleration into the rotary encoder library
 - New rotary encoder algorithm with debouncing
 - Interrupt driven free running ADC
 - Replaced analogWrite to reduce code size
 - Displaying motor current while running
 - Option to display Ampere or Watt while running
 - Enabled WDT while motor is running to turn it off if the software hangs
 - Rewritten the PID controller

## 1.0.3

- No display refresh with active current limit (I2C causes interferences with interrupts and timing issues)
- Current limit indicator LED
- Rewritten parts in C++
- Fixed issues with current limit in velocity mode

## 1.0.2

- Fixed displaying 0.0% RPM set point after reset
- Increased start duty cycle for velocity mode from 4% to 15%
- Max. PWM config option
- Improved current limit
- Tune PID settings menu while motor is running
- Dynamic averaging of the RPM sense pulses depending on the set RPM
- Improved menu acceleration
- Support for PCB Rev1.3
- Reduced min. RPM in velocity mode to 300

## 1.0.1

- Optimized code, ~2KB size reduction
- Brake enable/disable function
- Current limit indicator

## 1.0.0

- Initial version
