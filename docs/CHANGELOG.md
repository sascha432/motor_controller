# Changelog

## 1.0.5-dev

  - Fixed PWM mode running in velocity mode
  - Inverted current limit signal to match PCB Rev1.5
  - Improved current limit
  - Displaying average voltage to get rid of ADC noise
  - Fixed LED brightness display
  - Removed all dynamic memory allocations
  - Menu to select displaying Watt/Amps while the motor is running
  - Reduced overshoot of the PID controller after the current limit has been triggered

## 1.0.4

 - Using size optimized version of Adafruit SSD1306 to free 4.2KB flash memory
 - Using rising and falling edge for RPM detection to work more precisely
 - Reduced clock cycles required for the capture ISR
 - Added inline assembler for time critical parts
 - Reset defaults menu
 - Changed voltage detection analog reference to 1.1V
 - Reading buttons and rotary encoder via pin change interrupt
 - Display LED power
 - Implemented acceleration into the rotary encoder library
 - New rotary encoder algorithm with debouncing
 - Interrupt driven free running ADC
 - Replaced analogWrite to reduce code size
 - Display motor current or power while running
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
