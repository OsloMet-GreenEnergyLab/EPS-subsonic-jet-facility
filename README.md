# EPS-subsonic-jet-facility
Technical information &amp; 3d models of the sub-sonic jet facility in the green energy lab

## Features

- Fan Control:
Adjustable fan speed via potentiometer and PWM output, with real-time RPM monitoring using a tachometer input.

- Speaker Control:
Potential for speaker volume adjustment via a second potentiometer.

- LCD Displays:
Two RGB LCD displays show current fan speed and speaker volume.

- Switch Control:
Dedicated physical switches to toggle power to the fan and (optionally) the speaker.

- Multicore Task Management:
Fan and switch handling tasks are pinned to separate ESP32 cores for parallel processing.

## Installation Instructions

### Hardware Requirements:

* ESP32 Development Board
* Fan (with tachometer feedback output)
* RGB LCD screens (I2C interface)
* TCA9548A I2C multiplexer
* Potentiometers (for fan speed and volume)
* Switches (for power and speaker control)
* LEDs (for status indication)
* Supporting resistors and wiring

### Software Requirements:

* Arduino IDE

* ESP32 Board Support for Arduino

## Arduino Libraries:

* Wire.h

* Arduino.h

* rgb_lcd.h

### Installation Steps:

1. Clone this repository: -```bash git clone https://github.com/OsloMet-Green-energy-lab/EPS-subsonic-jet-facility.git```

2. Open esp-code.ino in the Arduino IDE.

3. Install necessary libraries via the Library Manager.

4. Select the NODEMCU ESP32 board and correct COM port.

5. Upload code.

## ESP32 Pinout

| Function                     | ESP32 Pin | Description                           |
|:-----------------------------|:----------|:--------------------------------------|
| Power Switch Input           | 5         | Toggles fan system ON/OFF             |
| Speaker Switch Input         | 4         | (Optional, currently commented)       |
| Fan Speed Potentiometer      | 34        | Analog read for fan speed setting     |
| Speaker Volume Potentiometer | 35        | Analog read for speaker volume        |
| Power LED Output             | 13        | Indicates fan system status           |
| Speaker LED Output           | 12        | (Optional, currently commented)       |
| Fan PWM Control              | 25        | PWM signal to control fan speed       |
| Fan Tachometer Input         | 14        | Reads pulses for RPM calculation      |
| I2C SDA                      | 21        | I2C data line for LCDs & multiplexer  |
| I2C SCL                      | 22        | I2C clock line for LCDs & multiplexer |
## Notes

Kickstart: Fan starts at 100% PWM for 5 seconds on boot.

I2C Multiplexer: The project uses the TCA9548A I2C switch to manage multiple identical LCDs on the same I2C bus.
