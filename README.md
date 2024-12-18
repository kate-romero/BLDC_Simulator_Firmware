# BLDC Simulator
## Description
This repository contains the firmware for a BLDC Simulator. The simulator serves as a motor controller and diagnostic tool for rotary actuators. This code was written by intern Kate Romero for [2G Engineering](https://www.2g-eng.com/).
### Watch demo at: [https://youtu.be/7H5ZqkXBpd8](https://youtu.be/7H5ZqkXBpd8)
### Modes of Operation
- **Actuator Only**
  - **Position**: Manually spin controller wheel to send simulated motor positions to actuator via Hall states. Can force illegal Hall states.
  - **Velocity**: Manually spin controller wheel to select a direction and rpm. Controller will send simulated Hall signals to the actuator at the appropriate rate to simulate motor rotation.
- **Motor Only**: Receive Hall signals from the motor. Observe position, velocity, Hall state progression, and invalid Hall state count. Can clear invalid Hall state count.
- **Passthrough**: Receive Hall signals from the motor and pass them to the actuator. Observe position, velocity, Hall state progression, and invalid Hall state count. Can clear invalid Hall state count. Useful for determining if the correct signals are being sent/received between motor and actuator. If correct signals are being sent/received, useful for testing/observing interaction between motor and actuator.
- **Intercept**
  - **Position**: Essentially runs **Motor Only** and **Actuator Only: Position** simultaneously. Useful for testing/observing interaction between motor and actuator when the actuator receives incorrect signals.
  - **Velocity**: Essentially runs **Motor Only** and **Actuator Only: Velocity** simultaneously. Useful for testing/observing interaction between motor and actuator when the actuator receives incorrect signals.
### Notes
- Tasks coordinated across both of the RP2040's cores to maximize sustainable motor RPM
  - core0: read signals from motor
  - core1: monitor user input, update OLED display update Neopixels, send signals to actuator
  - control flags set by user input are communicated between cores via RP2040 8 element FIFO queue
  - display values are passed to core1 via custom print buffers

- Files saved as type .cpp for compatibility with Arduino IDE. Actual language is majority C
## External Resources
- [pixelformer](https://www.qualibyte.com/pixelformer/): used to create OLED screen [pixel maps](/screen_pngs)
- [image2cpp](https://javl.github.io/image2cpp/): used to convert pixel maps into [bitmaps](/screen_pxs)
- [Adafruit_BusIO](https://github.com/adafruit/Adafruit_BusIO)
- [Adafruit_GFX_Library](https://github.com/adafruit/Adafruit-GFX-Library)
- [Adafruit_NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel)
- [Adafruit_seesaw_Library](https://github.com/adafruit/Adafruit_Seesaw)
- [Adafruit_SH110X](https://github.com/adafruit/Adafruit_SH110x)
- [Adafruit_SSD1306](https://github.com/adafruit/Adafruit_SSD1306)
- [Adafruit ST7735 and ST7789 Library](https://github.com/adafruit/Adafruit-ST7735-Library)
- [SD Library for Arduino](https://github.com/arduino-libraries/SD)
- [U8g2_Arduino](https://github.com/olikraus/U8g2_Arduino)
- [U8g2_for_Adafruit_GFX](https://github.com/olikraus/U8g2_for_Adafruit_GFX)
---
All files and materials not listed in External Resources are property of 2G Engineering. Posted publicly with permission.
