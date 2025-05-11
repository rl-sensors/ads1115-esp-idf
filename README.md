# ADS1115 
A simple driver for the ADS1115 ADC over I2C.

Partially based on [this driver](https://github.com/Molorius/esp32-ads1115/tree/master).

Uses ESP-IDF version 5.4

### Settings

Run the `menuconfig` and set the `SDL` and `SCL` pins.

### Config

See page 25 of the [datasheet](https://www.ti.com/lit/ds/symlink/ads1115.pdf) and adjust the `config` constant.

The full config has 2 bytes (16 bits) - just set the binary codes for each setting.
Based on the `FSR` value adjust the voltage multiplier.