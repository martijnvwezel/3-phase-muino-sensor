| Supported Targets | ESP32-S2 (not tested) | ESP32-S3 |
| ----------------- | -------- | -------- |



## How to use example

### Hardware Required

* A development board with a SOC which has a RISC-V ULP coprocessor (e.g., ESP32-S3)
* A USB cable for power supply and programming

#### Pin Assignment:

**Note:** The following pin assignments are used by default.

|                             | SDA   | SCL   |
| --------------------------- | ------| ------|
| ESP32-S2/S3 RTC I2C Master  | GPIO3 | GPIO2 |
| Muino Advance Sensor        | SDA   | SCL   |

**Note:** The SDA line can only be configured to use either GPIO1 or GPIO3 and the SCL line can only be configured to use either GPIO0 or GPIO2.
**Note:** This enables the internal pull-up resistors for the SDA/SCL lines by default. What is not needed because the lines are already being pulled-up


#### Steps to compile and upload
``` bash 

# Set device
CONFIG_IDF_TARGET="esp32s3"
# if already build
idf.py fullclean 
# activate chain
idf.py set-target esp32s3
# build
idf.py build
# flash, COM port is for each pc different..
idf.py -p COM5 flash
# see logs 
idf.py -p COM5  monitor --no-reset

# --no-reset so that jtag does not reset device

```
# Next steps to finish
This is a project that is work in progress, its hard and consumes alot of time :)
- [ ] Finetune the calibration functions to work with different ranges of values
- [ ] Add improvements like putting the light sensor to sleep or use less of the LED light
- [ ] Go to sleep but let WiFi shortly try to connect with fast connect wifi or other ways
- [ ] Move all the code to ESPHome so other users can use it
