# 3D-printed-model-rocket-

This repository contains the 3D models, code and schematics for a 3D printed model rocket which has a GY-87 sensor module with an Arduino Pro Mini mounted in the payload compartment. The data is recorded on an Adafruit OpenLog module. Furhermore, it includes a Python data analysis program which converts the data to usable units and plots it. This data comprises of 3-axis accelerometer, gyroscope, magnetometer pressure and temperature data. The Python program includes acceleration matrix rotation using the gyroscope from the sensor's local frame of reference to the global one, as well as calibration value calculation, from which Z-axis gravity offset may be subtracted. Via integration the velocities and distances are derived, and plotted.

Technical information:
- Arduino libraries: i2cDevLib libraries for 
- Python dependancies: 

Issues with current design:
- insufficient ventilation in payload compartment for consistent temperature measurements
- 

Electronics information
- GY-87
- Arduino Pro Mini from SparkFun
- OpenLog from SparkFun

Things to note:
- config.txt is the config file for the OpenLog module, copy it to the SD card or change the baud rte on your config file to the appropriate one from the Arduino program
