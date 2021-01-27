# 3D-printed-model-rocket-

This repository contains the 3D models, code and schematics for a 3D printed model rocket which has a GY-87 sensor module with an Arduino Pro Mini mounted in the payload compartment. The data is recorded on an Adafruit OpenLog module. Furhermore, it includes a Python data analysis program which converts the data to usable units and plots it. This data comprises of 3-axis accelerometer, gyroscope, magnetometer pressure and temperature data. The Python program includes acceleration matrix rotation using the gyroscope from the sensor's local frame of reference to the global one, as well as calibration value calculation, from which Z-axis gravity offset may be subtracted. Via integration the velocities and distances are derived, and plotted.

Technical information:
- Arduino libraries: i2cDevLib libraries for MPU6050, HMC5883L and BMP085
- Python dependancies: pandas, numpy, scipy and matplotlib
- 3D prining material used: PETG
- The rocket has MASS grams using PETG, which is a high amout of mass for its size, so the delay for deploying the parachute should be small. a B6-2 engine did not deploy the parachute on time, but a C6-3 engine worked fine. 

Rocket Assembly Instructions:
- 3D print all parts (you may wish to use translucent material for the payload compartment to see the indicator loghts)
- the fueselage is glued to the engine assmebly
- the parachute dimensions used for 143 grams: The radius of the vent hole is 4 cm, the length of the canopy is 19cm, and the length of the attachment section is 1cm. In total, the tarp has a 24cm radius, that is, a diameter of 48cm.
- internally, a rope is used to connect the bottom of the payload compartment to the fuesalage, which connects to the parachute
- you may wish to add insulation tape to the fueslage - payload compartment connection to increase friction
- payload and engine mounted, nose cone and engine cap screwed in


Issues with current design:
- insufficient ventilation in payload compartment for consistent temperature measurements
- very high drift after integration

Electronics information
- GY-87
- Arduino Pro Mini from SparkFun
- OpenLog from SparkFun
- dd05cvsa_05 charge controller
- generic 90 mAh LiPo battery
- electronics are seperated into 3 circular PCBs - the top one contains the OpenLog and serial wire breakouts that connect to the serial line. A 4-switch dip switch is present, 2 switches intersecting the serial line's connection to the OpenLog (you cannot upload via serial flasher and have OpenLog connected to the serial line, so this switch is used), and one switch connected to one of the battery's terminals. On the central board is the GY-87 module. On the bottom board is the Arduino Pro Mini and charge controler.
- !!!!!!!!!!!!!!!!!! WIRING DIAGRAM !!!!!!!!!!!!!!

Things to note:
- config.txt is the config file for the OpenLog module, copy it to the SD card or change the baud rte on your config file to the appropriate one from the Arduino program

