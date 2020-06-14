# AHRS
Attitude and Heading Reference System provides an interface for compass bearing/heading, and pose (yaw, pitch, roll), as well as navigation basics - GPS position, altitude, speed. Its the guts of what you would use for an aerospace control system. A reference Inertial Navigation System (INS) firmware is provided for testing and integration using Arduino and the Adafruit/Sparkfun libraries running on a Cortex M0 with the Sparkfun RTK GPS receivers, the Adafruit 9DOF sensor boards, and the Madgwick/Mahoney or NXP sensor fusion libraries are used for 9DOF IMU output in NED format. 

This software is a starting point for integrating both attitude/pose and centimeter level positioning into your system.


# Supported Hardware

A suitable AHRS Platform may be build from the below components:
- [Adafruit Feather M0 Basic Proto ](https://www.adafruit.com/product/2772)
- [NXP Precision 9DoF breakout (FXOS8700 3-Axis accel/mag, FXAS21002 3-axis gyro)](https://www.adafruit.com/product/3463)
- [SparkFun GPS-RTK2 Board - ZED-F9P ](https://www.sparkfun.com/products/15136)
- [SparkFun GPS-RTK Board - NEO-M8P-2 ](https://www.sparkfun.com/products/15005)


Follow [Kevin's excellent guide](https://learn.adafruit.com/nxp-precision-9dof-breakout) and [LadyAda's excellent guide on Sensorfusion](https://learn.adafruit.com/how-to-fuse-motion-sensor-data-into-ahrs-orientation-euler-quaternions) to setup the Hardware. You'll be using the I2C port on the M0 for the FXOS7800/FXAS21002 communication interface. Follow [Nathan's excellent guide on RTK GPS Setup](https://learn.sparkfun.com/tutorials/gps-rtk-hookup-guide). For standalone (non NTRIP/networked RTCM corrections), note that the ZED-F9P has two serial ports and should be used as the GPS Master by the ARM Cortex M0. In this configuration, the RTCM correction is sourced from the single output port of the NEO-M8P via the alternate UART port on the the ZED-F9P while the primary ZED-F9P UART is connected to the Arduino M0. This configuration allows for RTK Float mode in a standalone setup.

The following Arduino libraries are included in the firmware, and some modifications have been made to the Sparkfun GPS library for ECEF and NED coordinate systems.


- [Adafruit AHRS](https://github.com/adafruit/Adafruit_AHRS)
- [Adafruit Sensor](https://github.com/adafruit/Adafruit_Sensor) (modified for use with hardcoded config)
- [Adafruit Sensor Calibration](https://github.com/adafruit/Adafruit_Sensor_Calibration) (modified for use with hardcoded config)
- [Adafruit FXOS8700 Driver](https://github.com/adafruit/Adafruit_FXOS8700)
- [Adafruit FXAS2102C Driver](https://github.com/adafruit/Adafruit_FXAS21002C)
- [Sparkfun uBlox Driver](https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library) (modified for ECEF/NED)

All of the libraries and their source code are included in one directory for easy compile/grok of the sources
and it's behavior.

## Hardware Reference

The below references are for the open source hardware designs used in this Arduino sketch ("firmware"):

- [NXP FXOS8700/FXAS2102C](https://github.com/adafruit/Adafruit-FXOS8700-FXAS21002-9-DoF-Breakout-PCB)
- [ATSAMD21G18 ARM Cortex M0](https://github.com/adafruit/Adafruit-Feather-M0-Basic-Proto-PCB)
- [uBlox NEO-M8P high precision, RTK ready GNSS module](https://github.com/sparkfun/Qwiic_GPS-RTK)
- [uBlox ZED-F9P RTK module from ublox](https://github.com/sparkfun/Qwiic_GPS-RTK2)





# Software Prerequisites


## Arduino Firmware

Use Arduino 1.8/8 or later, open the sketch "ahrs.ino" and build. Once programmed, connect the Feather M0 USB port to the hosts system USB port. 
``
## Qt UI
- Qt 5.12.5 or later 
- Windows - Microsoft Visual Studio 2019 Community Edition
- Linux - GNU Compiler Collection (g++) 5.4 and later

To build, simply launch QTCreator and open the project file:

``
qtcreator AHRS.pro
``

Once built, connect to the USB Serial port detected by the system.



