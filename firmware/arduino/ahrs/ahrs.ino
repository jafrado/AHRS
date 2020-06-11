//AHRS Widget
//Test Output kernel
//Copyright (C) 2020 James F Dougherty <jafrado@gmail.com>
//Derived from AHRS Adafruit Sensor Library and AHRS Test App, SparkFun uBlox Library examples
//Added: Sparkfun uBlox library updates for NED/ECEF, hardcoded calibration for M0 without an EEPROM

// AHRS 
// Full orientation sensing using NXP/Madgwick/Mahony and a range of 9-DoF
// sensor sets.
// You *must* perform a magnetic calibration before this code will work.
//
// To view this data, use the Arduino Serial Monitor to watch the
// scrolling angles, or run the OrientationVisualiser example in Processing.
// Based on  https://github.com/PaulStoffregen/NXPMotionSense with adjustments
// to Adafruit Unified Sensor interface

//NOTE: See Adafruit_Sensor_Calibration.h for selection of where the Calibration offsets are stored

#define ADAFRUIT_SENSOR_CALIBRATION_HARDCODED

#include "Adafruit_Sensor_Calibration.h"
#include "Adafruit_AHRS.h"

//Sparkfun uBlox GPS Library 
//By: Nathan Seidle, Adapted from Example3_GetPosition by Thorsten von Eicken
//SparkFun Electronics
//Date: January 28rd, 2019
//License: MIT. See license file for more information but you can

#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS


Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

// uncomment one combo 9-DoF!
//#include "LSM6DS_LIS3MDL.h"  // can adjust to LSM6DS33, LSM6DS3U, LSM6DSOX...
//#include "LSM9DS.h"           // LSM9DS1 or LSM9DS0
#include "NXP_FXOS_FXAS.h"  // NXP 9-DoF breakout

// pick your filter! slower == better quality output
//Adafruit_NXPSensorFusion filter; // slowest
Adafruit_Madgwick filter;  // faster than NXP
//Adafruit_Mahony filter;  // fastest/smalleset


#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#elif defined(ADAFRUIT_SENSOR_CALIBRATION_HARDCODED)
  Adafruit_Sensor_Calibration_HC cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

SFE_UBLOX_GPS GPS;

#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10

//#define AHRS_DEBUG_OUTPUT

uint32_t timestamp;

void setup() {
  Serial.begin(115200);
  
  //Only for debug, output whether someone is connected or not
  while (!Serial) yield();

  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration helper");
  } else if (! cal.loadCalibration()) {
    Serial.println("No calibration loaded/found");
  } else {
    Serial.println("Loaded Calibration");
    cal.printSavedCalibration();
  }

  if (!init_sensors()) {
    Serial.println("Failed to find sensors");
    while (1) delay(10);
  }
  
  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();

  setup_sensors();
  filter.begin(FILTER_UPDATE_RATE_HZ);
  timestamp = millis();

  Wire.setClock(400000); // 400KHz


  //GPS Init
 
  //Assume that the U-Blox GPS is running at 9600 baud (the default) or at 38400 baud.
  //Loop until we're in sync and then ensure it's at 38400 baud.
  do {
    Serial.println("GPS: trying 115200 baud");
    Serial1.begin(38400);
    if (GPS.begin(Serial1) == true) break;

    delay(100);
    Serial.println("GPS: trying 9600 baud");
    Serial1.begin(9600);
    if (GPS.begin(Serial1) == true) {
        Serial.println("GPS: connected at 9600 baud, switching to 38400");
        GPS.setSerialRate(38400);
        delay(100);
    } else {
      //GPS.factoryReset();
        delay(2000); //Wait a bit before trying again to limit the Serial output
    }
  } while(1);
  
  Serial.println("GPS serial connected at 38400 baud");
  while (Serial1.available()) Serial1.read(); //Clear any latent chars in serial buffer
  
  //Serial.println("Press any key to send commands to begin Survey-In");
  //while (Serial.available() == 0) ; //Wait for user to press a key

  //GPS.setUART1Output(COM_TYPE_UBX|COM_TYPE_NMEA); //Set the UART port to output UBX only
  //GPS.setUART2Output(COM_TYPE_UBX); //Set the UART port to output UBX only
  GPS.setUART1Output(COM_TYPE_UBX);
  GPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  GPS.saveConfiguration(); //Save the current settings to flash and BBR
  delay(2000);

  //GPS.setNavigationFrequency(10); //Set output to 10 times a second
  GPS.setNavigationFrequency(5); //Set output to 1x a second
  //GPS.setNavigationFrequency(1); //Set output to 1x a second
  byte rate = GPS.getNavigationFrequency(); //Get the update rate of this module
  Serial.print("Current update rate:");
  Serial.println(rate);


  //This will pipe all NMEA sentences to the serial port so we can see them
  //GPS.setNMEAOutputPort(Serial);
  
  //Debug output on USB serial
  //GPS.enableDebugging(Serial); 

  
}


//Send AHRS and GPS data out serial port for our Application to render

void loop() {
  float roll, pitch, heading;
  float gx, gy, gz;
  static uint8_t counter = 0;
  static uint8_t gps_counter = 0;

  if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
    return;
  }
  
  timestamp = millis();
  // Read the motion sensors
  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);
  
#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("I2C took "); Serial.print(millis()-timestamp); Serial.println(" ms");
#endif

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);
  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important
  gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  // Update the SensorFusion filter
  filter.update(gx, gy, gz, 
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("Sensor Fusion Update took "); Serial.print(millis()-timestamp); Serial.println(" ms");
#endif


  // only print the calculated output once in a while
  if (counter++ <= PRINT_EVERY_N_UPDATES) {
    return;
  }
  // reset the counter
  counter = 0; 


#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("Raw: ");
  Serial.print(accel.acceleration.x, 4); Serial.print(", ");
  Serial.print(accel.acceleration.y, 4); Serial.print(", ");
  Serial.print(accel.acceleration.z, 4); Serial.print(", ");
  Serial.print(gx, 4); Serial.print(", ");
  Serial.print(gy, 4); Serial.print(", ");
  Serial.print(gz, 4); Serial.print(", ");
  Serial.print(mag.magnetic.x, 4); Serial.print(", ");
  Serial.print(mag.magnetic.y, 4); Serial.print(", ");
  Serial.print(mag.magnetic.z, 4); Serial.println("");
#endif

  // print the heading, pitch and roll
  //NOTE: this is computed from Quaternions so should not be susceptible to Gimbal Lock
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();

//  Serial.print("Orientation: ");
  Serial.print("$INS,");
  Serial.print(heading);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  //Serial.println(roll);
  Serial.print(roll);


  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);

#if 0
  //Not needed - Fusion library uses Quaternions internally to prevent Gimble lock
  //Serial.print("Quaternion: ");
  Serial.print(",");
  Serial.print(qw, 4);
  Serial.print(",");
  Serial.print(qx, 4);
  Serial.print(",");
  Serial.print(qy, 4);
  Serial.print(",");
  //Serial.println(qz, 4);  
  Serial.print(qz, 4);  
#endif

#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("AHRS Took "); Serial.print(millis()-timestamp); Serial.println(" ms");
#endif


  //if (millis() - timestamp > 1000)
  if (1)
  {
    timestamp = millis(); //Update the timer
#if 1
    long latitude = GPS.getLatitude();
    //Serial.print(F("Lat: "));
    Serial.print(",");
    Serial.print(latitude);

    long longitude = GPS.getLongitude();
    //Serial.print(F(" Long: "));
    Serial.print(",");
    Serial.print(longitude);
    //Serial.print(F(" (degrees * 10^-7)"));
#else
  long latitude = GPS.getHighResLatitude();
    //Serial.print(F("Lat: "));
    Serial.print(",");
    Serial.print(latitude);

    long longitude = GPS.getHighResLongitude();
    //Serial.print(F(" Long: "));
    Serial.print(",");
    Serial.print(longitude);
    //Serial.print(F(" (degrees * 10^-7)"));

#endif    
    Serial.print(",");
    long altitude = GPS.getAltitude();
    //Serial.print(F(" Alt: "));
    Serial.print(altitude);
    //Serial.print(F(" (mm)"));

    long altitudeMSL = GPS.getAltitudeMSL();
    Serial.print(",");
    Serial.print(altitudeMSL);
    

    Serial.print(",");
    byte SIV = GPS.getSIV();
    //Serial.print(F(" SIV: "));
    Serial.print(SIV);

    byte fixType = GPS.getFixType();
    Serial.print(",");
    Serial.print(fixType);
    
#if 0    
    //Serial.print(F(" Fix: "));
    if(fixType == 0) Serial.print(F("No fix"));
    else if(fixType == 1) Serial.print(F("Dead reckoning"));
    else if(fixType == 2) Serial.print(F("2D"));
    else if(fixType == 3) Serial.print(F("3D"));
    else if(fixType == 4) Serial.print(F("GNSS+Dead reckoning"));
#endif

    byte RTK = GPS.getCarrierSolutionType();
    //Serial.print(" RTK: ");
    Serial.print(",");
    Serial.print(RTK);
    
#if 0    
    if (RTK == 0) Serial.print(F("N/A"));
    if (RTK == 1) Serial.print(F("HPFF")); //High precision float fix
    if (RTK == 2) Serial.print(F("HPF")); //High precision fix
#endif

    long speed = GPS.getGroundSpeed();
    //Serial.print(F(" Speed: "));
    Serial.print(",");
    Serial.print(speed);
    //Serial.print(F(" (mm/s)"));


    long heading = GPS.getHeading();
    Serial.print(",");
    //Serial.print(F(" Heading: "));
    Serial.print(heading);
    //Serial.print(F(" (degrees * 10^-5)"));

#if 0
    //classical PDOP - frac meter accuracy
    Serial.print(",");
    int pDOP = GPS.getPDOP();
    //Serial.print(F(" pDOP: "));
    Serial.print(pDOP / 100.0, 2);
#else
    //RTK - report mm accuracy
    Serial.print(",");
    Serial.print(GPS.ecefPosInfo.pAcc, 2);
#endif


    

#if 1
  //ECEF or NED Position/Velocity
  if(1) { 
      //Serial.print(",");
      //Serial.print("E");
      
      Serial.print(",");
      //High precision component of ECEF X coordinate. Must be in the range of -99..+99. Precise coordinate in cm = ecefX + (ecefXHp * 1e-2).
      Serial.print(GPS.ecefPosInfo.ecefPosX + GPS.ecefPosInfo.ecefPosHPX * 1.0E-2);
      //Serial.print(",");
      //Serial.print(GPS.ecefPosInfo.ecefPosHPX);
   
      Serial.print(",");
      Serial.print(GPS.ecefPosInfo.ecefPosY + GPS.ecefPosInfo.ecefPosHPY * 1.0E-2);
      //Serial.print(",");
      //Serial.print(GPS.ecefPosInfo.ecefPosHPY);
      Serial.print(",");
      Serial.print(GPS.ecefPosInfo.ecefPosZ + GPS.ecefPosInfo.ecefPosHPZ * 1.0E-2);
      //Serial.print(",");
      //Serial.print(GPS.ecefPosInfo.ecefPosHPZ);

      Serial.print(",");
      Serial.print(GPS.ecefPosInfo.ecefVelX);
      Serial.print(",");
      Serial.print(GPS.ecefPosInfo.ecefVelY);
      Serial.print(",");
      Serial.print(GPS.ecefPosInfo.ecefVelZ);
      
  } 
  else { 

    // From the uBlox protocol spec
    //The NED frame is defined as the local topological system at the reference station. The relative position vector components in this message, 
    // along with their associated accuracies, are given in that local topological system. This message contains the relative position vector from 
    // the Reference Station to the Rover, including accuracy figures, in the local topological system defined at the reference station 
    // So, while NED is preferred, it requires 2 GPS receivers to operate, and if the local station is the same as the rover, you will be getting
    // true NED from the power-up (home) position
    
    //Serial.print(",");
    //Serial.print("N");
    
    Serial.print(",");
    Serial.print(GPS.relPosInfo.relPosN + GPS.relPosInfo.relPosHPN  * 1.0E-2);
    Serial.print(",");
    Serial.print(GPS.relPosInfo.relPosE + GPS.relPosInfo.relPosHPE * 1.0E-2);
    Serial.print(",");
    Serial.print(GPS.relPosInfo.relPosD + GPS.relPosInfo.relPosHPD * 1.0E-2);

    Serial.print(",");
    Serial.print(GPS.relPosInfo.velN);
    Serial.print(",");
    Serial.print(GPS.relPosInfo.velE);
    Serial.print(",");
    Serial.print(GPS.relPosInfo.velD);
            
    }
    Serial.println();
    if (gps_counter++ >= 10) { 
    //pull these last to prevent timing issues
#if 1
        GPS.getECEFPOSHP();
        GPS.getVELECEF();    
#else
        GPS.getRELPOSNED();
        GPS.getVELNED();        
#endif        
        gps_counter = 0;
      }
#else
    Serial.println();
#endif    

  }
  
#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("GPS took "); Serial.print(millis()-timestamp); Serial.println(" ms");
#endif

 

}
