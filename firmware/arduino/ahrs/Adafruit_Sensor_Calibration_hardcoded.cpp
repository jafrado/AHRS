#include "Adafruit_Sensor_Calibration.h"

#if defined(ADAFRUIT_SENSOR_CALIBRATION_HARDCODED)

/**************************************************************************/
/*!
    @brief Initializes Flash and filesystem
    @returns False if any failure to initialize flash or filesys
*/
/**************************************************************************/
bool Adafruit_Sensor_Calibration_HC::begin(uint8_t eeprom_addr) {
    return true;
}

bool Adafruit_Sensor_Calibration_HC::saveCalibration(void) {
  return printSavedCalibration();
}


//load hardcoded values from calibration
bool Adafruit_Sensor_Calibration_HC::loadCalibration(void) 
{

    /**! XYZ vector of offsets for zero-g, in m/s^2 */
    //float accel_zerog[3]={0.00,0.00,0.00,};

    /**! XYZ vector of offsets for zero-rate, in rad/s */
    //float gyro_zerorate[3]={0.00,0.00,0.00,};

    /**! XYZ vector of offsets for hard iron calibration (in uT) */
    //float mag_hardiron[3]={-10.85,-45.84,-50.92,};

    /**! The 3x3 matrix for soft-iron calibration (unitless) */
    //float mag_softiron[9]={1.00,-0.02,0.01,-0.02,0.99,0.01,0.01,0.01,1.01,};

    /**! The magnetic field magnitude in uTesla */
    //float mag_field=46.26;

      /**! XYZ vector of offsets for zero-g, in m/s^2 */
    accel_zerog[0] = 0.00;
    accel_zerog[1] = 0.00;
    accel_zerog[2] = 0.00;

    /**! XYZ vector of offsets for zero-rate, in rad/s */
    gyro_zerorate[0] = 0.00;
    gyro_zerorate[1] = 0.00;
    gyro_zerorate[2] = 0.00;
           
    /**! XYZ vector of offsets for hard iron calibration (in uT) */
    mag_hardiron[0] = -10.85;
    mag_hardiron[1] = -45.84;
    mag_hardiron[2] = -50.92 ; 

    /**! The 3x3 matrix for soft-iron calibration (unitless) */
    mag_softiron[0] = 1.00;
    mag_softiron[1] = -0.02;
    mag_softiron[2] = 0.01;

    mag_softiron[3] = -0.02;
    mag_softiron[4] = 0.99;
    mag_softiron[5] = 0.01;

    mag_softiron[9] = 0.01;
    mag_softiron[9] = 0.01;
    mag_softiron[9] = 1.01;
      
      
    /**! The magnetic field magnitude in uTesla */
    mag_field=46.26;

 
    return true;
}

bool Adafruit_Sensor_Calibration_HC::printSavedCalibration(void) 
{
  Serial.println("--------------------- SNIP --------------------\n");
    uint16_t i;  
  //zeroacc vec
  Serial.println("/**! XYZ vector of offsets for zero-g, in m/s^2 */");
  Serial.print("float accel_zerog[3]={");
  for (i = 0; i < 3; i++){
    Serial.print(accel_zerog[i]);
    Serial.print(",");
  }
  
  //zerogyro vec  
  Serial.println("};");
  Serial.println("/**! XYZ vector of offsets for zero-rate, in rad/s */");
  Serial.print("float gyro_zerorate[3]={");
  for (i = 0; i < 3; i++){
    Serial.print(gyro_zerorate[i]);
    Serial.print(",");
  }
  Serial.println("};");

  //hardiron vec
  Serial.println("/**! XYZ vector of offsets for hard iron calibration (in uT) */");
  Serial.print("float mag_hardiron[3]={");
  for (i = 0; i < 3; i++){
    Serial.print(mag_hardiron[i]);
    Serial.print(",");
  }
  Serial.println("};");
  
  //softiron 3x3
  Serial.println("/**! The 3x3 matrix for soft-iron calibration (unitless) */"); 
  Serial.print("float mag_softiron[9]={");
  for (i = 0; i < 9; i++){
    Serial.print(mag_softiron[i]);
    Serial.print(",");
  }
  Serial.println("};");

  Serial.println("/**! The magnetic field magnitude in uTesla */");
  Serial.print("float mag_field=");
  Serial.print(mag_field);
  Serial.println(";");

  Serial.println("\n--------------------- SNIP -------------------- ");

  return true;
}

#endif
