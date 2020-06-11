#ifndef __ADAFRUIT_SENSOR_CALIBRATION_HARDCODE__
#define __ADAFRUIT_SENSOR_CALIBRATION_HARDCODE__

#include "Adafruit_Sensor_Calibration.h"

#if defined(ADAFRUIT_SENSOR_CALIBRATION_HARDCODED)


/**!  @brief Class for managing hardcoded calibration
 * **/
class Adafruit_Sensor_Calibration_HC : public Adafruit_Sensor_Calibration {
public:
  bool begin(uint8_t eeprom_addr = 60);
  bool saveCalibration(void);
  bool loadCalibration(void);
  bool printSavedCalibration(void);

private:
  uint16_t ee_addr = 0;
};

#endif

#endif // include once
