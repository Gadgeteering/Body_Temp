/*!
 *  @file MAX30205.h
 *
 *   I2C Driver for Microchip's MAX30205 I2C Temp sensor
 *
 *  This is a library for the Adafruit MAX30205 breakout:
 *  http://www.adafruit.com/products/1782
 *
 *  Adafruit invests time and resources providing this open source code,
 *please support Adafruit and open-source hardware by purchasing products from
 *  Adafruit!
 *
 *
 *  BSD license (see license.txt)
 */

#ifndef _MAX30205_H
#define _MAX30205_H

#include "Adafruit_BusIO_Register.h"
#include "Arduino.h"
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>

#define MAX30205_I2CADDR_DEFAULT 0x48 ///< I2C address
#define MAX30205_I2CADDR_2ND 0x49 ///< I2C address
// Registers
#define MAX30205_TEMPERATURE    0x00  //  get temperature ,Read only
#define MAX30205_CONFIGURATION  0x01  //
#define MAX30205_THYST          0x02  //
#define MAX30205_TOS            0x03  //

#define MAX30205_REG_CONFIG_SHUTDOWN 0x0001   ///< shutdown config

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            MAX30205 Temp Sensor
 */
class MAX30205 : public Adafruit_Sensor {
public:
  MAX30205();
  bool begin();
  bool begin(TwoWire *theWire);
  bool begin(uint8_t addr);
  bool begin(uint8_t addr, TwoWire *theWire);

  bool init();
  float readTempC();
  float readTempF();
  uint8_t getResolution(void);
  void setResolution(uint8_t value);

  void shutdown_wake(boolean sw);
  void shutdown();
  void wake();

  void write16(uint8_t reg, uint16_t val);
  uint16_t read16(uint8_t reg);

  void write8(uint8_t reg, uint8_t val);
  uint8_t read8(uint8_t reg);

  /* Unified Sensor API Functions */
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  uint16_t _sensorID = 9808; ///< ID number for temperature
  Adafruit_I2CDevice *i2c_dev = NULL;
};

#endif
