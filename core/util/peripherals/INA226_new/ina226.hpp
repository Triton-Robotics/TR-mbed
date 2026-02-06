/*
 * Copyright (c) 2019 Branilson Luiz
 * INA226.hpp - Header file for the INA226 Bi-directional Current/Power Monitor
 * Mbed Library.
 * Version: 1.0.0
 * 
 * branilson (at) gmail dot com
 * Github: https://github.com/branilson/ina226_mbed_library
 * 
 * This program is free software: you can redistribute it and/or modify it un-
 * der the terms of the version 3 GNU General Public License as published by
 * the Free Software Foundation.
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FIT-
 * NESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef INA226_HPP_
#define INA226_HPP_

#include "mbed.h"

#define BIT_SOL  (0x8000)  // Shunt Voltage Over-Voltage
#define BIT_SUL  (0x4000)  // Shunt Voltage Under-Voltage
#define BIT_BOL  (0x2000)  // Bus Voltage Over-Voltage
#define BIT_BUL  (0x1000)  // Bus Voltage Under-Voltage
#define BIT_POL  (0x0800)  // Power Over-Limit
#define BIT_CNVR (0x0400)  // Conversion Ready
#define BIT_AFF  (0x0010)  // Alert Function Flag
#define BIT_CVRF (0x0008)  // Conversion Ready Flag
#define BIT_OVF  (0x0004)  // Math Overflow Flag
#define BIT_APOL (0x0002)  // Alert Polarity bit; sets the Alert pin polaritY
#define BIT_LEN  (0x0001)  // Alert Latch Enable; configures the latching feature of the Alert pin and Alert Flag bits

typedef enum  {
  AVERAGES_1             = 0b000,
  AVERAGES_4             = 0b001,
  AVERAGES_16            = 0b010,
  AVERAGES_64            = 0b011,
  AVERAGES_128           = 0b100,
  AVERAGES_256           = 0b101,
  AVERAGES_512           = 0b110,
  AVERAGES_1024          = 0b111
} ina226_averages_t;

typedef enum  {
  BUS_CONV_TIME_140US    = 0b000,
  BUS_CONV_TIME_204US    = 0b001,
  BUS_CONV_TIME_332US    = 0b010,
  BUS_CONV_TIME_588US    = 0b011,
  BUS_CONV_TIME_1100US   = 0b100,
  BUS_CONV_TIME_2116US   = 0b101,
  BUS_CONV_TIME_4156US   = 0b110,
  BUS_CONV_TIME_8244US   = 0b111
} ina226_busConvTime_t;

typedef enum  {
  SHUNT_CONV_TIME_140US   = 0b000,
  SHUNT_CONV_TIME_204US   = 0b001,
  SHUNT_CONV_TIME_332US   = 0b010,
  SHUNT_CONV_TIME_588US   = 0b011,
  SHUNT_CONV_TIME_1100US  = 0b100,
  SHUNT_CONV_TIME_2116US  = 0b101,
  SHUNT_CONV_TIME_4156US  = 0b110,
  SHUNT_CONV_TIME_8244US  = 0b111
} ina226_shuntConvTime_t;

typedef enum  {
  MODE_POWER_DOWN      = 0b000,
  MODE_SHUNT_TRIG      = 0b001,
  MODE_BUS_TRIG        = 0b010,
  MODE_SHUNT_BUS_TRIG  = 0b011,
  MODE_ADC_OFF         = 0b100,
  MODE_SHUNT_CONT      = 0b101,
  MODE_BUS_CONT        = 0b110,
  MODE_SHUNT_BUS_CONT  = 0b111,
} ina226_mode_t;

class ina226 {
 public:
  ina226(I2C& i2c, uint8_t address, int frequency);
  int setConfig(ina226_averages_t avg = AVERAGES_64,
   ina226_busConvTime_t busConvTime = BUS_CONV_TIME_1100US,
   ina226_shuntConvTime_t shuntConvTime = SHUNT_CONV_TIME_1100US,
   ina226_mode_t mode = MODE_SHUNT_BUS_CONT);
  int setCalibration(float rShuntValue = 0.01, float iMaxExpected = 8.191);
  // int setAlert(uint16_t val);
  float readShuntVoltage(void);
  float readCurrent(void);
  float readPower(void);
  float readBusVoltage(void);
  int readManufacturerID(void);
  int readDieID(void);
  int readCalibration(void);
  ina226_averages_t getAverages(void);
  ina226_busConvTime_t getBusConversionTime(void);
  ina226_shuntConvTime_t getShuntConversionTime(void);
  ina226_mode_t getMode(void);
  void enableShuntOverVoltageAlert(void);
  void enableShuntUnderVoltageAlert(void);
  void enableBusOvertVoltageAlert(void);
  void enableBusUnderVoltageAlert(void);
  void enableOverPowerAlert(void);
  void enableConversionReadyAlert(void);
  void setOverCurrentLimit(float current);
  void setBusVoltageLimit(float voltage);
  void setShuntVoltageLimit(float voltage);
  void setPowerLimit(float watts);
  void setAlertInvertedPolarity(bool inverted);
  void setAlertLatch(bool latch);
  bool isMathOverflow(void);
  bool isAlert(void);

 private:
  I2C& _i2c;
  const uint8_t i2c_addr;
  const int i2c_freq;
  float currentLSB, powerLSB, rShunt;
  const float vShuntMax = 0.08192f;
  const uint8_t REG_CONFIG = 0x00;
  const uint8_t REG_SHUNT_VOLTAGE = 0x01;
  const uint8_t REG_BUS_VOLTAGE = 0x02;
  const uint8_t REG_POWER = 0x03;
  const uint8_t REG_CURRENT = 0x04;
  const uint8_t REG_CALIBRATION = 0x05;
  const uint8_t REG_MASK = 0x06;
  const uint8_t REG_ALERT = 0x07;
  const uint8_t REG_MANUFACTURER_ID = 0xFE;
  const uint8_t REG_DIE_ID = 0xFF;
  int writeRegister16(uint8_t reg, uint16_t val);
  int16_t readRegister16(uint8_t reg);
  void setMask(uint16_t mask);
  uint16_t getMask(void);
};

#endif  // INA226_HPP_
