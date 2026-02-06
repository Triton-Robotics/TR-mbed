/*
 * Copyright (c) 2019 Branilson Luiz
 * INA226.cpp - Class file for the INA226 Bi-directional Current/Power Monitor
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


#include "ina226.hpp"

ina226::ina226(I2C& i2c, uint8_t address, int frequency)
  : _i2c(i2c),
    i2c_addr(address),
    i2c_freq(frequency) {
    _i2c.frequency(i2c_freq);
}

int ina226::setConfig(ina226_averages_t avg,
 ina226_busConvTime_t busConvTime,
 ina226_shuntConvTime_t shuntConvTime,
 ina226_mode_t mode)  {
  uint16_t config = 0;
  config |= (avg << 9 | busConvTime << 6 | shuntConvTime << 3 | mode);
  return writeRegister16(REG_CONFIG, config);
}

int ina226::setCalibration(float rShuntValue, float iMaxExpected)  {
  float test = iMaxExpected*rShuntValue;
  if ((test <= vShuntMax) && (test > -vShuntMax)) {
    uint16_t calibrationValue;
    float minimumLSB;
    rShunt = rShuntValue;
    minimumLSB = iMaxExpected / 32768;
    currentLSB = (uint16_t)(minimumLSB * 100000000);
    currentLSB /= 100000000;
    currentLSB /= 0.0001;
    currentLSB = ceil(currentLSB);
    currentLSB *= 0.0001;
    powerLSB = currentLSB * 25;
    calibrationValue = (uint16_t)((0.00512) / (currentLSB * rShuntValue));
    return writeRegister16(REG_CALIBRATION, calibrationValue);
  } else {
      return -1;  // Return in case of invalid parameters entered
  }
}

float ina226::readShuntVoltage(void) {
  return (readRegister16(REG_SHUNT_VOLTAGE) * 0.0000025);
}

float ina226::readCurrent(void)  {
  return (readRegister16(REG_CURRENT) * currentLSB);
}

float ina226::readPower(void)  {
  return (readRegister16(REG_POWER) * powerLSB);
}

float ina226::readBusVoltage(void)  {
  int16_t voltage;
  voltage = readRegister16(REG_BUS_VOLTAGE);
  return (voltage * 0.00125);
}

int ina226::readManufacturerID(void)  {
  return (readRegister16(REG_MANUFACTURER_ID));
}

int ina226::readDieID(void)  {
  return (readRegister16(REG_DIE_ID));
}

int ina226::readCalibration(void)  {
  return (readRegister16(REG_CALIBRATION));
}

ina226_averages_t ina226::getAverages(void) {
  uint16_t value;
  value = readRegister16(REG_CONFIG);
  value &= 0b0000111000000000;
  value >>= 9;
  return (ina226_averages_t)value;
}

ina226_busConvTime_t ina226::getBusConversionTime(void) {
  uint16_t value;
  value = readRegister16(REG_CONFIG);
  value &= 0b0000000111000000;
  value >>= 6;
  return (ina226_busConvTime_t)value;
}

ina226_shuntConvTime_t ina226::getShuntConversionTime(void) {
  uint16_t value;
  value = readRegister16(REG_CONFIG);
  value &= 0b0000000000111000;
  value >>= 3;
  return (ina226_shuntConvTime_t)value;
}

ina226_mode_t ina226::getMode(void) {
  uint16_t value;
  value = readRegister16(REG_CONFIG);
  value &= 0b0000000000000111;
  return (ina226_mode_t)value;
}

void ina226::enableShuntOverVoltageAlert(void) {
  writeRegister16(REG_MASK, BIT_SOL);
}

void ina226::enableShuntUnderVoltageAlert(void) {
  writeRegister16(REG_MASK, BIT_SUL);
}

void ina226::enableBusOvertVoltageAlert(void) {
  writeRegister16(REG_MASK, BIT_BOL);
}

void ina226::enableBusUnderVoltageAlert(void) {
  writeRegister16(REG_MASK, BIT_BUL);
}

void ina226::enableOverPowerAlert(void) {
  writeRegister16(REG_MASK, BIT_POL);
}

void ina226::enableConversionReadyAlert(void) {
  writeRegister16(REG_MASK, BIT_CNVR);
}

// Workaround using Shunt Over/Under Voltage feature
void ina226::setOverCurrentLimit(float current) {
  // Use enableShuntOverVoltageAlert() or enableBusUnderVoltageAlert() before
  uint16_t shunt_voltage = current * rShunt  * 400000;  // vShunt LSB = 2.5uV
  writeRegister16(REG_ALERT, shunt_voltage);
}

void ina226::setBusVoltageLimit(float voltage) {
  uint16_t value = voltage / 0.00125;
  writeRegister16(REG_ALERT, value);
}

void ina226::setShuntVoltageLimit(float voltage) {
  uint16_t value = voltage * 400000;  // vShunt LSB = 2.5uV
  writeRegister16(REG_ALERT, value);
}

void ina226::setPowerLimit(float watts) {
  uint16_t value = watts / powerLSB;
  writeRegister16(REG_ALERT, value);
}

void ina226::setAlertInvertedPolarity(bool inverted) {
  uint16_t temp = getMask();
  if (inverted) {
    temp |= BIT_APOL;
  } else {
      temp &= ~BIT_APOL;
    }
  setMask(temp);
}

void ina226::setAlertLatch(bool latch) {
  uint16_t temp = getMask();
  if (latch) {
      temp |= BIT_LEN;
  } else {
      temp &= ~BIT_LEN;
    }
  setMask(temp);
}

bool ina226::isMathOverflow(void) {
  return ((getMask() & BIT_OVF) == BIT_OVF);
}

bool ina226::isAlert(void) {
  return ((getMask() & BIT_AFF) == BIT_AFF);
}

int16_t ina226::readRegister16(uint8_t reg)  {
  char data[2];
  _i2c.write(i2c_addr, reinterpret_cast<char*>(&reg), 1);
  _i2c.read(i2c_addr, data, 2);
  uint16_t value = data[0] << 8 | data[1];
  return value;
}

int ina226::writeRegister16(uint8_t reg, uint16_t val)  {
  char data[3];
  data[0] = reg;
  data[1] = (val >> 8) & 0xff;
  data[2] = val & 0xff;
  return _i2c.write(i2c_addr, data, 3);
}

void ina226::setMask(uint16_t mask) {
  writeRegister16(REG_MASK, mask);
}

uint16_t ina226::getMask(void) {
  return readRegister16(REG_MASK);
}
