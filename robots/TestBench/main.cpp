/*
 * Copyright (c) 2019 Branilson Luiz
 * main.hpp - INA226: polling a single device example using the ina226
 * Mbed Library.
 * 
 * (c) 2019, Branilson Luiz
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

#include "mbed.h"
#include "main.h"
#include "util/peripherals/INA226_new/ina226.hpp"

// Serial pc(USBTX, USBRX);
DigitalOut myled(LED1);
DJIMotor feeder(5, CANHandler::CANBUS_2, C610);
I2C i2c(PB_7, PB_6);
unsigned const int I2C_FREQ = 400000;
const int ina_addr = 0x80;
const float current_limit = 1.0;
ina226 ina(i2c, ina_addr, I2C_FREQ); 

int main() {
  printf("INA226 TEST Program. (BUILD:[" __DATE__ "/" __TIME__ "])\n");
  int count = 1;

  // Device configuration. See header file for details.
  printf("INA226 Config return: %d\n", ina.setConfig());  // default configuration
  printf("INA226 Calibration return: %d\n", ina.setCalibration());  // default calibration
  ina.enableShuntOverVoltageAlert();
  ina.setOverCurrentLimit(current_limit);

  while (1)  {
    printf("\n%d:\n", count);
    printf("Device %xh: ManID %d, DieID %d, Cal %d, ShuntV %+2.6fV, %+2.6fV, %+2.6fA, %+2.6fW\n",
     ina_addr,
     ina.readManufacturerID(),
     ina.readDieID(),
     ina.readCalibration(),
     ina.readShuntVoltage(),
     ina.readBusVoltage(),
     ina.readCurrent(),
     ina.readPower());
    if (ina.isAlert()) {
      printf("Overcurrent detected\n");
    }

    myled = !myled;
    wait_us(1000000); // Wait one second.
    count++;
  }
}