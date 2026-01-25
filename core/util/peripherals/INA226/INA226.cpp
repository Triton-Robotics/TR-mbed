/*
 * mbed library program
 *  INA226 High-or Low-Side Measurement,Bi-Directional CURRENT/POWER MONITOR with I2C Interface
 *  by Texas Instruments
 *
 * Copyright (c) 2015,'17 Kenji Arai / JH1PJL
 *  http://www.page.sannet.ne.jp/kenjia/index.html
 *  http://mbed.org/users/kenjiArai/
 *      Created: January    25th, 2015
 *      Revised: August     23rd, 2017
 */

#include    "mbed.h"
#include    "INA226.h"

INA226::INA226 (PinName p_sda, PinName p_scl, const INA226_TypeDef *ina226_parameter)
 : _i2c_p(new I2C(p_sda, p_scl)), _i2c(*_i2c_p)
{
    _i2c.frequency(400000);
    ina226_set_data = *ina226_parameter;
    initialize();
}

INA226::INA226 (PinName p_sda, PinName p_scl, uint8_t addr)
 : _i2c_p(new I2C(p_sda, p_scl)), _i2c(*_i2c_p)
{
    _i2c.frequency(400000);
    // Use standard setting
    ina226_set_data = ina226_std_paramtr;
    // Change user defined address
    ina226_set_data.addr = addr;
    initialize();
}

INA226::INA226 (PinName p_sda, PinName p_scl)
 : _i2c_p(new I2C(p_sda, p_scl)), _i2c(*_i2c_p)
{
    _i2c.frequency(400000);
    // Use standard setting
    ina226_set_data = ina226_std_paramtr;
    initialize();
}

INA226::INA226 (I2C& p_i2c, const INA226_TypeDef *ina226_parameter)
 : _i2c(p_i2c)
{
    _i2c.frequency(400000);
    ina226_set_data = *ina226_parameter;
    initialize();
}

INA226::INA226 (I2C& p_i2c, uint8_t addr)
 : _i2c(p_i2c)
{
    _i2c.frequency(400000);
    // Use standard setting
    ina226_set_data = ina226_std_paramtr;
    // Change user defined address
    ina226_set_data.addr = addr;
    initialize();
}

INA226::INA226 (I2C& p_i2c)
 : _i2c(p_i2c)
{
    _i2c.frequency(400000);
    // Use standard setting
    ina226_set_data = ina226_std_paramtr;
    initialize();
}

/////////////// Read Current //////////////////////////////
float INA226::read_current()
{
    dt[0] = INA226_CURRENT;
    _i2c.write((int)ina226_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina226_set_data.addr, (char *)dt, 2, false);
    int16_t data = (dt[0] << 8) | dt[1];
    return (float)data /1000;
}

/////////////// Read Power ////////////////////////////////
float INA226::read_power()
{
    dt[0] = INA226_POWER;
    _i2c.write((int)ina226_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina226_set_data.addr, (char *)dt, 2, false);
    int16_t data = (dt[0] << 8) | dt[1];
    return (float)data * 25 / 1000;
}

/////////////// Read Bus_volt /////////////////////////////
float INA226::read_bus_voltage()
{
    dt[0] = INA226_BUS_VOLT;
    _i2c.write((int)ina226_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina226_set_data.addr, (char *)dt, 2, false);
    int16_t data = (dt[0] << 8) | dt[1];
    return (float)data * 1.25f / 1000.0f;
}

/////////////// Read Shunt volt ///////////////////////////
float INA226::read_shunt_voltage()
{
    dt[0] = INA226_SHUNT_V;
    _i2c.write((int)ina226_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina226_set_data.addr, (char *)dt, 2, false);
    int16_t data = (dt[0] << 8) | dt[1];
    return (float)data * 2.5f / 1000.0f;
}

float INA226::read_current_by_shuntvolt()
{
    dt[0] = INA226_SHUNT_V;
    _i2c.write((int)ina226_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina226_set_data.addr, (char *)dt, 2, false);
    int16_t data = (dt[0] << 8) | dt[1];
    return (float)data / 10;
//    return ((float)data / ina226_set_data.shunt_register) / 1000;
}

int16_t INA226::read_shunt_raw_voltage()
{
    dt[0] = INA226_SHUNT_V;
    _i2c.write((int)ina226_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina226_set_data.addr, (char *)dt, 2, false);
    int16_t data = (dt[0] << 8) | dt[1];
    return data;
}

/////////////// Read configulation ////////////////////////
uint16_t INA226::read_config()
{
    dt[0] = INA226_CONFIG;
    _i2c.write((int)ina226_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina226_set_data.addr, (char *)dt, 2, false);
    uint16_t data = (dt[0] << 8) | dt[1];
    return data;
}

/////////////// Set configulation /////////////////////////
uint16_t INA226::set_config(uint16_t cfg)
{
    uint16_t data = cfg;
    dt[0] = INA226_CONFIG;
    dt[1] = data >> 8;    // MSB 1st
    dt[2] = data & 0xff;  // LSB 2nd
    _i2c.write((int)ina226_set_data.addr, (char *)dt, 3, false);
    return data;
}

/////////////// Read Calibration reg. /////////////////////
uint16_t INA226::read_calb(void)
{
    dt[0] = INA226_CALBLATION;
    _i2c.write((int)ina226_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina226_set_data.addr, (char *)dt, 2, false);
    uint16_t data = (dt[0] << 8) | dt[1];
    return data;
}

/////////////// Set Calibration reg. //////////////////////
uint16_t INA226::set_calb(uint16_t clb)
{
    uint16_t data = clb;
    dt[0] = INA226_CALBLATION;
    dt[1] = data >> 8;    // MSB 1st
    dt[2] = data & 0xff;  // LSB 2nd
    _i2c.write((int)ina226_set_data.addr, (char *)dt, 3, false);
    return data;
}

/////////////// Read Mask/Enable reg. /////////////////////
uint16_t INA226::read_msk_enbl(void)
{
    dt[0] = INA226_MASK_EN;
    _i2c.write((int)ina226_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina226_set_data.addr, (char *)dt, 2, false);
    uint16_t data = (dt[0] << 8) | dt[1];
    return data;
}

/////////////// Set Mask/Enable reg.  /////////////////////
uint16_t INA226::set_msk_enbl(uint16_t msk_enable)
{
    uint16_t data = msk_enable;
    dt[0] = INA226_MASK_EN;
    dt[1] = data >> 8;    // MSB 1st
    dt[2] = data & 0xff;  // LSB 2nd
    _i2c.write((int)ina226_set_data.addr, (char *)dt, 3, false);
    return data;
}

uint8_t INA226::read_ID()
{
    dt[0] = INA226_DIE_ID;
    _i2c.write((int)ina226_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina226_set_data.addr, (char *)dt, 1, false);
    id_number = dt[0];
    return id_number;
}

/////////////// Read/Write specific register //////////////
uint8_t INA226::read_reg(uint8_t addr)
{
    dt[0] = addr;
    _i2c.write((int)ina226_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina226_set_data.addr, (char *)dt, 1, false);
    return dt[0];
}

uint8_t INA226::write_reg(uint8_t addr, uint8_t data)
{
    dt[0] = addr;
    dt[1] = data;
    _i2c.write((int)ina226_set_data.addr, (char *)dt, 2, false);
    return dt[1];
}

/////////////// Initialize ////////////////////////////////
void INA226::initialize()
{
    uint16_t data = 0;
    data  = (ina226_set_data.average & 0x07) << 9;
    data |= (ina226_set_data.b_volt_cnv_time & 0x07) << 6;
    data |= (ina226_set_data.s_volt_cnv_time & 0x07) << 3;
    data |= (ina226_set_data.mode & 0x07);
    dt[0] = INA226_CONFIG;
    dt[1] = data >> 8;    // MSB 1st
    dt[2] = data & 0xff;  // LSB 2nd
    _i2c.write((int)ina226_set_data.addr, (char *)dt, 3, false);
    dt[0] = INA226_CALBLATION;
    dt[1] = ina226_set_data.calibration_data >> 8;    // MSB 1st
    dt[2] = ina226_set_data.calibration_data & 0xff;  // LSB 2nd
    _i2c.write((int)ina226_set_data.addr, (char *)dt, 3, false);
    scale_factor = 0;
    read_ID();
}

/////////////// I2C Freq. /////////////////////////////////
void INA226::frequency(int hz)
{
    _i2c.frequency(hz);
}

