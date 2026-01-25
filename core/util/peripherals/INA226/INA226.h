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
/*
 *---------------- REFERENCE ----------------------------------------------------------------------
 * Original Information
 *  http://www.ti.com/product/INA226/description
 *  http://www.ti.com/lit/ds/symlink/ina226.pdf
 * Device kit
 *  http://strawberry-linux.com/catalog/items?code=12031
 *  http://strawberry-linux.com/catalog/items?code=12226
 */

#ifndef        MBED_INA226
#define        MBED_INA226

/////////// ADDRESS /////////////////////////////
//  7bit address = 0b1000000(0x40)
//  G=GND, V=VS+, A=SDA, L=SCL
//  e.g. _VG: A1=VS+, A0=GND
//    -> Please make sure your H/W configuration
// Set data into "addr"
#define INA226_ADDR_GG             (0x40 << 1)
#define INA226_ADDR_GV             (0x41 << 1)
#define INA226_ADDR_GA             (0x42 << 1)
#define INA226_ADDR_GL             (0x43 << 1)
#define INA226_ADDR_VG             (0x44 << 1)
#define INA226_ADDR_VV             (0x45 << 1)
#define INA226_ADDR_VA             (0x46 << 1)
#define INA226_ADDR_VL             (0x47 << 1)
#define INA226_ADDR_AG             (0x48 << 1)
#define INA226_ADDR_AV             (0x49 << 1)
#define INA226_ADDR_AA             (0x4a << 1)
#define INA226_ADDR_AL             (0x4b << 1)
#define INA226_ADDR_LG             (0x4c << 1)
#define INA226_ADDR_LV             (0x4d << 1)
#define INA226_ADDR_LA             (0x4e << 1)
#define INA226_ADDR_LL             (0x4f << 1)

/////////// REGISTER DEFINITION /////////////////
#define INA226_CONFIG              0x00
#define INA226_SHUNT_V             0x01
#define INA226_BUS_VOLT            0x02
#define INA226_POWER               0x03
#define INA226_CURRENT             0x04
#define INA226_CALBLATION          0x05
#define INA226_MASK_EN             0x06
#define INA226_ALERT_LMT           0x07
#define INA226_DIE_ID              0xff

/////////// PARAMETER SETTING for Configuration Reg. //////////////////////////
// Set data into "shunt_register"
#define INA226_PAR_R_2MORM         2    // Strawberry Linux borad
#define INA226_PAR_R_25MOHM        25
#define INA226_PAR_R_MORM(x)       (x)
// Set data into "average", Averaging Mode
#define INA226_PAR_A_1             0    // Default
#define INA226_PAR_A_4             1
#define INA226_PAR_A_16            2
#define INA226_PAR_A_64            3
#define INA226_PAR_A_128           4
#define INA226_PAR_A_256           5
#define INA226_PAR_A_512           6
#define INA226_PAR_A_1024          7
// Set data into "b_volt_cnv_time" Bus Voltage Conversion Time
#define INA226_CFG_BUS_V_TR140     0
#define INA226_CFG_BUS_V_TR204     1
#define INA226_CFG_BUS_V_TR332     2
#define INA226_CFG_BUS_V_TR588     3
#define INA226_CFG_BUS_V_T1R1      4    // Default
#define INA226_CFG_BUS_V_T2R116    5
#define INA226_CFG_BUS_V_T4R156    6
#define INA226_CFG_BUS_V_T8R244    7
// Set data into "s_volt_cnv_time", Shunt Voltage Conversion Time
#define INA226_CFG_SHT_V_TR140     0
#define INA226_CFG_SHT_V_TR204     1
#define INA226_CFG_SHT_V_TR332     2
#define INA226_CFG_SHT_V_TR588     3
#define INA226_CFG_SHT_V_T1R1      4    // Default
#define INA226_CFG_SHT_V_T2R116    5
#define INA226_CFG_SHT_V_T4R156    6
#define INA226_CFG_SHT_V_T8R244    7
// Set data into "mode", Operating Mode
#define INA226_PAR_M_PDWN          0
#define INA226_PAR_M_SHNT_TRG      1
#define INA226_PAR_M_BUS_TRG       2
#define INA226_PAR_M_SHNTBUS_TRG   3
#define INA226_PAR_M_ADC_OFF       4
#define INA226_PAR_M_SHNT_CONT     5
#define INA226_PAR_M_BUS_CONT      6
#define INA226_PAR_M_SHNTBUS_CONT  7    // Default

////////////// DATA TYPE DEFINITION ///////////////////////
typedef struct {
    uint8_t addr;
    uint16_t shunt_register;
    uint8_t average;
    uint8_t b_volt_cnv_time;
    uint8_t s_volt_cnv_time;
    uint8_t mode;
    uint16_t calibration_data;
} INA226_TypeDef;

////////////// DEFAULT SETTING ////////////////////////////
// Standard parameter for easy set-up
const INA226_TypeDef ina226_std_paramtr = {
    // I2C Address
    INA226_ADDR_GG,
    // CONFIG REG
    INA226_PAR_R_2MORM, // 2 milli-ohm
    INA226_PAR_A_1,     // Averaging Mode
    INA226_CFG_BUS_V_T1R1,
    INA226_CFG_SHT_V_T1R1,
    INA226_PAR_M_SHNTBUS_CONT,
    // CALBLATION REG
    2560
};

/** INA226 High-or Low-Side Measurement,Bi-Directional CURRENT/POWER MONITOR with I2C Interface
 *
 * @code
 * //--------- Default setting -----------------
 * #include "mbed.h"
 * #include "INA226.h"
 *
 * // I2C Communication
 * INA226 current(dp5, dp27, INA226_ADDR_GG);
 * // If you connected I2C line not only this device but also other devices,
 * //     you need to declare following method.
 * I2C    i2c(dp5, dp27);
 * INA226 current(I2C& p_i2c, INA226_ADDR_GG);
 *
 * int main() {
 *     while(1){
 *         printf("I=%+6.3f [A]\r\n", current.read_current());
 *         wait(1.0):
 *     }
 * }
 * //--------- Detail setting -----------------
 * #include "mbed.h"
 * #include "INA226.h"
 *
 * const INA226_TypeDef ina226_my_paramtr = {
 *    // I2C Address
 *    INA226_ADDR_GG,
 *    // CONFIG REG
 *    INA226_PAR_R_100MOHM,     // 100 milli-ohm
 *    INA226_PAR_A_1,           // Averaging Mode
 *    INA226_CFG_BUS_V_T1R1,    // Bus Voltage Conversion Time
 *    INA226_CFG_SHT_V_T1R1,    // Shunt Voltage Conversion Time
 *    INA226_PAR_M_SHNTBUS_CONT,// Operating Mode
 *    // CALBLATION REG
 *    2560
 * };
 *
 * I2C    i2c(dp5,dp27);
 * INA226 current(I2C& p_i2c, &ina226_my_paramtr);
 *
 * int main() {
 *     while(1){
 *         printf("I=%+6.3f [A]\r\n", current.read_current());
 *         wait(1.0):
 *     }
 * }
 *  @endcode
 */

class INA226
{
public:
    /** Configure data pin
      * @param data SDA and SCL pins
      * @param parameter address chip (INA226_TypeDef)
      * @param or just set address or just port
      */
    INA226(PinName p_sda, PinName p_scl, const INA226_TypeDef *ina226_parameter);
    INA226(PinName p_sda, PinName p_scl, uint8_t addr);
    INA226(PinName p_sda, PinName p_scl);

    /** Configure data pin (with other devices on I2C line)
      * @param I2C previous definition
      * @param parameter address chip (INA226_TypeDef)
      * @param or just set address or just port
      */
    INA226(I2C& p_i2c, const INA226_TypeDef *ina226_parameter);
    INA226(I2C& p_i2c, uint8_t addr);
    INA226(I2C& p_i2c);

    /** Read Current data
      * @param none
      * @return current [mA]
      */
    float read_current(void);
    float read_current_by_shuntvolt(void);

    /** Read Power data
      * @param none
      * @return power [w]
      */
    float read_power(void);

    /** Read Bus voltage
      * @param none
      * @return voltage [v]
      */
    float read_bus_voltage(void);

    /** Read Shunt voltage data
      * @param none
      * @return voltage [v]
      */
    float read_shunt_voltage(void);

    /** Read Shunt voltage data
      * @param none
      * @return voltage related value
      */
    int16_t read_shunt_raw_voltage(void);

    /** Read configration reg.
      * @param none
      * @return configrartion register value
      */
    uint16_t read_config(void);

    /** Set configration reg.
      * @param configuration data
      * @return configrartion register value
      */
    uint16_t set_config(uint16_t cfg);

    /** Read calibration reg.
      * @param none
      * @return calibration register value
      */
    uint16_t read_calb(void);

    /** Set calibration reg.
      * @param calibration data
      * @return calibration register value
      */
    uint16_t set_calb(uint16_t clb);

    /** Read Mask/Enable reg.
      * @param none
      * @return calibration register value
      */
    uint16_t read_msk_enbl(void);

    /** Set Mask/Enable reg.
      * @param mask enable data
      * @return calibration register value
      */
    uint16_t set_msk_enbl(uint16_t clb);

    /** Read ID
      * @param none
      * @return ID
      */
    uint8_t read_ID();

    /** Set I2C clock frequency
      * @param freq.
      * @return none
      */
    void frequency(int hz);

    /** Read register (general purpose)
      * @param register's address
      * @return register data
      */
    uint8_t read_reg(uint8_t addr);

    /** Write register (general purpose)
      * @param register's address
      * @param data
      * @return register data
      */
    uint8_t write_reg(uint8_t addr, uint8_t data);

protected:
    I2C *_i2c_p;
    I2C &_i2c;

    void initialize(void);

private:
    uint8_t id_number;
    INA226_TypeDef ina226_set_data;
    int32_t scale_factor;
    uint8_t dt[4];
};

#endif  //  MBED_INA226

