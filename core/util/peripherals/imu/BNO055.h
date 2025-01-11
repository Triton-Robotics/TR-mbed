///*
// * mbed library program
// *  BNO055 Intelligent 9-axis absolute orientation sensor
// *  by Bosch Sensortec
// *
// * Copyright (c) 2015,'17 Kenji Arai / JH1PJL
// *  http://www.page.sannet.ne.jp/kenjia/index.html
// *  http://mbed.org/users/kenjiArai/
// *      Created: March     30th, 2015
// *      Revised: August    23rd, 2017
// */
///*
// *---------------- REFERENCE ----------------------------------------------------------------------
// * Original Information
// *  https://www.bosch-sensortec.com/en/homepage/products_3/sensor_hubs/iot_solutions/bno055_1/bno055_4
// *  Intelligent 9-axis absolute orientation sensor / Data Sheet  BST_BNO055_DS000_12 Nov. 2014 rev.1.2
// *  Sample software   https://github.com/BoschSensortec/BNO055_driver
// * Sensor board
// *  https://www.rutronik24.com/product/bosch+se/bno055+shuttle+board+mems/6431291.html
// *  http://microcontrollershop.com/product_info.php?products_id=7140&osCsid=10645k86db2crld4tfi0vol5g5
// */
//
///*
// * This library has been modified. Original can be found here: https://os.mbed.com/users/AlexanderLill/code/BNO055_fusion/
// */
//
//#ifndef BNO055_H
//#define BNO055_H
//
//#include "mbed.h"
//
//#define PI 3.14159265
//
////  BNO055
////  7bit address = 0b010100x(0x28 or 0x29 depends on COM3)
//#define BNO055_G_CHIP_ADDR      (0x28 << 1) // COM3 = GND
//#define BNO055_V_CHIP_ADDR      (0x29 << 1) // COM3 = Vdd
//
//// Fusion mode
//#define CONFIGMODE              0x00
//#define MODE_IMU                0x08
//#define MODE_COMPASS            0x09
//#define MODE_M4G                0x0a
//#define MODE_NDOF_FMC_OFF       0x0b
//#define MODE_NDOF               0x0c
//
////  UNIT
//#define UNIT_ACC_MSS            0x00    // acc m/s2
//#define UNIT_ACC_MG             0x01    // acc mg
//#define UNIT_GYR_DPS            0x00    // gyro Dps
//#define UNIT_GYR_RPS            0x02    // gyro Rps
//#define UNIT_EULER_DEG          0x00    // euler Degrees
//#define UNIT_EULER_RAD          0x04    // euler Radians
//#define UNIT_TEMP_C             0x00    // temperature degC
//#define UNIT_TEMP_F             0x10    // temperature degF
//#define UNIT_ORI_WIN            0x00    // Windows orientation
//#define UNIT_ORI_ANDROID        0x80    // Android orientation
//
////  ID's
//#define I_AM_BNO055_CHIP        0xa0    // CHIP ID
//#define I_AM_BNO055_ACC         0xfb    // ACC ID
//#define I_AM_BNO055_MAG         0x32    // MAG ID
//#define I_AM_BNO055_GYR         0x0f    // GYR ID
//
//////////////// DATA TYPE DEFINITION ///////////////////////
//typedef struct {
//    uint8_t  chip_id;
//    uint8_t  acc_id;
//    uint8_t  mag_id;
//    uint8_t  gyr_id;
//    uint8_t  bootldr_rev_id;
//    uint16_t sw_rev_id;
//} BNO055_ID_INF_TypeDef;
//
//typedef struct {
//    double h;
//    double r;
//    double p;
//} BNO055_EULER_TypeDef;
//
//typedef struct {
//    double x;
//    double y;
//    double z;
//    double w;
//} BNO055_QUATERNION_TypeDef;
//
//typedef struct{
//    double yaw;
//    double roll;
//    double pitch;
//} BNO055_ANGULAR_POSITION_typedef;
//
//typedef struct {
//    double x;
//    double y;
//    double z;
//} BNO055_VECTOR_TypeDef;
//
//typedef struct {
//    int8_t acc_chip;
//    int8_t gyr_chip;
//} BNO055_TEMPERATURE_TypeDef;
//
//enum {MT_P0 = 0, MT_P1, MT_P2, MT_P3, MT_P4, MT_P5, MT_P6, MT_P7};
//
///** Interface for Bosch Sensortec Intelligent 9-axis absolute orientation sensor
// *      Chip: BNO055
// *
// * @code
// * #include    "mbed.h"
// * #include    "BNO055.h"
// *
// * Serial pc(USBTX,USBRX);
// * I2C    i2c(PB_9, PB_8);         // SDA, SCL
// * BNO055 imu(i2c, PA_8);          // Reset
// *
// * BNO055_ID_INF_TypeDef bno055_id_inf;
// * BNO055_EULER_TypeDef  euler_angles;
// *
// * int main() {
// *     pc.printf("Bosch Sensortec BNO055 test program on " __DATE__ "/" __TIME__ "\r\n");
// *     if (imu.chip_ready() == 0){
// *         pc.printf("Bosch BNO055 is NOT avirable!!\r\n");
// *     }
// *     imu.read_id_inf(&bno055_id_inf);
// *     pc.printf("CHIP:0x%02x, ACC:0x%02x, MAG:0x%02x, GYR:0x%02x, , SW:0x%04x, , BL:0x%02x\r\n",
// *                bno055_id_inf.chip_id, bno055_id_inf.acc_id, bno055_id_inf.mag_id,
// *                bno055_id_inf.gyr_id, bno055_id_inf.sw_rev_id, bno055_id_inf.bootldr_rev_id);
// *     while(1) {
// *         imu.get_Euler_Angles(&euler_angles);
// *         pc.printf("Heading:%+6.1f [deg], Roll:%+6.1f [deg], Pich:%+6.1f [deg]\r\n",
// *                    euler_angles.h, euler_angles.r, euler_angles.p);
// *         wait(0.5);
// *     }
// * }
// * @endcode
// */
//
//class BNO055
//{
//public:
//    /** Configure data pin
//      * @param data SDA and SCL pins
//      * @param device address
//      */
//    BNO055(PinName p_sda, PinName p_scl, PinName p_reset, uint8_t addr, uint8_t mode);
//
//    /** Configure data pin
//      * @param data SDA and SCL pins
//      * @param Other parameters are set default data
//      */
//    BNO055(PinName p_sda, PinName p_scl, PinName p_reset);
//
//    /** Configure data pin (with other devices on I2C line)
//      * @param I2C previous definition
//      * @param device address
//      */
//    BNO055(I2C& p_i2c, PinName p_reset, uint8_t addr, uint8_t mode);
//
//    /** Configure data pin (with other devices on I2C line)
//      * @param I2C previous definition
//      * @param Other parameters are set default data
//      */
//    BNO055(I2C& p_i2c, PinName p_reset, uint8_t mode);
//
//    /** Get Euler Angles
//     * @param double type of 3D data address
//     */
//    void get_euler_angles(BNO055_EULER_TypeDef *el);
//
//    /** Get Quaternion XYZ&W
//     * @param int16_t type of 4D data address
//     */
//    void get_quaternion(BNO055_QUATERNION_TypeDef *qua);
//
//    /** Get Angular position from quaternion
//     *  @param double type of 3D data address
//     */
//    void get_angular_position_quat(BNO055_ANGULAR_POSITION_typedef *an_pos);
//
//    /** Get Linear accel data
//     * @param double type of 3D data address
//     */
//    void get_linear_accel(BNO055_VECTOR_TypeDef *la);
//
//    /** Get Mag data
//     * @param double type of 3D data address
//     */
//    void get_mag(BNO055_VECTOR_TypeDef *qua);
//
//    /** Get Accel data
//     * @param double type of 3D data address
//     */
//    void get_accel(BNO055_VECTOR_TypeDef *la);
//
//    /** Get Gyro data
//     * @param double type of 3D data address
//     */
//    void get_gyro(BNO055_VECTOR_TypeDef *gr);
//
//    /** Get Gravity data
//     * @param double type of 3D data address
//     */
//    void get_gravity(BNO055_VECTOR_TypeDef *gr);
//
//    /** Get Chip temperature data both Acc & Gyro
//     * @param int8_t type of data address
//     */
//    void get_chip_temperature(BNO055_TEMPERATURE_TypeDef *tmp);
//
//    /** Change fusion mode
//      * @param fusion mode
//      * @return none
//      */
//    void change_fusion_mode(uint8_t mode);
//
//    /** Set Mouting position
//      *  Please make sure your mounting direction of BNO055 chip
//      *  refrence: BNO055 data sheet BST-BNO055-DS000-12 3.4 Axis remap
//      * @param Set P0 to P7 mounting position data
//      * @return none
//      */
//    void set_mounting_position(uint8_t position);
//
//    /** Read BNO055 ID information
//      * @param ID information address
//      * @return none
//      */
//    void read_id_inf(BNO055_ID_INF_TypeDef *id);
//
//    /** Check chip is avairable or not
//      * @param none
//      * @return OK = 1, NG = 0;
//      */
//    uint8_t chip_ready(void);
//
//    /** Calibrate IMU
//      * @param none
//      * @return none
//      */
//    void calibrate(void);
//
//    /** Read calibration status
//      * @param none
//      * @return SYS(7:6),GYR(5:4),ACC(3:2),MAG(1:0) 3 = Calibrated, 0= not yet
//      */
//    uint8_t read_calib_status(void);
//
//    /** Reset
//      * @param none
//      * @return 0 = sucess, 1 = Not available chip
//      */
//    uint8_t reset(void);
//
//    /** Set I2C clock frequency
//      * @param freq.
//      * @return none
//      */
//    void frequency(int hz);
//
//    /** Read page 0 register
//      * @param register's address
//      * @return register data
//      */
//    uint8_t read_reg0(uint8_t addr);
//
//    /** Write page 0 register
//      * @param register's address
//      * @param data
//      * @return register data
//      */
//    uint8_t write_reg0(uint8_t addr, uint8_t data);
//
//    /** Read page 1 register
//      * @param register's address
//      * @return register data
//      */
//    uint8_t read_reg1(uint8_t addr);
//
//    /** Write page 1 register
//      * @param register's address
//      * @param data
//      * @return register data
//      */
//    uint8_t write_reg1(uint8_t addr, uint8_t data);
//
//    /** Determine if degrees (instead of radians) are used.
//      * @return true if degrees are used
//      */
//    bool use_degrees();
//
//    /** Determine if m/s*s (instead of mg) is used.
//      * @return true if m/s*s are used
//      */
//    bool use_mss();
//
//    /** Determine if dps (instead of rps) are used.
//      * @return true if dps are used
//      */
//    bool use_dps();
//
//    /** Determine if celsius (instead of fahrenheit) is used.
//      * @return true if celsius are used
//      */
//    bool use_celsius();
//
//    int cantReadDataCount;
//
//    double multiturnYaw;
//
//protected:
//    void initialize(void);
//    void initialize_reset_pin(void);
//    void get_id(void);
//    void set_initial_dt_to_regs(void);
//    void unit_selection(void);
//    uint8_t get_operating_mode(void);
//    uint8_t select_page(uint8_t page);
//
//    I2C *_i2c_p;
//    I2C &_i2c;
//    DigitalOut _res;
//
//private:
//    char     dt[10];      // working buffer
//    uint8_t  chip_addr;
//    uint8_t  chip_mode;
//    uint8_t  ready_flag;
//    uint8_t  page_flag;
//
//    uint8_t  chip_id;
//    uint8_t  acc_id;
//    uint8_t  mag_id;
//    uint8_t  gyr_id;
//    uint8_t  bootldr_rev_id;
//    uint16_t sw_rev_id;
//
//    bool unit_flag_is_set(uint8_t flag);
//};
//
////---------------------------------------------------------
////----- Register's definition -----------------------------
////---------------------------------------------------------
//// Page id register definition
//#define BNO055_PAGE_ID          0x07
//
////----- page0 ---------------------------------------------
//#define BNO055_CHIP_ID          0x00
//#define BNO055_ACCEL_REV_ID     0x01
//#define BNO055_MAG_REV_ID       0x02
//#define BNO055_GYRO_REV_ID      0x03
//#define BNO055_SW_REV_ID_LSB    0x04
//#define BNO055_SW_REV_ID_MSB    0x05
//#define BNO055_BL_REV_ID        0x06
//
//// Accel data register*/
//#define BNO055_ACC_X_LSB        0x08
//#define BNO055_ACC_X_MSB        0x09
//#define BNO055_ACC_Y_LSB        0x0a
//#define BNO055_ACC_Y_MSB        0x0b
//#define BNO055_ACC_Z_LSB        0x0c
//#define BNO055_ACC_Z_MSB        0x0d
//
//// Mag data register
//#define BNO055_MAG_X_LSB        0x0e
//#define BNO055_MAG_X_MSB        0x0f
//#define BNO055_MAG_Y_LSB        0x10
//#define BNO055_MAG_Y_MSB        0x11
//#define BNO055_MAG_Z_LSB        0x12
//#define BNO055_MAG_Z_MSB        0x13
//
//// Gyro data registers
//#define BNO055_GYR_X_LSB        0x14
//#define BNO055_GYR_X_MSB        0x15
//#define BNO055_GYR_Y_LSB        0x16
//#define BNO055_GYR_Y_MSB        0x17
//#define BNO055_GYR_Z_LSB        0x18
//#define BNO055_GYR_Z_MSB        0x19
//
//// Euler data registers
//#define BNO055_EULER_H_LSB      0x1a
//#define BNO055_EULER_H_MSB      0x1b
//
//#define BNO055_EULER_R_LSB      0x1c
//#define BNO055_EULER_R_MSB      0x1d
//
//#define BNO055_EULER_P_LSB      0x1e
//#define BNO055_EULER_P_MSB      0x1f
//
//// Quaternion data registers
//#define BNO055_QUATERNION_W_LSB 0x20
//#define BNO055_QUATERNION_W_MSB 0x21
//#define BNO055_QUATERNION_X_LSB 0x22
//#define BNO055_QUATERNION_X_MSB 0x23
//#define BNO055_QUATERNION_Y_LSB 0x24
//#define BNO055_QUATERNION_Y_MSB 0x25
//#define BNO055_QUATERNION_Z_LSB 0x26
//#define BNO055_QUATERNION_Z_MSB 0x27
//
//// Linear acceleration data registers
//#define BNO055_LINEAR_ACC_X_LSB 0x28
//#define BNO055_LINEAR_ACC_X_MSB 0x29
//#define BNO055_LINEAR_ACC_Y_LSB 0x2a
//#define BNO055_LINEAR_ACC_Y_MSB 0x2b
//#define BNO055_LINEAR_ACC_Z_LSB 0x2c
//#define BNO055_LINEAR_ACC_Z_MSB 0x2d
//
//// Gravity data registers
//#define BNO055_GRAVITY_X_LSB    0x2e
//#define BNO055_GRAVITY_X_MSB    0x2f
//#define BNO055_GRAVITY_Y_LSB    0x30
//#define BNO055_GRAVITY_Y_MSB    0x31
//#define BNO055_GRAVITY_Z_LSB    0x32
//#define BNO055_GRAVITY_Z_MSB    0x33
//
//// Temperature data register
//#define BNO055_TEMP             0x34
//
//// Status registers
//#define BNO055_CALIB_STAT       0x35
//#define BNO055_SELFTEST_RESULT  0x36
//#define BNO055_INTR_STAT        0x37
//#define BNO055_SYS_CLK_STAT     0x38
//#define BNO055_SYS_STAT         0x39
//#define BNO055_SYS_ERR          0x3a
//
//// Unit selection register
//#define BNO055_UNIT_SEL         0x3b
//#define BNO055_DATA_SELECT      0x3c
//
//// Mode registers
//#define BNO055_OPR_MODE         0x3d
//#define BNO055_PWR_MODE         0x3e
//#define BNO055_SYS_TRIGGER      0x3f
//#define BNO055_TEMP_SOURCE      0x40
//
//// Axis remap registers
//#define BNO055_AXIS_MAP_CONFIG  0x41
//#define BNO055_AXIS_MAP_SIGN    0x42
//
//// SIC registers
//#define BNO055_SIC_MTRX_0_LSB   0x43
//#define BNO055_SIC_MTRX_0_MSB   0x44
//#define BNO055_SIC_MTRX_1_LSB   0x45
//#define BNO055_SIC_MTRX_1_MSB   0x46
//#define BNO055_SIC_MTRX_2_LSB   0x47
//#define BNO055_SIC_MTRX_2_MSB   0x48
//#define BNO055_SIC_MTRX_3_LSB   0x49
//#define BNO055_SIC_MTRX_3_MSB   0x4a
//#define BNO055_SIC_MTRX_4_LSB   0x4b
//#define BNO055_SIC_MTRX_4_MSB   0x4c
//#define BNO055_SIC_MTRX_5_LSB   0x4d
//#define BNO055_SIC_MTRX_5_MSB   0x4e
//#define BNO055_SIC_MTRX_6_LSB   0x4f
//#define BNO055_SIC_MTRX_6_MSB   0x50
//#define BNO055_SIC_MTRX_7_LSB   0x51
//#define BNO055_SIC_MTRX_7_MSB   0x52
//#define BNO055_SIC_MTRX_8_LSB   0x53
//#define BNO055_SIC_MTRX_8_MSB   0x54
//
//// Accelerometer Offset registers
//#define ACCEL_OFFSET_X_LSB      0x55
//#define ACCEL_OFFSET_X_MSB      0x56
//#define ACCEL_OFFSET_Y_LSB      0x57
//#define ACCEL_OFFSET_Y_MSB      0x58
//#define ACCEL_OFFSET_Z_LSB      0x59
//#define ACCEL_OFFSET_Z_MSB      0x5a
//
//// Magnetometer Offset registers
//#define MAG_OFFSET_X_LSB        0x5b
//#define MAG_OFFSET_X_MSB        0x5c
//#define MAG_OFFSET_Y_LSB        0x5d
//#define MAG_OFFSET_Y_MSB        0x5e
//#define MAG_OFFSET_Z_LSB        0x5f
//#define MAG_OFFSET_Z_MSB        0x60
//
//// Gyroscope Offset registers
//#define GYRO_OFFSET_X_LSB       0x61
//#define GYRO_OFFSET_X_MSB       0x62
//#define GYRO_OFFSET_Y_LSB       0x63
//#define GYRO_OFFSET_Y_MSB       0x64
//#define GYRO_OFFSET_Z_LSB       0x65
//#define GYRO_OFFSET_Z_MSB       0x66
//
//// Radius registers
//#define ACCEL_RADIUS_LSB        0x67
//#define ACCEL_RADIUS_MSB        0x68
//#define MAG_RADIUS_LSB          0x69
//#define MAG_RADIUS_MSB          0x6a
//
////----- page1 ---------------------------------------------
//// Configuration registers
//#define ACCEL_CONFIG            0x08
//#define MAG_CONFIG              0x09
//#define GYRO_CONFIG             0x0a
//#define GYRO_MODE_CONFIG        0x0b
//#define ACCEL_SLEEP_CONFIG      0x0c
//#define GYRO_SLEEP_CONFIG       0x0d
//#define MAG_SLEEP_CONFIG        0x0e
//
//// Interrupt registers
//#define INT_MASK                0x0f
//#define INT                     0x10
//#define ACCEL_ANY_MOTION_THRES  0x11
//#define ACCEL_INTR_SETTINGS     0x12
//#define ACCEL_HIGH_G_DURN       0x13
//#define ACCEL_HIGH_G_THRES      0x14
//#define ACCEL_NO_MOTION_THRES   0x15
//#define ACCEL_NO_MOTION_SET     0x16
//#define GYRO_INTR_SETING        0x17
//#define GYRO_HIGHRATE_X_SET     0x18
//#define GYRO_DURN_X             0x19
//#define GYRO_HIGHRATE_Y_SET     0x1a
//#define GYRO_DURN_Y             0x1b
//#define GYRO_HIGHRATE_Z_SET     0x1c
//#define GYRO_DURN_Z             0x1d
//#define GYRO_ANY_MOTION_THRES   0x1e
//#define GYRO_ANY_MOTION_SET     0x1f
//
//#endif      // BNO055_H
//






/***************************************************************************
  This is a library for the BNO055 orientation sensor

  Designed specifically to work with the Adafruit BNO055 Breakout.

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by KTOWN for Adafruit Industries.

  MIT license, all text above must be included in any redistribution
 ***************************************************************************/
/*
* Copyright (C) 2008 The Android Open Source Project
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software< /span>
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

/* Update by K. Townsend (Adafruit Industries) for lighter typedefs, and
 * extended sensor support to include color, voltage and current */
































#ifndef _ADAFRUIT_SENSOR_H
#define _ADAFRUIT_SENSOR_H


/* Intentionally modeled after sensors.h in the Android API:
 * https://github.com/android/platform_hardware_libhardware/blob/master/include/hardware/sensors.h */

/* Constants */
#define SENSORS_GRAVITY_EARTH             (9.80665F)              /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_MOON              (1.6F)                  /**< The moon's gravity in m/s^2 */
#define SENSORS_GRAVITY_SUN               (275.0F)                /**< The sun's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD          (SENSORS_GRAVITY_EARTH)
#define SENSORS_MAGFIELD_EARTH_MAX        (60.0F)                 /**< Maximum magnetic field on Earth's surface */
#define SENSORS_MAGFIELD_EARTH_MIN        (30.0F)                 /**< Minimum magnetic field on Earth's surface */
#define SENSORS_PRESSURE_SEALEVELHPA      (1013.25F)              /**< Average sea level pressure is 1013.25 hPa */
#define SENSORS_DPS_TO_RADS               (0.017453293F)          /**< Degrees/s to rad/s multiplier */
#define SENSORS_GAUSS_TO_MICROTESLA       (100)                   /**< Gauss to micro-Tesla multiplier */

/** Sensor types */
typedef enum
{
SENSOR_TYPE_ACCELEROMETER         = (1),   /**< Gravity + linear acceleration */
SENSOR_TYPE_MAGNETIC_FIELD        = (2),
SENSOR_TYPE_ORIENTATION           = (3),
SENSOR_TYPE_GYROSCOPE             = (4),
SENSOR_TYPE_LIGHT                 = (5),
SENSOR_TYPE_PRESSURE              = (6),
SENSOR_TYPE_PROXIMITY             = (8),
SENSOR_TYPE_GRAVITY               = (9),
SENSOR_TYPE_LINEAR_ACCELERATION   = (10),  /**< Acceleration not including gravity */
SENSOR_TYPE_ROTATION_VECTOR       = (11),
SENSOR_TYPE_RELATIVE_HUMIDITY     = (12),
SENSOR_TYPE_AMBIENT_TEMPERATURE   = (13),
SENSOR_TYPE_VOLTAGE               = (15),
SENSOR_TYPE_CURRENT               = (16),
SENSOR_TYPE_COLOR                 = (17)
} sensors_type_t;

/** struct sensors_vec_s is used to return a vector in a common format. */
typedef struct {
union {
float v[3];
struct {
float x;
float y;
float z;
};
/* Orientation sensors */
struct {
float roll;    /**< Rotation around the longitudinal axis (the plane body, 'X axis'). Roll is positive and increasing when moving downward. -90°<=roll<=90° */
float pitch;   /**< Rotation around the lateral axis (the wing span, 'Y axis'). Pitch is positive and increasing when moving upwards. -180°<=pitch<=180°) */
float heading; /**< Angle between the longitudinal axis (the plane body) and magnetic north, measured clockwise when viewing from the top of the device. 0-359° */
};
};
char status;
unsigned char reserved[3];
} sensors_vec_t;
typedef struct{
    double yaw;
    double roll;
    double pitch;
} BNO055_ANGULAR_POSITION_typedef;

/** struct sensors_color_s is used to return color data in a common format. */
typedef struct {
union {
float c[3];
/* RGB color space */
struct {
float r;       /**< Red component */
float g;       /**< Green component */
float b;       /**< Blue component */
};
};
unsigned int rgba;         /**< 24-bit RGBA value */
} sensors_color_t;

/* Sensor event (36 bytes) */
/** struct sensor_event_s is used to provide a single sensor event in a common format. */
typedef struct
{
int version;                          /**< must be sizeof(struct sensors_event_t) */
int sensor_id;                        /**< unique sensor identifier */
int type;                             /**< sensor type */
int reserved0;                        /**< reserved */
int timestamp;                        /**< time is in milliseconds */
union
{
float           data[4];
sensors_vec_t   acceleration;         /**< acceleration values are in meter per second per second (m/s^2) */
sensors_vec_t   magnetic;             /**< magnetic vector values are in micro-Tesla (uT) */
sensors_vec_t   orientation;          /**< orientation values are in degrees */
sensors_vec_t   gyro;                 /**< gyroscope values are in rad/s */
float           temperature;          /**< temperature is in degrees centigrade (Celsius) */
float           distance;             /**< distance in centimeters */
float           light;                /**< light in SI lux units */
float           pressure;             /**< pressure in hectopascal (hPa) */
float           relative_humidity;    /**< relative humidity in percent */
float           current;              /**< current in milliamps (mA) */
float           voltage;              /**< voltage in volts (V) */
sensors_color_t color;                /**< color in RGB component values */
};
} sensors_event_t;
typedef struct {
    double x;
    double y;
    double z;
    double w;
} BNO055_QUATERNION_TypeDef;

/* Sensor details (40 bytes) */
/** struct sensor_s is used to describe basic information about a specific sensor. */
typedef struct
{
char     name[12];                        /**< sensor name */
int  version;                         /**< version of the hardware + driver */
int  sensor_id;                       /**< unique sensor identifier */
int  type;                            /**< this sensor's type (ex. SENSOR_TYPE_LIGHT) */
float    max_value;                       /**< maximum value of this sensor's value in SI units */
float    min_value;                       /**< minimum value of this sensor's value in SI units */
float    resolution;                      /**< smallest difference between two values reported by this sensor */
int  min_delay;                       /**< min delay in microseconds between events. zero = not a constant rate */
} sensor_t;

class Adafruit_Sensor {
public:
// Constructor(s)
Adafruit_Sensor() {}
virtual ~Adafruit_Sensor() {}

// These must be defined by the subclass
virtual void enableAutoRange(bool enabled) {};
virtual bool getEvent(sensors_event_t*) = 0;
virtual void getSensor(sensor_t*) = 0;

private:
bool _autoRange;
};

#endif
#ifndef __ADAFRUIT_BNO055_H__
#define __ADAFRUIT_BNO055_H__
#endif

#ifndef BNO055_H
#define BNO055_H

//#include "Adafruit_Sensor.h"
//#include "imumaths.h"
#ifndef IMUMATH_H
#define IMUMATH_H


//#include "vector.h"


/*
    Inertial Measurement Unit Maths Library
    Copyright (C) 2013-2014  Samuel Cowen
    www.camelsoftware.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef IMUMATH_VECTOR_HPP
#define IMUMATH_VECTOR_HPP

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>


namespace imu
{

template <uint8_t N> class Vector
{
public:
Vector()
{
memset(p_vec, 0, sizeof(double)*N);
}

Vector(double a)
{
memset(p_vec, 0, sizeof(double)*N);
p_vec[0] = a;
}

Vector(double a, double b)
{
memset(p_vec, 0, sizeof(double)*N);
p_vec[0] = a;
p_vec[1] = b;
}

Vector(double a, double b, double c)
{
memset(p_vec, 0, sizeof(double)*N);
p_vec[0] = a;
p_vec[1] = b;
p_vec[2] = c;
}

Vector(double a, double b, double c, double d)
{
memset(p_vec, 0, sizeof(double)*N);
p_vec[0] = a;
p_vec[1] = b;
p_vec[2] = c;
p_vec[3] = d;
}

Vector(const Vector<N> &v)
{
for (int x = 0; x < N; x++)
p_vec[x] = v.p_vec[x];
}

~Vector()
{
}

uint8_t n() { return N; }

double magnitude()
{
double res = 0;
int i;
for(i = 0; i < N; i++)
res += (p_vec[i] * p_vec[i]);

if(isnan(res))
return 0;
if((fabs(res-1)) >= 0.000001) // Avoid a sqrt if possible.
return sqrt(res);
return 1;
}

void normalize()
{
double mag = magnitude();
if(abs(mag) <= 0.0001)
return;

int i;
for(i = 0; i < N; i++)
p_vec[i] = p_vec[i]/mag;
}

double dot(Vector v)
{
double ret = 0;
int i;
for(i = 0; i < N; i++)
ret += p_vec[i] * v.p_vec[i];

return ret;
}

Vector cross(Vector v)
{
Vector ret;

// The cross product is only valid for vectors with 3 dimensions,
// with the exception of higher dimensional stuff that is beyond the intended scope of this library
if(N != 3)
return ret;

ret.p_vec[0] = (p_vec[1] * v.p_vec[2]) - (p_vec[2] * v.p_vec[1]);
ret.p_vec[1] = (p_vec[2] * v.p_vec[0]) - (p_vec[0] * v.p_vec[2]);
ret.p_vec[2] = (p_vec[0] * v.p_vec[1]) - (p_vec[1] * v.p_vec[0]);
return ret;
}

Vector scale(double scalar) const
{
Vector ret;
for(int i = 0; i < N; i++)
ret.p_vec[i] = p_vec[i] * scalar;
return ret;
}

Vector invert() const
{
Vector ret;
for(int i = 0; i < N; i++)
ret.p_vec[i] = -p_vec[i];
return ret;
}

Vector operator = (Vector v)
{
for (int x = 0; x < N; x++ )
p_vec[x] = v.p_vec[x];
return *this;
}

double& operator [](int n)
{
return p_vec[n];
}

double operator [](int n) const
{
return p_vec[n];
}

double& operator ()(int n)
{
return p_vec[n];
}

double operator ()(int n) const
{
return p_vec[n];
}

Vector operator + (Vector v) const
{
Vector ret;
for(int i = 0; i < N; i++)
ret.p_vec[i] = p_vec[i] + v.p_vec[i];
return ret;
}

Vector operator - (Vector v) const
{
Vector ret;
for(int i = 0; i < N; i++)
ret.p_vec[i] = p_vec[i] - v.p_vec[i];
return ret;
}

Vector operator * (double scalar) const
{
return scale(scalar);
}

Vector operator / (double scalar) const
{
Vector ret;
for(int i = 0; i < N; i++)
ret.p_vec[i] = p_vec[i] / scalar;
return ret;
}

void toDegrees()
{
for(int i = 0; i < N; i++)
p_vec[i] *= 57.2957795131; //180/pi
}

void toRadians()
{
for(int i = 0; i < N; i++)
p_vec[i] *= 0.01745329251;  //pi/180
}

double& x() { return p_vec[0]; }
double& y() { return p_vec[1]; }
double& z() { return p_vec[2]; }
double x() const { return p_vec[0]; }
double y() const { return p_vec[1]; }
double z() const { return p_vec[2]; }


private:
double  p_vec[N];
};


};

#endif


//#include "matrix.h"
/*
    Inertial Measurement Unit Maths Library
    Copyright (C) 2013-2014  Samuel Cowen
    www.camelsoftware.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef IMUMATH_MATRIX_HPP
#define IMUMATH_MATRIX_HPP

namespace imu
{


template <uint8_t N> class Matrix
{
public:
Matrix()
{
int r = sizeof(double)*N;
_cell = &_cell_data[0];
memset(_cell, 0, r*r);
}

Matrix(const Matrix &v)
{
int r = sizeof(double)*N;
_cell = &_cell_data[0];
memset(_cell, 0, r*r);
for (int x = 0; x < N; x++ )
{
for(int y = 0; y < N; y++)
{
_cell[x*N+y] = v._cell[x*N+y];
}
}
}

~Matrix()
{
}

void operator = (Matrix m)
{
for(int x = 0; x < N; x++)
{
for(int y = 0; y < N; y++)
{
cell(x, y) = m.cell(x, y);
}
}
}

Vector<N> row_to_vector(int y)
{
Vector<N> ret;
for(int i = 0; i < N; i++)
{
ret[i] = _cell[y*N+i];
}
return ret;
}

Vector<N> col_to_vector(int x)
{
Vector<N> ret;
for(int i = 0; i < N; i++)
{
ret[i] = _cell[i*N+x];
}
return ret;
}

void vector_to_row(Vector<N> v, int row)
{
for(int i = 0; i < N; i++)
{
cell(row, i) = v(i);
}
}

void vector_to_col(Vector<N> v, int col)
{
for(int i = 0; i < N; i++)
{
cell(i, col) = v(i);
}
}

//double& operator ()(int x, int y)
double& operator()( int x,  int y) const
{
return _cell[x*N+y];
}

double& cell(int x, int y) const
{
return _cell[x*N+y];
}


Matrix operator + (Matrix m)
{
Matrix ret;
for(int x = 0; x < N; x++)
{
for(int y = 0; y < N; y++)
{
ret._cell[x*N+y] = _cell[x*N+y] + m._cell[x*N+y];
}
}
return ret;
}

Matrix operator - (Matrix m)
{
Matrix ret;
for(int x = 0; x < N; x++)
{
for(int y = 0; y < N; y++)
{
ret._cell[x*N+y] = _cell[x*N+y] - m._cell[x*N+y];
}
}
return ret;
}

Matrix operator * (double scalar)
{
Matrix ret;
for(int x = 0; x < N; x++)
{
for(int y = 0; y < N; y++)
{
ret._cell[x*N+y] = _cell[x*N+y] * scalar;
}
}
return ret;
}

Matrix operator * (Matrix m)
{
Matrix ret;
for(int x = 0; x < N; x++)
{
for(int y = 0; y < N; y++)
{
Vector<N> row = row_to_vector(x);
Vector<N> col = m.col_to_vector(y);
ret.cell(x, y) = row.dot(col);
}
}
return ret;
}

Matrix transpose()
{
Matrix ret;
for(int x = 0; x < N; x++)
{
for(int y = 0; y < N; y++)
{
ret.cell(y, x) = cell(x, y);
}
}
return ret;
}

Matrix<N-1> minor_matrix(int row, int col)
{
int colCount = 0, rowCount = 0;
Matrix<N-1> ret;
for(int i = 0; i < N; i++ )
{
if( i != row )
{
for(int j = 0; j < N; j++ )
{
if( j != col )
{
ret(rowCount, colCount) = cell(i, j);
colCount++;
}
}
rowCount++;
}
}
return ret;
}

double determinant()
{
if(N == 1)
return cell(0, 0);

float det = 0.0;
for(int i = 0; i < N; i++ )
{
Matrix<N-1> minor = minor_matrix(0, i);
det += (i%2==1?-1.0:1.0) * cell(0, i) * minor.determinant();
}
return det;
}

Matrix invert()
{
Matrix ret;
float det = determinant();

for(int x = 0; x < N; x++)
{
for(int y = 0; y < N; y++)
{
Matrix<N-1> minor = minor_matrix(y, x);
ret(x, y) = det*minor.determinant();
if( (x+y)%2 == 1)
ret(x, y) = -ret(x, y);
}
}
return ret;
}
double trace() const {
double tr = 0.0;
for (int i = 0; i < N; ++i)
tr += cell(i, i);
return tr;
}
private:
double* _cell;
double  _cell_data[N*N];
};


};

#endif

//#include "quaternion.h"
//  Inertial Measurement Unit Maths Library
//
//  Copyright 2013-2021 Sam Cowen <samuel.cowen@camelsoftware.com>
//  Bug fixes and cleanups by Gé Vissers (gvissers@gmail.com)
//
//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
//  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.

#ifndef IMUMATH_QUATERNION_HPP
#define IMUMATH_QUATERNION_HPP
//#include "matrix.h"

namespace imu {

class Quaternion {
public:
Quaternion() : _w(1.0), _x(0.0), _y(0.0), _z(0.0) {}

Quaternion(double w, double x, double y, double z)
: _w(w), _x(x), _y(y), _z(z) {}

Quaternion(double w, Vector<3> vec)
: _w(w), _x(vec.x()), _y(vec.y()), _z(vec.z()) {}

double &w() { return _w; }
double &x() { return _x; }
double &y() { return _y; }
double &z() { return _z; }

double w() const { return _w; }
double x() const { return _x; }
double y() const { return _y; }
double z() const { return _z; }

double magnitude() const {
return sqrt(_w * _w + _x * _x + _y * _y + _z * _z);
}

void normalize() {
double mag = magnitude();
*this = this->scale(1 / mag);
}

Quaternion conjugate() const { return Quaternion(_w, -_x, -_y, -_z); }

void fromAxisAngle(const Vector<3> &axis, double theta) {
_w = cos(theta / 2);
// only need to calculate sine of half theta once
double sht = sin(theta / 2);
_x = axis.x() * sht;
_y = axis.y() * sht;
_z = axis.z() * sht;
}

void fromMatrix(const Matrix<3> &m) {
double tr = m.trace();

double S;
if (tr > 0) {
S = sqrt(tr + 1.0) * 2;
_w = 0.25 * S;
_x = (m(2, 1) - m(1, 2)) / S;
_y = (m(0, 2) - m(2, 0)) / S;
_z = (m(1, 0) - m(0, 1)) / S;
} else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2)) {
S = sqrt(1.0 + m(0, 0) - m(1, 1) - m(2, 2)) * 2;
_w = (m(2, 1) - m(1, 2)) / S;
_x = 0.25 * S;
_y = (m(0, 1) + m(1, 0)) / S;
_z = (m(0, 2) + m(2, 0)) / S;
} else if (m(1, 1) > m(2, 2)) {
S = sqrt(1.0 + m(1, 1) - m(0, 0) - m(2, 2)) * 2;
_w = (m(0, 2) - m(2, 0)) / S;
_x = (m(0, 1) + m(1, 0)) / S;
_y = 0.25 * S;
_z = (m(1, 2) + m(2, 1)) / S;
} else {
S = sqrt(1.0 + m(2, 2) - m(0, 0) - m(1, 1)) * 2;
_w = (m(1, 0) - m(0, 1)) / S;
_x = (m(0, 2) + m(2, 0)) / S;
_y = (m(1, 2) + m(2, 1)) / S;
_z = 0.25 * S;
}
}

void toAxisAngle(Vector<3> &axis, double &angle) const {
double sqw = sqrt(1 - _w * _w);
if (sqw == 0) // it's a singularity and divide by zero, avoid
return;

angle = 2 * acos(_w);
axis.x() = _x / sqw;
axis.y() = _y / sqw;
axis.z() = _z / sqw;
}

Matrix<3> toMatrix() const {
Matrix<3> ret;
ret.cell(0, 0) = 1 - 2 * _y * _y - 2 * _z * _z;
ret.cell(0, 1) = 2 * _x * _y - 2 * _w * _z;
ret.cell(0, 2) = 2 * _x * _z + 2 * _w * _y;

ret.cell(1, 0) = 2 * _x * _y + 2 * _w * _z;
ret.cell(1, 1) = 1 - 2 * _x * _x - 2 * _z * _z;
ret.cell(1, 2) = 2 * _y * _z - 2 * _w * _x;

ret.cell(2, 0) = 2 * _x * _z - 2 * _w * _y;
ret.cell(2, 1) = 2 * _y * _z + 2 * _w * _x;
ret.cell(2, 2) = 1 - 2 * _x * _x - 2 * _y * _y;
return ret;
}

// Returns euler angles that represent the quaternion.  Angles are
// returned in rotation order and right-handed about the specified
// axes:
//
//   v[0] is applied 1st about z (ie, roll)
//   v[1] is applied 2nd about y (ie, pitch)
//   v[2] is applied 3rd about x (ie, yaw)
//
// Note that this means result.x() is not a rotation about x;
// similarly for result.z().
//
Vector<3> toEuler() const {
Vector<3> ret;
double sqw = _w * _w;
double sqx = _x * _x;
double sqy = _y * _y;
double sqz = _z * _z;

ret.x() = atan2(2.0 * (_x * _y + _z * _w), (sqx - sqy - sqz + sqw));
ret.y() = asin(-2.0 * (_x * _z - _y * _w) / (sqx + sqy + sqz + sqw));
ret.z() = atan2(2.0 * (_y * _z + _x * _w), (-sqx - sqy + sqz + sqw));

return ret;
}

Vector<3> toAngularVelocity(double dt) const {
Vector<3> ret;
Quaternion one(1.0, 0.0, 0.0, 0.0);
Quaternion delta = one - *this;
Quaternion r = (delta / dt);
r = r * 2;
r = r * one;

ret.x() = r.x();
ret.y() = r.y();
ret.z() = r.z();
return ret;
}

Vector<3> rotateVector(const Vector<2> &v) const {
return rotateVector(Vector<3>(v.x(), v.y()));
}

Vector<3> rotateVector(const Vector<3> &v) const {
Vector<3> qv(_x, _y, _z);
Vector<3> t = qv.cross(v) * 2.0;
return v + t * _w + qv.cross(t);
}

Quaternion operator*(const Quaternion &q) const {
return Quaternion(_w * q._w - _x * q._x - _y * q._y - _z * q._z,
_w * q._x + _x * q._w + _y * q._z - _z * q._y,
_w * q._y - _x * q._z + _y * q._w + _z * q._x,
_w * q._z + _x * q._y - _y * q._x + _z * q._w);
}

Quaternion operator+(const Quaternion &q) const {
return Quaternion(_w + q._w, _x + q._x, _y + q._y, _z + q._z);
}

Quaternion operator-(const Quaternion &q) const {
return Quaternion(_w - q._w, _x - q._x, _y - q._y, _z - q._z);
}

Quaternion operator/(double scalar) const {
return Quaternion(_w / scalar, _x / scalar, _y / scalar, _z / scalar);
}

Quaternion operator*(double scalar) const { return scale(scalar); }

Quaternion scale(double scalar) const {
return Quaternion(_w * scalar, _x * scalar, _y * scalar, _z * scalar);
}

private:
double _w, _x, _y, _z;
};

} // namespace imu

#endif

#endif
//#include "mbed.h"

#define BNO055_ADDRESS_A (0x28)
#define BNO055_ADDRESS_B (0x29)
#define BNO055_ID        (0xA0)
enum {MT_P0 = 0, MT_P1, MT_P2, MT_P3, MT_P4, MT_P5, MT_P6, MT_P7};

//class Adafruit_BNO055 : public Adafruit_Sensor
class BNO055 : public Adafruit_Sensor
{
public:
typedef enum
{
/* Page id register definition */
BNO055_PAGE_ID_ADDR                                     = 0X07,

/* PAGE0 REGISTER DEFINITION START*/
BNO055_CHIP_ID_ADDR                                     = 0x00,
BNO055_ACCEL_REV_ID_ADDR                                = 0x01,
BNO055_MAG_REV_ID_ADDR                                  = 0x02,
BNO055_GYRO_REV_ID_ADDR                                 = 0x03,
BNO055_SW_REV_ID_LSB_ADDR                               = 0x04,
BNO055_SW_REV_ID_MSB_ADDR                               = 0x05,
BNO055_BL_REV_ID_ADDR                                   = 0X06,

/* Accel data register */
BNO055_ACCEL_DATA_X_LSB_ADDR                            = 0X08,
BNO055_ACCEL_DATA_X_MSB_ADDR                            = 0X09,
BNO055_ACCEL_DATA_Y_LSB_ADDR                            = 0X0A,
BNO055_ACCEL_DATA_Y_MSB_ADDR                            = 0X0B,
BNO055_ACCEL_DATA_Z_LSB_ADDR                            = 0X0C,
BNO055_ACCEL_DATA_Z_MSB_ADDR                            = 0X0D,

/* Mag data register */
BNO055_MAG_DATA_X_LSB_ADDR                              = 0X0E,
BNO055_MAG_DATA_X_MSB_ADDR                              = 0X0F,
BNO055_MAG_DATA_Y_LSB_ADDR                              = 0X10,
BNO055_MAG_DATA_Y_MSB_ADDR                              = 0X11,
BNO055_MAG_DATA_Z_LSB_ADDR                              = 0X12,
BNO055_MAG_DATA_Z_MSB_ADDR                              = 0X13,

/* Gyro data registers */
BNO055_GYRO_DATA_X_LSB_ADDR                             = 0X14,
BNO055_GYRO_DATA_X_MSB_ADDR                             = 0X15,
BNO055_GYRO_DATA_Y_LSB_ADDR                             = 0X16,
BNO055_GYRO_DATA_Y_MSB_ADDR                             = 0X17,
BNO055_GYRO_DATA_Z_LSB_ADDR                             = 0X18,
BNO055_GYRO_DATA_Z_MSB_ADDR                             = 0X19,

/* Euler data registers */
BNO055_EULER_H_LSB_ADDR                                 = 0X1A,
BNO055_EULER_H_MSB_ADDR                                 = 0X1B,
BNO055_EULER_R_LSB_ADDR                                 = 0X1C,
BNO055_EULER_R_MSB_ADDR                                 = 0X1D,
BNO055_EULER_P_LSB_ADDR                                 = 0X1E,
BNO055_EULER_P_MSB_ADDR                                 = 0X1F,

/* Quaternion data registers */
BNO055_QUATERNION_DATA_W_LSB_ADDR                       = 0X20,
BNO055_QUATERNION_DATA_W_MSB_ADDR                       = 0X21,
BNO055_QUATERNION_DATA_X_LSB_ADDR                       = 0X22,
BNO055_QUATERNION_DATA_X_MSB_ADDR                       = 0X23,
BNO055_QUATERNION_DATA_Y_LSB_ADDR                       = 0X24,
BNO055_QUATERNION_DATA_Y_MSB_ADDR                       = 0X25,
BNO055_QUATERNION_DATA_Z_LSB_ADDR                       = 0X26,
BNO055_QUATERNION_DATA_Z_MSB_ADDR                       = 0X27,

/* Linear acceleration data registers */
BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR                     = 0X28,
BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR                     = 0X29,
BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR                     = 0X2A,
BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR                     = 0X2B,
BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR                     = 0X2C,
BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR                     = 0X2D,

/* Gravity data registers */
BNO055_GRAVITY_DATA_X_LSB_ADDR                          = 0X2E,
BNO055_GRAVITY_DATA_X_MSB_ADDR                          = 0X2F,
BNO055_GRAVITY_DATA_Y_LSB_ADDR                          = 0X30,
BNO055_GRAVITY_DATA_Y_MSB_ADDR                          = 0X31,
BNO055_GRAVITY_DATA_Z_LSB_ADDR                          = 0X32,
BNO055_GRAVITY_DATA_Z_MSB_ADDR                          = 0X33,

/* Temperature data register */
BNO055_TEMP_ADDR                                        = 0X34,

/* Status registers */
BNO055_CALIB_STAT_ADDR                                  = 0X35,
BNO055_SELFTEST_RESULT_ADDR                             = 0X36,
BNO055_INTR_STAT_ADDR                                   = 0X37,

BNO055_SYS_CLK_STAT_ADDR                                = 0X38,
BNO055_SYS_STAT_ADDR                                    = 0X39,
BNO055_SYS_ERR_ADDR                                     = 0X3A,

/* Unit selection register */
BNO055_UNIT_SEL_ADDR                                    = 0X3B,
BNO055_DATA_SELECT_ADDR                                 = 0X3C,

/* Mode registers */
BNO055_OPR_MODE_ADDR                                    = 0X3D,
BNO055_PWR_MODE_ADDR                                    = 0X3E,

BNO055_SYS_TRIGGER_ADDR                                 = 0X3F,
BNO055_TEMP_SOURCE_ADDR                                 = 0X40,

/* Axis remap registers */
BNO055_AXIS_MAP_CONFIG_ADDR                             = 0X41,
BNO055_AXIS_MAP_SIGN_ADDR                               = 0X42,

/* SIC registers */
BNO055_SIC_MATRIX_0_LSB_ADDR                            = 0X43,
BNO055_SIC_MATRIX_0_MSB_ADDR                            = 0X44,
BNO055_SIC_MATRIX_1_LSB_ADDR                            = 0X45,
BNO055_SIC_MATRIX_1_MSB_ADDR                            = 0X46,
BNO055_SIC_MATRIX_2_LSB_ADDR                            = 0X47,
BNO055_SIC_MATRIX_2_MSB_ADDR                            = 0X48,
BNO055_SIC_MATRIX_3_LSB_ADDR                            = 0X49,
BNO055_SIC_MATRIX_3_MSB_ADDR                            = 0X4A,
BNO055_SIC_MATRIX_4_LSB_ADDR                            = 0X4B,
BNO055_SIC_MATRIX_4_MSB_ADDR                            = 0X4C,
BNO055_SIC_MATRIX_5_LSB_ADDR                            = 0X4D,
BNO055_SIC_MATRIX_5_MSB_ADDR                            = 0X4E,
BNO055_SIC_MATRIX_6_LSB_ADDR                            = 0X4F,
BNO055_SIC_MATRIX_6_MSB_ADDR                            = 0X50,
BNO055_SIC_MATRIX_7_LSB_ADDR                            = 0X51,
BNO055_SIC_MATRIX_7_MSB_ADDR                            = 0X52,
BNO055_SIC_MATRIX_8_LSB_ADDR                            = 0X53,
BNO055_SIC_MATRIX_8_MSB_ADDR                            = 0X54,

/* Accelerometer Offset registers */
ACCEL_OFFSET_X_LSB_ADDR                                 = 0X55,
ACCEL_OFFSET_X_MSB_ADDR                                 = 0X56,
ACCEL_OFFSET_Y_LSB_ADDR                                 = 0X57,
ACCEL_OFFSET_Y_MSB_ADDR                                 = 0X58,
ACCEL_OFFSET_Z_LSB_ADDR                                 = 0X59,
ACCEL_OFFSET_Z_MSB_ADDR                                 = 0X5A,

/* Magnetometer Offset registers */
MAG_OFFSET_X_LSB_ADDR                                   = 0X5B,
MAG_OFFSET_X_MSB_ADDR                                   = 0X5C,
MAG_OFFSET_Y_LSB_ADDR                                   = 0X5D,
MAG_OFFSET_Y_MSB_ADDR                                   = 0X5E,
MAG_OFFSET_Z_LSB_ADDR                                   = 0X5F,
MAG_OFFSET_Z_MSB_ADDR                                   = 0X60,

/* Gyroscope Offset register s*/
GYRO_OFFSET_X_LSB_ADDR                                  = 0X61,
GYRO_OFFSET_X_MSB_ADDR                                  = 0X62,
GYRO_OFFSET_Y_LSB_ADDR                                  = 0X63,
GYRO_OFFSET_Y_MSB_ADDR                                  = 0X64,
GYRO_OFFSET_Z_LSB_ADDR                                  = 0X65,
GYRO_OFFSET_Z_MSB_ADDR                                  = 0X66,

/* Radius registers */
ACCEL_RADIUS_LSB_ADDR                                   = 0X67,
ACCEL_RADIUS_MSB_ADDR                                   = 0X68,
MAG_RADIUS_LSB_ADDR                                     = 0X69,
MAG_RADIUS_MSB_ADDR                                     = 0X6A
} adafruit_bno055_reg_t;

typedef enum
{
POWER_MODE_NORMAL                                       = 0X00,
POWER_MODE_LOWPOWER                                     = 0X01,
POWER_MODE_SUSPEND                                      = 0X02
} adafruit_bno055_powermode_t;

typedef enum
{
/* Operation mode settings*/
OPERATION_MODE_CONFIG                                   = 0X00,
OPERATION_MODE_ACCONLY                                  = 0X01,
OPERATION_MODE_MAGONLY                                  = 0X02,
OPERATION_MODE_GYRONLY                                  = 0X03,
OPERATION_MODE_ACCMAG                                   = 0X04,
OPERATION_MODE_ACCGYRO                                  = 0X05,
OPERATION_MODE_MAGGYRO                                  = 0X06,
OPERATION_MODE_AMG                                      = 0X07,
OPERATION_MODE_IMUPLUS                                  = 0X08,
OPERATION_MODE_COMPASS                                  = 0X09,
OPERATION_MODE_M4G                                      = 0X0A,
OPERATION_MODE_NDOF_FMC_OFF                             = 0X0B,
OPERATION_MODE_NDOF                                     = 0X0C
} adafruit_bno055_opmode_t;

typedef struct
{
uint8_t  accel_rev;
uint8_t  mag_rev;
uint8_t  gyro_rev;
uint16_t sw_rev;
uint8_t  bl_rev;
} adafruit_bno055_rev_info_t;

typedef enum
{
VECTOR_ACCELEROMETER = BNO055_ACCEL_DATA_X_LSB_ADDR,
VECTOR_MAGNETOMETER  = BNO055_MAG_DATA_X_LSB_ADDR,
VECTOR_GYROSCOPE     = BNO055_GYRO_DATA_X_LSB_ADDR,
VECTOR_EULER         = BNO055_EULER_H_LSB_ADDR,
VECTOR_LINEARACCEL   = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR,
VECTOR_GRAVITY       = BNO055_GRAVITY_DATA_X_LSB_ADDR
} adafruit_vector_type_t;

//Adafruit_BNO055 ( int32_t sensorID = -1, uint8_t address = BNO055_ADDRESS_A, I2C* i2c_ptr = 0 );
BNO055 (I2C& i2c_ptr, int32_t sensorID = -1, uint8_t address = BNO055_ADDRESS_A);

bool  begin               ( adafruit_bno055_opmode_t mode = OPERATION_MODE_NDOF );
void  setMode             ( adafruit_bno055_opmode_t mode );
void  getRevInfo          ( adafruit_bno055_rev_info_t* );
void  displayRevInfo      ( void );
void  setExtCrystalUse    ( bool usextal );
void  getSystemStatus     ( uint8_t *system_status,
                            uint8_t *self_test_result,
                            uint8_t *system_error);
void  displaySystemStatus ( void );
void  getCalibration      ( uint8_t* system, uint8_t* gyro, uint8_t* accel, uint8_t* mag);
void  get_angular_position_quat
                          ( BNO055_ANGULAR_POSITION_typedef *an_pos);
void set_mounting_position( uint8_t position);



imu::Vector<3>  getVector ( adafruit_vector_type_t vector_type );
imu::Quaternion getQuat   ( void );
int8_t          getTemp   ( void );

/* Adafruit_Sensor implementation */
bool  getEvent  ( sensors_event_t* );
void  getSensor ( sensor_t* );

private:
char  read8   ( adafruit_bno055_reg_t );
bool  readLen ( adafruit_bno055_reg_t, char* buffer, int len );
bool  write8  ( adafruit_bno055_reg_t, char value );
//uint8_t BNO055::select_page(uint8_t page);

uint8_t _address;
int32_t _sensorID;
adafruit_bno055_opmode_t _mode;
I2C& i2c;
};

#endif
//
//
/////*
//// * mbed library program
//// *  BNO055 Intelligent 9-axis absolute orientation sensor
//// *  by Bosch Sensortec
//// *
//// * Copyright (c) 2015,'17 Kenji Arai / JH1PJL
//// *  http://www.page.sannet.ne.jp/kenjia/index.html
//// *  http://mbed.org/users/kenjiArai/
//// *      Created: March     30th, 2015
//// *      Revised: August    23rd, 2017
//// */
/////*
//// *---------------- REFERENCE ----------------------------------------------------------------------
//// * Original Information
//// *  https://www.bosch-sensortec.com/en/homepage/products_3/sensor_hubs/iot_solutions/bno055_1/bno055_4
//// *  Intelligent 9-axis absolute orientation sensor / Data Sheet  BST_BNO055_DS000_12 Nov. 2014 rev.1.2
//// *  Sample software   https://github.com/BoschSensortec/BNO055_driver
//// * Sensor board
//// *  https://www.rutronik24.com/product/bosch+se/bno055+shuttle+board+mems/6431291.html
//// *  http://microcontrollershop.com/product_info.php?products_id=7140&osCsid=10645k86db2crld4tfi0vol5g5
//// */
////
/////*
//// * This library has been modified. Original can be found here: https://os.mbed.com/users/AlexanderLill/code/BNO055_fusion/
//// */
////
////#ifndef BNO055_H
////#define BNO055_H
////
////#include "mbed.h"
////
////#define PI 3.14159265
////
//////  BNO055
//////  7bit address = 0b010100x(0x28 or 0x29 depends on COM3)
////#define BNO055_G_CHIP_ADDR      (0x28 << 1) // COM3 = GND
////#define BNO055_V_CHIP_ADDR      (0x29 << 1) // COM3 = Vdd
////
////// Fusion mode
////#define CONFIGMODE              0x00
////#define MODE_IMU                0x08
////#define MODE_COMPASS            0x09
////#define MODE_M4G                0x0a
////#define MODE_NDOF_FMC_OFF       0x0b
////#define MODE_NDOF               0x0c
////
//////  UNIT
////#define UNIT_ACC_MSS            0x00    // acc m/s2
////#define UNIT_ACC_MG             0x01    // acc mg
////#define UNIT_GYR_DPS            0x00    // gyro Dps
////#define UNIT_GYR_RPS            0x02    // gyro Rps
////#define UNIT_EULER_DEG          0x00    // euler Degrees
////#define UNIT_EULER_RAD          0x04    // euler Radians
////#define UNIT_TEMP_C             0x00    // temperature degC
////#define UNIT_TEMP_F             0x10    // temperature degF
////#define UNIT_ORI_WIN            0x00    // Windows orientation
////#define UNIT_ORI_ANDROID        0x80    // Android orientation
////
//////  ID's
////#define I_AM_BNO055_CHIP        0xa0    // CHIP ID
////#define I_AM_BNO055_ACC         0xfb    // ACC ID
////#define I_AM_BNO055_MAG         0x32    // MAG ID
////#define I_AM_BNO055_GYR         0x0f    // GYR ID
////
////////////////// DATA TYPE DEFINITION ///////////////////////
////typedef struct {
////    uint8_t  chip_id;
////    uint8_t  acc_id;
////    uint8_t  mag_id;
////    uint8_t  gyr_id;
////    uint8_t  bootldr_rev_id;
////    uint16_t sw_rev_id;
////} BNO055_ID_INF_TypeDef;
////
////typedef struct {
////    double h;
////    double r;
////    double p;
////} BNO055_EULER_TypeDef;
////
////typedef struct {
////    double x;
////    double y;
////    double z;
////    double w;
////} BNO055_QUATERNION_TypeDef;
////
////typedef struct{
////    double yaw;
////    double roll;
////    double pitch;
////} BNO055_ANGULAR_POSITION_typedef;
////
////typedef struct {
////    double x;
////    double y;
////    double z;
////} BNO055_VECTOR_TypeDef;
////
////typedef struct {
////    int8_t acc_chip;
////    int8_t gyr_chip;
////} BNO055_TEMPERATURE_TypeDef;
////
////enum {MT_P0 = 0, MT_P1, MT_P2, MT_P3, MT_P4, MT_P5, MT_P6, MT_P7};
////
/////** Interface for Bosch Sensortec Intelligent 9-axis absolute orientation sensor
//// *      Chip: BNO055
//// *
//// * @code
//// * #include    "mbed.h"
//// * #include    "BNO055.h"
//// *
//// * Serial pc(USBTX,USBRX);
//// * I2C    i2c(PB_9, PB_8);         // SDA, SCL
//// * BNO055 imu(i2c, PA_8);          // Reset
//// *
//// * BNO055_ID_INF_TypeDef bno055_id_inf;
//// * BNO055_EULER_TypeDef  euler_angles;
//// *
//// * int main() {
//// *     pc.printf("Bosch Sensortec BNO055 test program on " __DATE__ "/" __TIME__ "\r\n");
//// *     if (imu.chip_ready() == 0){
//// *         pc.printf("Bosch BNO055 is NOT avirable!!\r\n");
//// *     }
//// *     imu.read_id_inf(&bno055_id_inf);
//// *     pc.printf("CHIP:0x%02x, ACC:0x%02x, MAG:0x%02x, GYR:0x%02x, , SW:0x%04x, , BL:0x%02x\r\n",
//// *                bno055_id_inf.chip_id, bno055_id_inf.acc_id, bno055_id_inf.mag_id,
//// *                bno055_id_inf.gyr_id, bno055_id_inf.sw_rev_id, bno055_id_inf.bootldr_rev_id);
//// *     while(1) {
//// *         imu.get_Euler_Angles(&euler_angles);
//// *         pc.printf("Heading:%+6.1f [deg], Roll:%+6.1f [deg], Pich:%+6.1f [deg]\r\n",
//// *                    euler_angles.h, euler_angles.r, euler_angles.p);
//// *         wait(0.5);
//// *     }
//// * }
//// * @endcode
//// */
////
////class BNO055
////{
////public:
////    /** Configure data pin
////      * @param data SDA and SCL pins
////      * @param device address
////      */
////    BNO055(PinName p_sda, PinName p_scl, PinName p_reset, uint8_t addr, uint8_t mode);
////
////    /** Configure data pin
////      * @param data SDA and SCL pins
////      * @param Other parameters are set default data
////      */
////    BNO055(PinName p_sda, PinName p_scl, PinName p_reset);
////
////    /** Configure data pin (with other devices on I2C line)
////      * @param I2C previous definition
////      * @param device address
////      */
////    BNO055(I2C& p_i2c, PinName p_reset, uint8_t addr, uint8_t mode);
////
////    /** Configure data pin (with other devices on I2C line)
////      * @param I2C previous definition
////      * @param Other parameters are set default data
////      */
////    BNO055(I2C& p_i2c, PinName p_reset, uint8_t mode);
////
////    /** Get Euler Angles
////     * @param double type of 3D data address
////     */
////    void get_euler_angles(BNO055_EULER_TypeDef *el);
////
////    /** Get Quaternion XYZ&W
////     * @param int16_t type of 4D data address
////     */
////    void get_quaternion(BNO055_QUATERNION_TypeDef *qua);
////
////    /** Get Angular position from quaternion
////     *  @param double type of 3D data address
////     */
////    void get_angular_position_quat(BNO055_ANGULAR_POSITION_typedef *an_pos);
////
////    /** Get Linear accel data
////     * @param double type of 3D data address
////     */
////    void get_linear_accel(BNO055_VECTOR_TypeDef *la);
////
////    /** Get Mag data
////     * @param double type of 3D data address
////     */
////    void get_mag(BNO055_VECTOR_TypeDef *qua);
////
////    /** Get Accel data
////     * @param double type of 3D data address
////     */
////    void get_accel(BNO055_VECTOR_TypeDef *la);
////
////    /** Get Gyro data
////     * @param double type of 3D data address
////     */
////    void get_gyro(BNO055_VECTOR_TypeDef *gr);
////
////    /** Get Gravity data
////     * @param double type of 3D data address
////     */
////    void get_gravity(BNO055_VECTOR_TypeDef *gr);
////
////    /** Get Chip temperature data both Acc & Gyro
////     * @param int8_t type of data address
////     */
////    void get_chip_temperature(BNO055_TEMPERATURE_TypeDef *tmp);
////
////    /** Change fusion mode
////      * @param fusion mode
////      * @return none
////      */
////    void change_fusion_mode(uint8_t mode);
////
////    /** Set Mouting position
////      *  Please make sure your mounting direction of BNO055 chip
////      *  refrence: BNO055 data sheet BST-BNO055-DS000-12 3.4 Axis remap
////      * @param Set P0 to P7 mounting position data
////      * @return none
////      */
////    void set_mounting_position(uint8_t position);
////
////    /** Read BNO055 ID information
////      * @param ID information address
////      * @return none
////      */
////    void read_id_inf(BNO055_ID_INF_TypeDef *id);
////
////    /** Check chip is avairable or not
////      * @param none
////      * @return OK = 1, NG = 0;
////      */
////    uint8_t chip_ready(void);
////
////    /** Calibrate IMU
////      * @param none
////      * @return none
////      */
////    void calibrate(void);
////
////    /** Read calibration status
////      * @param none
////      * @return SYS(7:6),GYR(5:4),ACC(3:2),MAG(1:0) 3 = Calibrated, 0= not yet
////      */
////    uint8_t read_calib_status(void);
////
////    /** Reset
////      * @param none
////      * @return 0 = sucess, 1 = Not available chip
////      */
////    uint8_t reset(void);
////
////    /** Set I2C clock frequency
////      * @param freq.
////      * @return none
////      */
////    void frequency(int hz);
////
////    /** Read page 0 register
////      * @param register's address
////      * @return register data
////      */
////    uint8_t read_reg0(uint8_t addr);
////
////    /** Write page 0 register
////      * @param register's address
////      * @param data
////      * @return register data
////      */
////    uint8_t write_reg0(uint8_t addr, uint8_t data);
////
////    /** Read page 1 register
////      * @param register's address
////      * @return register data
////      */
////    uint8_t read_reg1(uint8_t addr);
////
////    /** Write page 1 register
////      * @param register's address
////      * @param data
////      * @return register data
////      */
////    uint8_t write_reg1(uint8_t addr, uint8_t data);
////
////    /** Determine if degrees (instead of radians) are used.
////      * @return true if degrees are used
////      */
////    bool use_degrees();
////
////    /** Determine if m/s*s (instead of mg) is used.
////      * @return true if m/s*s are used
////      */
////    bool use_mss();
////
////    /** Determine if dps (instead of rps) are used.
////      * @return true if dps are used
////      */
////    bool use_dps();
////
////    /** Determine if celsius (instead of fahrenheit) is used.
////      * @return true if celsius are used
////      */
////    bool use_celsius();
////
////    int cantReadDataCount;
////
////    double multiturnYaw;
////
////protected:
////    void initialize(void);
////    void initialize_reset_pin(void);
////    void get_id(void);
////    void set_initial_dt_to_regs(void);
////    void unit_selection(void);
////    uint8_t get_operating_mode(void);
////    uint8_t select_page(uint8_t page);
////
////    I2C *_i2c_p;
////    I2C &_i2c;
////    DigitalOut _res;
////
////private:
////    char     dt[10];      // working buffer
////    uint8_t  chip_addr;
////    uint8_t  chip_mode;
////    uint8_t  ready_flag;
////    uint8_t  page_flag;
////
////    uint8_t  chip_id;
////    uint8_t  acc_id;
////    uint8_t  mag_id;
////    uint8_t  gyr_id;
////    uint8_t  bootldr_rev_id;
////    uint16_t sw_rev_id;
////
////    bool unit_flag_is_set(uint8_t flag);
////};
////
//////---------------------------------------------------------
//////----- Register's definition -----------------------------
//////---------------------------------------------------------
////// Page id register definition
////#define BNO055_PAGE_ID          0x07
////
//////----- page0 ---------------------------------------------
////#define BNO055_CHIP_ID          0x00
////#define BNO055_ACCEL_REV_ID     0x01
////#define BNO055_MAG_REV_ID       0x02
////#define BNO055_GYRO_REV_ID      0x03
////#define BNO055_SW_REV_ID_LSB    0x04
////#define BNO055_SW_REV_ID_MSB    0x05
////#define BNO055_BL_REV_ID        0x06
////
////// Accel data register*/
////#define BNO055_ACC_X_LSB        0x08
////#define BNO055_ACC_X_MSB        0x09
////#define BNO055_ACC_Y_LSB        0x0a
////#define BNO055_ACC_Y_MSB        0x0b
////#define BNO055_ACC_Z_LSB        0x0c
////#define BNO055_ACC_Z_MSB        0x0d
////
////// Mag data register
////#define BNO055_MAG_X_LSB        0x0e
////#define BNO055_MAG_X_MSB        0x0f
////#define BNO055_MAG_Y_LSB        0x10
////#define BNO055_MAG_Y_MSB        0x11
////#define BNO055_MAG_Z_LSB        0x12
////#define BNO055_MAG_Z_MSB        0x13
////
////// Gyro data registers
////#define BNO055_GYR_X_LSB        0x14
////#define BNO055_GYR_X_MSB        0x15
////#define BNO055_GYR_Y_LSB        0x16
////#define BNO055_GYR_Y_MSB        0x17
////#define BNO055_GYR_Z_LSB        0x18
////#define BNO055_GYR_Z_MSB        0x19
////
////// Euler data registers
////#define BNO055_EULER_H_LSB      0x1a
////#define BNO055_EULER_H_MSB      0x1b
////
////#define BNO055_EULER_R_LSB      0x1c
////#define BNO055_EULER_R_MSB      0x1d
////
////#define BNO055_EULER_P_LSB      0x1e
////#define BNO055_EULER_P_MSB      0x1f
////
////// Quaternion data registers
////#define BNO055_QUATERNION_W_LSB 0x20
////#define BNO055_QUATERNION_W_MSB 0x21
////#define BNO055_QUATERNION_X_LSB 0x22
////#define BNO055_QUATERNION_X_MSB 0x23
////#define BNO055_QUATERNION_Y_LSB 0x24
////#define BNO055_QUATERNION_Y_MSB 0x25
////#define BNO055_QUATERNION_Z_LSB 0x26
////#define BNO055_QUATERNION_Z_MSB 0x27
////
////// Linear acceleration data registers
////#define BNO055_LINEAR_ACC_X_LSB 0x28
////#define BNO055_LINEAR_ACC_X_MSB 0x29
////#define BNO055_LINEAR_ACC_Y_LSB 0x2a
////#define BNO055_LINEAR_ACC_Y_MSB 0x2b
////#define BNO055_LINEAR_ACC_Z_LSB 0x2c
////#define BNO055_LINEAR_ACC_Z_MSB 0x2d
////
////// Gravity data registers
////#define BNO055_GRAVITY_X_LSB    0x2e
////#define BNO055_GRAVITY_X_MSB    0x2f
////#define BNO055_GRAVITY_Y_LSB    0x30
////#define BNO055_GRAVITY_Y_MSB    0x31
////#define BNO055_GRAVITY_Z_LSB    0x32
////#define BNO055_GRAVITY_Z_MSB    0x33
////
////// Temperature data register
////#define BNO055_TEMP             0x34
////
////// Status registers
////#define BNO055_CALIB_STAT       0x35
////#define BNO055_SELFTEST_RESULT  0x36
////#define BNO055_INTR_STAT        0x37
////#define BNO055_SYS_CLK_STAT     0x38
////#define BNO055_SYS_STAT         0x39
////#define BNO055_SYS_ERR          0x3a
////
////// Unit selection register
////#define BNO055_UNIT_SEL         0x3b
////#define BNO055_DATA_SELECT      0x3c
////
////// Mode registers
////#define BNO055_OPR_MODE         0x3d
////#define BNO055_PWR_MODE         0x3e
////#define BNO055_SYS_TRIGGER      0x3f
////#define BNO055_TEMP_SOURCE      0x40
////
////// Axis remap registers
////#define BNO055_AXIS_MAP_CONFIG  0x41
////#define BNO055_AXIS_MAP_SIGN    0x42
////
////// SIC registers
////#define BNO055_SIC_MTRX_0_LSB   0x43
////#define BNO055_SIC_MTRX_0_MSB   0x44
////#define BNO055_SIC_MTRX_1_LSB   0x45
////#define BNO055_SIC_MTRX_1_MSB   0x46
////#define BNO055_SIC_MTRX_2_LSB   0x47
////#define BNO055_SIC_MTRX_2_MSB   0x48
////#define BNO055_SIC_MTRX_3_LSB   0x49
////#define BNO055_SIC_MTRX_3_MSB   0x4a
////#define BNO055_SIC_MTRX_4_LSB   0x4b
////#define BNO055_SIC_MTRX_4_MSB   0x4c
////#define BNO055_SIC_MTRX_5_LSB   0x4d
////#define BNO055_SIC_MTRX_5_MSB   0x4e
////#define BNO055_SIC_MTRX_6_LSB   0x4f
////#define BNO055_SIC_MTRX_6_MSB   0x50
////#define BNO055_SIC_MTRX_7_LSB   0x51
////#define BNO055_SIC_MTRX_7_MSB   0x52
////#define BNO055_SIC_MTRX_8_LSB   0x53
////#define BNO055_SIC_MTRX_8_MSB   0x54
////
////// Accelerometer Offset registers
////#define ACCEL_OFFSET_X_LSB      0x55
////#define ACCEL_OFFSET_X_MSB      0x56
////#define ACCEL_OFFSET_Y_LSB      0x57
////#define ACCEL_OFFSET_Y_MSB      0x58
////#define ACCEL_OFFSET_Z_LSB      0x59
////#define ACCEL_OFFSET_Z_MSB      0x5a
////
////// Magnetometer Offset registers
////#define MAG_OFFSET_X_LSB        0x5b
////#define MAG_OFFSET_X_MSB        0x5c
////#define MAG_OFFSET_Y_LSB        0x5d
////#define MAG_OFFSET_Y_MSB        0x5e
////#define MAG_OFFSET_Z_LSB        0x5f
////#define MAG_OFFSET_Z_MSB        0x60
////
////// Gyroscope Offset registers
////#define GYRO_OFFSET_X_LSB       0x61
////#define GYRO_OFFSET_X_MSB       0x62
////#define GYRO_OFFSET_Y_LSB       0x63
////#define GYRO_OFFSET_Y_MSB       0x64
////#define GYRO_OFFSET_Z_LSB       0x65
////#define GYRO_OFFSET_Z_MSB       0x66
////
////// Radius registers
////#define ACCEL_RADIUS_LSB        0x67
////#define ACCEL_RADIUS_MSB        0x68
////#define MAG_RADIUS_LSB          0x69
////#define MAG_RADIUS_MSB          0x6a
////
//////----- page1 ---------------------------------------------
////// Configuration registers
////#define ACCEL_CONFIG            0x08
////#define MAG_CONFIG              0x09
////#define GYRO_CONFIG             0x0a
////#define GYRO_MODE_CONFIG        0x0b
////#define ACCEL_SLEEP_CONFIG      0x0c
////#define GYRO_SLEEP_CONFIG       0x0d
////#define MAG_SLEEP_CONFIG        0x0e
////
////// Interrupt registers
////#define INT_MASK                0x0f
////#define INT                     0x10
////#define ACCEL_ANY_MOTION_THRES  0x11
////#define ACCEL_INTR_SETTINGS     0x12
////#define ACCEL_HIGH_G_DURN       0x13
////#define ACCEL_HIGH_G_THRES      0x14
////#define ACCEL_NO_MOTION_THRES   0x15
////#define ACCEL_NO_MOTION_SET     0x16
////#define GYRO_INTR_SETING        0x17
////#define GYRO_HIGHRATE_X_SET     0x18
////#define GYRO_DURN_X             0x19
////#define GYRO_HIGHRATE_Y_SET     0x1a
////#define GYRO_DURN_Y             0x1b
////#define GYRO_HIGHRATE_Z_SET     0x1c
////#define GYRO_DURN_Z             0x1d
////#define GYRO_ANY_MOTION_THRES   0x1e
////#define GYRO_ANY_MOTION_SET     0x1f
////
////#endif      // BNO055_H
////
//
//
//
//
//
//
///***************************************************************************
//  This is a library for the BNO055 orientation sensor
//
//  Designed specifically to work with the Adafruit BNO055 Breakout.
//
//  Pick one up today in the adafruit shop!
//  ------> http://www.adafruit.com/products
//
//  These sensors use I2C to communicate, 2 pins are required to interface.
//
//  Adafruit invests time and resources providing this open source code,
//  please support Adafruit andopen-source hardware by purchasing products
//  from Adafruit!
//
//  Written by KTOWN for Adafruit Industries.
//
//  MIT license, all text above must be included in any redistribution
// ***************************************************************************/
///*
//* Copyright (C) 2008 The Android Open Source Project
//*
//* Licensed under the Apache License, Version 2.0 (the "License");
//* you may not use this file except in compliance with the License.
//* You may obtain a copy of the License at
//*
//* http://www.apache.org/licenses/LICENSE-2.0
//*
//* Unless required by applicable law or agreed to in writing, software< /span>
//* distributed under the License is distributed on an "AS IS" BASIS,
//* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//* See the License for the specific language governing permissions and
//* limitations under the License.
//*/
//
///* Update by K. Townsend (Adafruit Industries) for lighter typedefs, and
// * extended sensor support to include color, voltage and current */
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//#ifndef _ADAFRUIT_SENSOR_H
//#define _ADAFRUIT_SENSOR_H
//
//
///* Intentionally modeled after sensors.h in the Android API:
// * https://github.com/android/platform_hardware_libhardware/blob/master/include/hardware/sensors.h */
//
///* Constants */
//#define SENSORS_GRAVITY_EARTH             (9.80665F)              /**< Earth's gravity in m/s^2 */
//#define SENSORS_GRAVITY_MOON              (1.6F)                  /**< The moon's gravity in m/s^2 */
//#define SENSORS_GRAVITY_SUN               (275.0F)                /**< The sun's gravity in m/s^2 */
//#define SENSORS_GRAVITY_STANDARD          (SENSORS_GRAVITY_EARTH)
//#define SENSORS_MAGFIELD_EARTH_MAX        (60.0F)                 /**< Maximum magnetic field on Earth's surface */
//#define SENSORS_MAGFIELD_EARTH_MIN        (30.0F)                 /**< Minimum magnetic field on Earth's surface */
//#define SENSORS_PRESSURE_SEALEVELHPA      (1013.25F)              /**< Average sea level pressure is 1013.25 hPa */
//#define SENSORS_DPS_TO_RADS               (0.017453293F)          /**< Degrees/s to rad/s multiplier */
//#define SENSORS_GAUSS_TO_MICROTESLA       (100)                   /**< Gauss to micro-Tesla multiplier */
//
///** Sensor types */
//typedef enum
//{
//SENSOR_TYPE_ACCELEROMETER         = (1),   /**< Gravity + linear acceleration */
//SENSOR_TYPE_MAGNETIC_FIELD        = (2),
//SENSOR_TYPE_ORIENTATION           = (3),
//SENSOR_TYPE_GYROSCOPE             = (4),
//SENSOR_TYPE_LIGHT                 = (5),
//SENSOR_TYPE_PRESSURE              = (6),
//SENSOR_TYPE_PROXIMITY             = (8),
//SENSOR_TYPE_GRAVITY               = (9),
//SENSOR_TYPE_LINEAR_ACCELERATION   = (10),  /**< Acceleration not including gravity */
//SENSOR_TYPE_ROTATION_VECTOR       = (11),
//SENSOR_TYPE_RELATIVE_HUMIDITY     = (12),
//SENSOR_TYPE_AMBIENT_TEMPERATURE   = (13),
//SENSOR_TYPE_VOLTAGE               = (15),
//SENSOR_TYPE_CURRENT               = (16),
//SENSOR_TYPE_COLOR                 = (17)
//} sensors_type_t;
//
///** struct sensors_vec_s is used to return a vector in a common format. */
//typedef struct {
//union {
//float v[3];
//struct {
//float x;
//float y;
//float z;
//};
///* Orientation sensors */
//struct {
//float roll;    /**< Rotation around the longitudinal axis (the plane body, 'X axis'). Roll is positive and increasing when moving downward. -90°<=roll<=90° */
//float pitch;   /**< Rotation around the lateral axis (the wing span, 'Y axis'). Pitch is positive and increasing when moving upwards. -180°<=pitch<=180°) */
//float heading; /**< Angle between the longitudinal axis (the plane body) and magnetic north, measured clockwise when viewing from the top of the device. 0-359° */
//};
//};
//char status;
//unsigned char reserved[3];
//} sensors_vec_t;
//
///** struct sensors_color_s is used to return color data in a common format. */
//typedef struct {
//union {
//float c[3];
///* RGB color space */
//struct {
//float r;       /**< Red component */
//float g;       /**< Green component */
//float b;       /**< Blue component */
//};
//};
//unsigned int rgba;         /**< 24-bit RGBA value */
//} sensors_color_t;
//
///* Sensor event (36 bytes) */
///** struct sensor_event_s is used to provide a single sensor event in a common format. */
//typedef struct
//{
//int version;                          /**< must be sizeof(struct sensors_event_t) */
//int sensor_id;                        /**< unique sensor identifier */
//int type;                             /**< sensor type */
//int reserved0;                        /**< reserved */
//int timestamp;                        /**< time is in milliseconds */
//union
//{
//float           data[4];
//sensors_vec_t   acceleration;         /**< acceleration values are in meter per second per second (m/s^2) */
//sensors_vec_t   magnetic;             /**< magnetic vector values are in micro-Tesla (uT) */
//sensors_vec_t   orientation;          /**< orientation values are in degrees */
//sensors_vec_t   gyro;                 /**< gyroscope values are in rad/s */
//float           temperature;          /**< temperature is in degrees centigrade (Celsius) */
//float           distance;             /**< distance in centimeters */
//float           light;                /**< light in SI lux units */
//float           pressure;             /**< pressure in hectopascal (hPa) */
//float           relative_humidity;    /**< relative humidity in percent */
//float           current;              /**< current in milliamps (mA) */
//float           voltage;              /**< voltage in volts (V) */
//sensors_color_t color;                /**< color in RGB component values */
//};
//} sensors_event_t;
//
///* Sensor details (40 bytes) */
///** struct sensor_s is used to describe basic information about a specific sensor. */
//typedef struct
//{
//char     name[12];                        /**< sensor name */
//int  version;                         /**< version of the hardware + driver */
//int  sensor_id;                       /**< unique sensor identifier */
//int  type;                            /**< this sensor's type (ex. SENSOR_TYPE_LIGHT) */
//float    max_value;                       /**< maximum value of this sensor's value in SI units */
//float    min_value;                       /**< minimum value of this sensor's value in SI units */
//float    resolution;                      /**< smallest difference between two values reported by this sensor */
//int  min_delay;                       /**< min delay in microseconds between events. zero = not a constant rate */
//} sensor_t;
//
//class Adafruit_Sensor {
//public:
//// Constructor(s)
//Adafruit_Sensor() {}
//virtual ~Adafruit_Sensor() {}
//
//// These must be defined by the subclass
//virtual void enableAutoRange(bool enabled) {};
//virtual bool getEvent(sensors_event_t*) = 0;
//virtual void getSensor(sensor_t*) = 0;
//
//private:
//bool _autoRange;
//};
//
//#endif
//#ifndef __ADAFRUIT_BNO055_H__
//#define __ADAFRUIT_BNO055_H__
//#endif
//
//#ifndef BNO055_H
//#define BNO055_H
//
////#include "Adafruit_Sensor.h"
////#include "imumaths.h"
//#ifndef IMUMATH_H
//#define IMUMATH_H
//
//
////#include "vector.h"
//
//
///*
//    Inertial Measurement Unit Maths Library
//    Copyright (C) 2013-2014  Samuel Cowen
//    www.camelsoftware.com
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.
//*/
//
//#ifndef IMUMATH_VECTOR_HPP
//#define IMUMATH_VECTOR_HPP
//
//#include <stdlib.h>
//#include <string.h>
//#include <stdint.h>
//#include <math.h>
//
//
//namespace imu
//{
//
//template <uint8_t N> class Vector
//{
//public:
//Vector()
//{
//memset(p_vec, 0, sizeof(double)*N);
//}
//
//Vector(double a)
//{
//memset(p_vec, 0, sizeof(double)*N);
//p_vec[0] = a;
//}
//
//Vector(double a, double b)
//{
//memset(p_vec, 0, sizeof(double)*N);
//p_vec[0] = a;
//p_vec[1] = b;
//}
//
//Vector(double a, double b, double c)
//{
//memset(p_vec, 0, sizeof(double)*N);
//p_vec[0] = a;
//p_vec[1] = b;
//p_vec[2] = c;
//}
//
//Vector(double a, double b, double c, double d)
//{
//memset(p_vec, 0, sizeof(double)*N);
//p_vec[0] = a;
//p_vec[1] = b;
//p_vec[2] = c;
//p_vec[3] = d;
//}
//
//Vector(const Vector<N> &v)
//{
//for (int x = 0; x < N; x++)
//p_vec[x] = v.p_vec[x];
//}
//
//~Vector()
//{
//}
//
//uint8_t n() { return N; }
//
//double magnitude()
//{
//double res = 0;
//int i;
//for(i = 0; i < N; i++)
//res += (p_vec[i] * p_vec[i]);
//
//if(isnan(res))
//return 0;
//if((fabs(res-1)) >= 0.000001) // Avoid a sqrt if possible.
//return sqrt(res);
//return 1;
//}
//
//void normalize()
//{
//double mag = magnitude();
//if(abs(mag) <= 0.0001)
//return;
//
//int i;
//for(i = 0; i < N; i++)
//p_vec[i] = p_vec[i]/mag;
//}
//
//double dot(Vector v)
//{
//double ret = 0;
//int i;
//for(i = 0; i < N; i++)
//ret += p_vec[i] * v.p_vec[i];
//
//return ret;
//}
//
//Vector cross(Vector v)
//{
//Vector ret;
//
//// The cross product is only valid for vectors with 3 dimensions,
//// with the exception of higher dimensional stuff that is beyond the intended scope of this library
//if(N != 3)
//return ret;
//
//ret.p_vec[0] = (p_vec[1] * v.p_vec[2]) - (p_vec[2] * v.p_vec[1]);
//ret.p_vec[1] = (p_vec[2] * v.p_vec[0]) - (p_vec[0] * v.p_vec[2]);
//ret.p_vec[2] = (p_vec[0] * v.p_vec[1]) - (p_vec[1] * v.p_vec[0]);
//return ret;
//}
//
//Vector scale(double scalar) const
//{
//Vector ret;
//for(int i = 0; i < N; i++)
//ret.p_vec[i] = p_vec[i] * scalar;
//return ret;
//}
//
//Vector invert() const
//{
//Vector ret;
//for(int i = 0; i < N; i++)
//ret.p_vec[i] = -p_vec[i];
//return ret;
//}
//
//Vector operator = (Vector v)
//{
//for (int x = 0; x < N; x++ )
//p_vec[x] = v.p_vec[x];
//return *this;
//}
//
//double& operator [](int n)
//{
//return p_vec[n];
//}
//
//double operator [](int n) const
//{
//return p_vec[n];
//}
//
//double& operator ()(int n)
//{
//return p_vec[n];
//}
//
//double operator ()(int n) const
//{
//return p_vec[n];
//}
//
//Vector operator + (Vector v) const
//{
//Vector ret;
//for(int i = 0; i < N; i++)
//ret.p_vec[i] = p_vec[i] + v.p_vec[i];
//return ret;
//}
//
//Vector operator - (Vector v) const
//{
//Vector ret;
//for(int i = 0; i < N; i++)
//ret.p_vec[i] = p_vec[i] - v.p_vec[i];
//return ret;
//}
//
//Vector operator * (double scalar) const
//{
//return scale(scalar);
//}
//
//Vector operator / (double scalar) const
//{
//Vector ret;
//for(int i = 0; i < N; i++)
//ret.p_vec[i] = p_vec[i] / scalar;
//return ret;
//}
//
//void toDegrees()
//{
//for(int i = 0; i < N; i++)
//p_vec[i] *= 57.2957795131; //180/pi
//}
//
//void toRadians()
//{
//for(int i = 0; i < N; i++)
//p_vec[i] *= 0.01745329251;  //pi/180
//}
//
//double& x() { return p_vec[0]; }
//double& y() { return p_vec[1]; }
//double& z() { return p_vec[2]; }
//double x() const { return p_vec[0]; }
//double y() const { return p_vec[1]; }
//double z() const { return p_vec[2]; }
//
//
//private:
//double  p_vec[N];
//};
//
//
//};
//
//#endif
//
//
////#include "matrix.h"
///*
//    Inertial Measurement Unit Maths Library
//    Copyright (C) 2013-2014  Samuel Cowen
//    www.camelsoftware.com
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.
//*/
//
//#ifndef IMUMATH_MATRIX_HPP
//#define IMUMATH_MATRIX_HPP
//
//namespace imu
//{
//
//
//template <uint8_t N> class Matrix
//{
//public:
//Matrix()
//{
//int r = sizeof(double)*N;
//_cell = &_cell_data[0];
//memset(_cell, 0, r*r);
//}
//
//Matrix(const Matrix &v)
//{
//int r = sizeof(double)*N;
//_cell = &_cell_data[0];
//memset(_cell, 0, r*r);
//for (int x = 0; x < N; x++ )
//{
//for(int y = 0; y < N; y++)
//{
//_cell[x*N+y] = v._cell[x*N+y];
//}
//}
//}
//
//~Matrix()
//{
//}
//
//void operator = (Matrix m)
//{
//for(int x = 0; x < N; x++)
//{
//for(int y = 0; y < N; y++)
//{
//cell(x, y) = m.cell(x, y);
//}
//}
//}
//
//Vector<N> row_to_vector(int y)
//{
//Vector<N> ret;
//for(int i = 0; i < N; i++)
//{
//ret[i] = _cell[y*N+i];
//}
//return ret;
//}
//
//Vector<N> col_to_vector(int x)
//{
//Vector<N> ret;
//for(int i = 0; i < N; i++)
//{
//ret[i] = _cell[i*N+x];
//}
//return ret;
//}
//
//void vector_to_row(Vector<N> v, int row)
//{
//for(int i = 0; i < N; i++)
//{
//cell(row, i) = v(i);
//}
//}
//
//void vector_to_col(Vector<N> v, int col)
//{
//for(int i = 0; i < N; i++)
//{
//cell(i, col) = v(i);
//}
//}
//
////double& operator ()(int x, int y)
//double& operator()( int x,  int y) const
//{
//return _cell[x*N+y];
//}
//
//double& cell(int x, int y) const
//{
//return _cell[x*N+y];
//}
//
//
//Matrix operator + (Matrix m)
//{
//Matrix ret;
//for(int x = 0; x < N; x++)
//{
//for(int y = 0; y < N; y++)
//{
//ret._cell[x*N+y] = _cell[x*N+y] + m._cell[x*N+y];
//}
//}
//return ret;
//}
//
//Matrix operator - (Matrix m)
//{
//Matrix ret;
//for(int x = 0; x < N; x++)
//{
//for(int y = 0; y < N; y++)
//{
//ret._cell[x*N+y] = _cell[x*N+y] - m._cell[x*N+y];
//}
//}
//return ret;
//}
//
//Matrix operator * (double scalar)
//{
//Matrix ret;
//for(int x = 0; x < N; x++)
//{
//for(int y = 0; y < N; y++)
//{
//ret._cell[x*N+y] = _cell[x*N+y] * scalar;
//}
//}
//return ret;
//}
//
//Matrix operator * (Matrix m)
//{
//Matrix ret;
//for(int x = 0; x < N; x++)
//{
//for(int y = 0; y < N; y++)
//{
//Vector<N> row = row_to_vector(x);
//Vector<N> col = m.col_to_vector(y);
//ret.cell(x, y) = row.dot(col);
//}
//}
//return ret;
//}
//
//Matrix transpose()
//{
//Matrix ret;
//for(int x = 0; x < N; x++)
//{
//for(int y = 0; y < N; y++)
//{
//ret.cell(y, x) = cell(x, y);
//}
//}
//return ret;
//}
//
//Matrix<N-1> minor_matrix(int row, int col)
//{
//int colCount = 0, rowCount = 0;
//Matrix<N-1> ret;
//for(int i = 0; i < N; i++ )
//{
//if( i != row )
//{
//for(int j = 0; j < N; j++ )
//{
//if( j != col )
//{
//ret(rowCount, colCount) = cell(i, j);
//colCount++;
//}
//}
//rowCount++;
//}
//}
//return ret;
//}
//
//double determinant()
//{
//if(N == 1)
//return cell(0, 0);
//
//float det = 0.0;
//for(int i = 0; i < N; i++ )
//{
//Matrix<N-1> minor = minor_matrix(0, i);
//det += (i%2==1?-1.0:1.0) * cell(0, i) * minor.determinant();
//}
//return det;
//}
//
//Matrix invert()
//{
//Matrix ret;
//float det = determinant();
//
//for(int x = 0; x < N; x++)
//{
//for(int y = 0; y < N; y++)
//{
//Matrix<N-1> minor = minor_matrix(y, x);
//ret(x, y) = det*minor.determinant();
//if( (x+y)%2 == 1)
//ret(x, y) = -ret(x, y);
//}
//}
//return ret;
//}
//double trace() const {
//double tr = 0.0;
//for (int i = 0; i < N; ++i)
//tr += cell(i, i);
//return tr;
//}
//private:
//double* _cell;
//double  _cell_data[N*N];
//};
//
//
//};
//
//#endif
//
////#include "quaternion.h"
////  Inertial Measurement Unit Maths Library
////
////  Copyright 2013-2021 Sam Cowen <samuel.cowen@camelsoftware.com>
////  Bug fixes and cleanups by Gé Vissers (gvissers@gmail.com)
////
////  Permission is hereby granted, free of charge, to any person obtaining a
////  copy of this software and associated documentation files (the "Software"),
////  to deal in the Software without restriction, including without limitation
////  the rights to use, copy, modify, merge, publish, distribute, sublicense,
////  and/or sell copies of the Software, and to permit persons to whom the
////  Software is furnished to do so, subject to the following conditions:
////
////  The above copyright notice and this permission notice shall be included in
////  all copies or substantial portions of the Software.
////
////  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
////  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
////  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
////  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
////  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
////  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
////  DEALINGS IN THE SOFTWARE.
//
//#ifndef IMUMATH_QUATERNION_HPP
//#define IMUMATH_QUATERNION_HPP
////#include "matrix.h"
//
//namespace imu {
//
//class Quaternion {
//public:
//Quaternion() : _w(1.0), _x(0.0), _y(0.0), _z(0.0) {}
//
//Quaternion(double w, double x, double y, double z)
//: _w(w), _x(x), _y(y), _z(z) {}
//
//Quaternion(double w, Vector<3> vec)
//: _w(w), _x(vec.x()), _y(vec.y()), _z(vec.z()) {}
//
//double &w() { return _w; }
//double &x() { return _x; }
//double &y() { return _y; }
//double &z() { return _z; }
//
//double w() const { return _w; }
//double x() const { return _x; }
//double y() const { return _y; }
//double z() const { return _z; }
//
//double magnitude() const {
//return sqrt(_w * _w + _x * _x + _y * _y + _z * _z);
//}
//
//void normalize() {
//double mag = magnitude();
//*this = this->scale(1 / mag);
//}
//
//Quaternion conjugate() const { return Quaternion(_w, -_x, -_y, -_z); }
//
//void fromAxisAngle(const Vector<3> &axis, double theta) {
//_w = cos(theta / 2);
//// only need to calculate sine of half theta once
//double sht = sin(theta / 2);
//_x = axis.x() * sht;
//_y = axis.y() * sht;
//_z = axis.z() * sht;
//}
//
//void fromMatrix(const Matrix<3> &m) {
//double tr = m.trace();
//
//double S;
//if (tr > 0) {
//S = sqrt(tr + 1.0) * 2;
//_w = 0.25 * S;
//_x = (m(2, 1) - m(1, 2)) / S;
//_y = (m(0, 2) - m(2, 0)) / S;
//_z = (m(1, 0) - m(0, 1)) / S;
//} else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2)) {
//S = sqrt(1.0 + m(0, 0) - m(1, 1) - m(2, 2)) * 2;
//_w = (m(2, 1) - m(1, 2)) / S;
//_x = 0.25 * S;
//_y = (m(0, 1) + m(1, 0)) / S;
//_z = (m(0, 2) + m(2, 0)) / S;
//} else if (m(1, 1) > m(2, 2)) {
//S = sqrt(1.0 + m(1, 1) - m(0, 0) - m(2, 2)) * 2;
//_w = (m(0, 2) - m(2, 0)) / S;
//_x = (m(0, 1) + m(1, 0)) / S;
//_y = 0.25 * S;
//_z = (m(1, 2) + m(2, 1)) / S;
//} else {
//S = sqrt(1.0 + m(2, 2) - m(0, 0) - m(1, 1)) * 2;
//_w = (m(1, 0) - m(0, 1)) / S;
//_x = (m(0, 2) + m(2, 0)) / S;
//_y = (m(1, 2) + m(2, 1)) / S;
//_z = 0.25 * S;
//}
//}
//
//void toAxisAngle(Vector<3> &axis, double &angle) const {
//double sqw = sqrt(1 - _w * _w);
//if (sqw == 0) // it's a singularity and divide by zero, avoid
//return;
//
//angle = 2 * acos(_w);
//axis.x() = _x / sqw;
//axis.y() = _y / sqw;
//axis.z() = _z / sqw;
//}
//
//Matrix<3> toMatrix() const {
//Matrix<3> ret;
//ret.cell(0, 0) = 1 - 2 * _y * _y - 2 * _z * _z;
//ret.cell(0, 1) = 2 * _x * _y - 2 * _w * _z;
//ret.cell(0, 2) = 2 * _x * _z + 2 * _w * _y;
//
//ret.cell(1, 0) = 2 * _x * _y + 2 * _w * _z;
//ret.cell(1, 1) = 1 - 2 * _x * _x - 2 * _z * _z;
//ret.cell(1, 2) = 2 * _y * _z - 2 * _w * _x;
//
//ret.cell(2, 0) = 2 * _x * _z - 2 * _w * _y;
//ret.cell(2, 1) = 2 * _y * _z + 2 * _w * _x;
//ret.cell(2, 2) = 1 - 2 * _x * _x - 2 * _y * _y;
//return ret;
//}
//
//// Returns euler angles that represent the quaternion.  Angles are
//// returned in rotation order and right-handed about the specified
//// axes:
////
////   v[0] is applied 1st about z (ie, roll)
////   v[1] is applied 2nd about y (ie, pitch)
////   v[2] is applied 3rd about x (ie, yaw)
////
//// Note that this means result.x() is not a rotation about x;
//// similarly for result.z().
////
//Vector<3> toEuler() const {
//Vector<3> ret;
//double sqw = _w * _w;
//double sqx = _x * _x;
//double sqy = _y * _y;
//double sqz = _z * _z;
//
//ret.x() = atan2(2.0 * (_x * _y + _z * _w), (sqx - sqy - sqz + sqw));
//ret.y() = asin(-2.0 * (_x * _z - _y * _w) / (sqx + sqy + sqz + sqw));
//ret.z() = atan2(2.0 * (_y * _z + _x * _w), (-sqx - sqy + sqz + sqw));
//
//return ret;
//}
//
//Vector<3> toAngularVelocity(double dt) const {
//Vector<3> ret;
//Quaternion one(1.0, 0.0, 0.0, 0.0);
//Quaternion delta = one - *this;
//Quaternion r = (delta / dt);
//r = r * 2;
//r = r * one;
//
//ret.x() = r.x();
//ret.y() = r.y();
//ret.z() = r.z();
//return ret;
//}
//
//Vector<3> rotateVector(const Vector<2> &v) const {
//return rotateVector(Vector<3>(v.x(), v.y()));
//}
//
//Vector<3> rotateVector(const Vector<3> &v) const {
//Vector<3> qv(_x, _y, _z);
//Vector<3> t = qv.cross(v) * 2.0;
//return v + t * _w + qv.cross(t);
//}
//
//Quaternion operator*(const Quaternion &q) const {
//return Quaternion(_w * q._w - _x * q._x - _y * q._y - _z * q._z,
//_w * q._x + _x * q._w + _y * q._z - _z * q._y,
//_w * q._y - _x * q._z + _y * q._w + _z * q._x,
//_w * q._z + _x * q._y - _y * q._x + _z * q._w);
//}
//
//Quaternion operator+(const Quaternion &q) const {
//return Quaternion(_w + q._w, _x + q._x, _y + q._y, _z + q._z);
//}
//
//Quaternion operator-(const Quaternion &q) const {
//return Quaternion(_w - q._w, _x - q._x, _y - q._y, _z - q._z);
//}
//
//Quaternion operator/(double scalar) const {
//return Quaternion(_w / scalar, _x / scalar, _y / scalar, _z / scalar);
//}
//
//Quaternion operator*(double scalar) const { return scale(scalar); }
//
//Quaternion scale(double scalar) const {
//return Quaternion(_w * scalar, _x * scalar, _y * scalar, _z * scalar);
//}
//
//private:
//double _w, _x, _y, _z;
//};
//
//} // namespace imu
//
//#endif
//
//#endif
////#include "mbed.h"
//
//#define BNO055_ADDRESS_A (0x28)
//#define BNO055_ADDRESS_B (0x29)
//#define BNO055_ID        (0xA0)
//
//class Adafruit_BNO055 : public Adafruit_Sensor
//{
//public:
//typedef enum
//{
///* Page id register definition */
//BNO055_PAGE_ID_ADDR                                     = 0X07,
//
///* PAGE0 REGISTER DEFINITION START*/
//BNO055_CHIP_ID_ADDR                                     = 0x00,
//BNO055_ACCEL_REV_ID_ADDR                                = 0x01,
//BNO055_MAG_REV_ID_ADDR                                  = 0x02,
//BNO055_GYRO_REV_ID_ADDR                                 = 0x03,
//BNO055_SW_REV_ID_LSB_ADDR                               = 0x04,
//BNO055_SW_REV_ID_MSB_ADDR                               = 0x05,
//BNO055_BL_REV_ID_ADDR                                   = 0X06,
//
///* Accel data register */
//BNO055_ACCEL_DATA_X_LSB_ADDR                            = 0X08,
//BNO055_ACCEL_DATA_X_MSB_ADDR                            = 0X09,
//BNO055_ACCEL_DATA_Y_LSB_ADDR                            = 0X0A,
//BNO055_ACCEL_DATA_Y_MSB_ADDR                            = 0X0B,
//BNO055_ACCEL_DATA_Z_LSB_ADDR                            = 0X0C,
//BNO055_ACCEL_DATA_Z_MSB_ADDR                            = 0X0D,
//
///* Mag data register */
//BNO055_MAG_DATA_X_LSB_ADDR                              = 0X0E,
//BNO055_MAG_DATA_X_MSB_ADDR                              = 0X0F,
//BNO055_MAG_DATA_Y_LSB_ADDR                              = 0X10,
//BNO055_MAG_DATA_Y_MSB_ADDR                              = 0X11,
//BNO055_MAG_DATA_Z_LSB_ADDR                              = 0X12,
//BNO055_MAG_DATA_Z_MSB_ADDR                              = 0X13,
//
///* Gyro data registers */
//BNO055_GYRO_DATA_X_LSB_ADDR                             = 0X14,
//BNO055_GYRO_DATA_X_MSB_ADDR                             = 0X15,
//BNO055_GYRO_DATA_Y_LSB_ADDR                             = 0X16,
//BNO055_GYRO_DATA_Y_MSB_ADDR                             = 0X17,
//BNO055_GYRO_DATA_Z_LSB_ADDR                             = 0X18,
//BNO055_GYRO_DATA_Z_MSB_ADDR                             = 0X19,
//
///* Euler data registers */
//BNO055_EULER_H_LSB_ADDR                                 = 0X1A,
//BNO055_EULER_H_MSB_ADDR                                 = 0X1B,
//BNO055_EULER_R_LSB_ADDR                                 = 0X1C,
//BNO055_EULER_R_MSB_ADDR                                 = 0X1D,
//BNO055_EULER_P_LSB_ADDR                                 = 0X1E,
//BNO055_EULER_P_MSB_ADDR                                 = 0X1F,
//
///* Quaternion data registers */
//BNO055_QUATERNION_DATA_W_LSB_ADDR                       = 0X20,
//BNO055_QUATERNION_DATA_W_MSB_ADDR                       = 0X21,
//BNO055_QUATERNION_DATA_X_LSB_ADDR                       = 0X22,
//BNO055_QUATERNION_DATA_X_MSB_ADDR                       = 0X23,
//BNO055_QUATERNION_DATA_Y_LSB_ADDR                       = 0X24,
//BNO055_QUATERNION_DATA_Y_MSB_ADDR                       = 0X25,
//BNO055_QUATERNION_DATA_Z_LSB_ADDR                       = 0X26,
//BNO055_QUATERNION_DATA_Z_MSB_ADDR                       = 0X27,
//
///* Linear acceleration data registers */
//BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR                     = 0X28,
//BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR                     = 0X29,
//BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR                     = 0X2A,
//BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR                     = 0X2B,
//BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR                     = 0X2C,
//BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR                     = 0X2D,
//
///* Gravity data registers */
//BNO055_GRAVITY_DATA_X_LSB_ADDR                          = 0X2E,
//BNO055_GRAVITY_DATA_X_MSB_ADDR                          = 0X2F,
//BNO055_GRAVITY_DATA_Y_LSB_ADDR                          = 0X30,
//BNO055_GRAVITY_DATA_Y_MSB_ADDR                          = 0X31,
//BNO055_GRAVITY_DATA_Z_LSB_ADDR                          = 0X32,
//BNO055_GRAVITY_DATA_Z_MSB_ADDR                          = 0X33,
//
///* Temperature data register */
//BNO055_TEMP_ADDR                                        = 0X34,
//
///* Status registers */
//BNO055_CALIB_STAT_ADDR                                  = 0X35,
//BNO055_SELFTEST_RESULT_ADDR                             = 0X36,
//BNO055_INTR_STAT_ADDR                                   = 0X37,
//
//BNO055_SYS_CLK_STAT_ADDR                                = 0X38,
//BNO055_SYS_STAT_ADDR                                    = 0X39,
//BNO055_SYS_ERR_ADDR                                     = 0X3A,
//
///* Unit selection register */
//BNO055_UNIT_SEL_ADDR                                    = 0X3B,
//BNO055_DATA_SELECT_ADDR                                 = 0X3C,
//
///* Mode registers */
//BNO055_OPR_MODE_ADDR                                    = 0X3D,
//BNO055_PWR_MODE_ADDR                                    = 0X3E,
//
//BNO055_SYS_TRIGGER_ADDR                                 = 0X3F,
//BNO055_TEMP_SOURCE_ADDR                                 = 0X40,
//
///* Axis remap registers */
//BNO055_AXIS_MAP_CONFIG_ADDR                             = 0X41,
//BNO055_AXIS_MAP_SIGN_ADDR                               = 0X42,
//
///* SIC registers */
//BNO055_SIC_MATRIX_0_LSB_ADDR                            = 0X43,
//BNO055_SIC_MATRIX_0_MSB_ADDR                            = 0X44,
//BNO055_SIC_MATRIX_1_LSB_ADDR                            = 0X45,
//BNO055_SIC_MATRIX_1_MSB_ADDR                            = 0X46,
//BNO055_SIC_MATRIX_2_LSB_ADDR                            = 0X47,
//BNO055_SIC_MATRIX_2_MSB_ADDR                            = 0X48,
//BNO055_SIC_MATRIX_3_LSB_ADDR                            = 0X49,
//BNO055_SIC_MATRIX_3_MSB_ADDR                            = 0X4A,
//BNO055_SIC_MATRIX_4_LSB_ADDR                            = 0X4B,
//BNO055_SIC_MATRIX_4_MSB_ADDR                            = 0X4C,
//BNO055_SIC_MATRIX_5_LSB_ADDR                            = 0X4D,
//BNO055_SIC_MATRIX_5_MSB_ADDR                            = 0X4E,
//BNO055_SIC_MATRIX_6_LSB_ADDR                            = 0X4F,
//BNO055_SIC_MATRIX_6_MSB_ADDR                            = 0X50,
//BNO055_SIC_MATRIX_7_LSB_ADDR                            = 0X51,
//BNO055_SIC_MATRIX_7_MSB_ADDR                            = 0X52,
//BNO055_SIC_MATRIX_8_LSB_ADDR                            = 0X53,
//BNO055_SIC_MATRIX_8_MSB_ADDR                            = 0X54,
//
///* Accelerometer Offset registers */
//ACCEL_OFFSET_X_LSB_ADDR                                 = 0X55,
//ACCEL_OFFSET_X_MSB_ADDR                                 = 0X56,
//ACCEL_OFFSET_Y_LSB_ADDR                                 = 0X57,
//ACCEL_OFFSET_Y_MSB_ADDR                                 = 0X58,
//ACCEL_OFFSET_Z_LSB_ADDR                                 = 0X59,
//ACCEL_OFFSET_Z_MSB_ADDR                                 = 0X5A,
//
///* Magnetometer Offset registers */
//MAG_OFFSET_X_LSB_ADDR                                   = 0X5B,
//MAG_OFFSET_X_MSB_ADDR                                   = 0X5C,
//MAG_OFFSET_Y_LSB_ADDR                                   = 0X5D,
//MAG_OFFSET_Y_MSB_ADDR                                   = 0X5E,
//MAG_OFFSET_Z_LSB_ADDR                                   = 0X5F,
//MAG_OFFSET_Z_MSB_ADDR                                   = 0X60,
//
///* Gyroscope Offset register s*/
//GYRO_OFFSET_X_LSB_ADDR                                  = 0X61,
//GYRO_OFFSET_X_MSB_ADDR                                  = 0X62,
//GYRO_OFFSET_Y_LSB_ADDR                                  = 0X63,
//GYRO_OFFSET_Y_MSB_ADDR                                  = 0X64,
//GYRO_OFFSET_Z_LSB_ADDR                                  = 0X65,
//GYRO_OFFSET_Z_MSB_ADDR                                  = 0X66,
//
///* Radius registers */
//ACCEL_RADIUS_LSB_ADDR                                   = 0X67,
//ACCEL_RADIUS_MSB_ADDR                                   = 0X68,
//MAG_RADIUS_LSB_ADDR                                     = 0X69,
//MAG_RADIUS_MSB_ADDR                                     = 0X6A
//} adafruit_bno055_reg_t;
//
//typedef enum
//{
//POWER_MODE_NORMAL                                       = 0X00,
//POWER_MODE_LOWPOWER                                     = 0X01,
//POWER_MODE_SUSPEND                                      = 0X02
//} adafruit_bno055_powermode_t;
//
//typedef enum
//{
///* Operation mode settings*/
//OPERATION_MODE_CONFIG                                   = 0X00,
//OPERATION_MODE_ACCONLY                                  = 0X01,
//OPERATION_MODE_MAGONLY                                  = 0X02,
//OPERATION_MODE_GYRONLY                                  = 0X03,
//OPERATION_MODE_ACCMAG                                   = 0X04,
//OPERATION_MODE_ACCGYRO                                  = 0X05,
//OPERATION_MODE_MAGGYRO                                  = 0X06,
//OPERATION_MODE_AMG                                      = 0X07,
//OPERATION_MODE_IMUPLUS                                  = 0X08,
//OPERATION_MODE_COMPASS                                  = 0X09,
//OPERATION_MODE_M4G                                      = 0X0A,
//OPERATION_MODE_NDOF_FMC_OFF                             = 0X0B,
//OPERATION_MODE_NDOF                                     = 0X0C
//} adafruit_bno055_opmode_t;
//
//typedef struct
//{
//uint8_t  accel_rev;
//uint8_t  mag_rev;
//uint8_t  gyro_rev;
//uint16_t sw_rev;
//uint8_t  bl_rev;
//} adafruit_bno055_rev_info_t;
//
//typedef enum
//{
//VECTOR_ACCELEROMETER = BNO055_ACCEL_DATA_X_LSB_ADDR,
//VECTOR_MAGNETOMETER  = BNO055_MAG_DATA_X_LSB_ADDR,
//VECTOR_GYROSCOPE     = BNO055_GYRO_DATA_X_LSB_ADDR,
//VECTOR_EULER         = BNO055_EULER_H_LSB_ADDR,
//VECTOR_LINEARACCEL   = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR,
//VECTOR_GRAVITY       = BNO055_GRAVITY_DATA_X_LSB_ADDR
//} adafruit_vector_type_t;
//
//Adafruit_BNO055 ( int32_t sensorID = -1, uint8_t address = BNO055_ADDRESS_A, I2C* i2c_ptr = 0 );
//
//bool  begin               ( adafruit_bno055_opmode_t mode = OPERATION_MODE_NDOF );
//void  setMode             ( adafruit_bno055_opmode_t mode );
//void  getRevInfo          ( adafruit_bno055_rev_info_t* );
//void  displayRevInfo      ( void );
//void  setExtCrystalUse    ( bool usextal );
//void  getSystemStatus     ( uint8_t *system_status,
//uint8_t *self_test_result,
//uint8_t *system_error);
//void  displaySystemStatus ( void );
//void  getCalibration      ( uint8_t* system, uint8_t* gyro, uint8_t* accel, uint8_t* mag);
//
//imu::Vector<3>  getVector ( adafruit_vector_type_t vector_type );
//imu::Quaternion getQuat   ( void );
//int8_t          getTemp   ( void );
//
///* Adafruit_Sensor implementation */
//bool  getEvent  ( sensors_event_t* );
//void  getSensor ( sensor_t* );
//
//private:
//char  read8   ( adafruit_bno055_reg_t );
//bool  readLen ( adafruit_bno055_reg_t, char* buffer, int len );
//bool  write8  ( adafruit_bno055_reg_t, char value );
//
//uint8_t _address;
//int32_t _sensorID;
//adafruit_bno055_opmode_t _mode;
//I2C* i2c;
//};
//
//#endif