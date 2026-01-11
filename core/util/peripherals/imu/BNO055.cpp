/*
 * mbed library program
 *  BNO055 Intelligent 9-axis absolute orientation sensor
 *  by Bosch Sensortec
 *
 * Copyright (c) 2015,'17 Kenji Arai / JH1PJL
 *  http://www.page.sannet.ne.jp/kenjia/index.html
 *  http://mbed.org/users/kenjiArai/
 *      Created: March     30th, 2015
 *      Revised: August    23rd, 2017
 */

/*
 * This library has been modified. Original can be found here: https://os.mbed.com/users/AlexanderLill/code/BNO055_fusion/
 */

#include "mbed.h"
#include "BNO055.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "UnusedParameter"
#pragma ide diagnostic ignored "UnusedLocalVariable"

BNO055::BNO055():
    _i2c_p(new I2C(PB_9, PB_8)), _i2c(*_i2c_p), _res(PA_8)
{
}

BNO055::BNO055 (PinName p_sda, PinName p_scl, PinName p_reset, uint8_t addr, uint8_t mode):
    _i2c_p(new I2C(p_sda, p_scl)), _i2c(*_i2c_p), _res(p_reset)
{
    chip_addr = addr;
    chip_mode = mode;
    initialize_reset_pin();
    initialize ();
}

BNO055::BNO055 (PinName p_sda, PinName p_scl, PinName p_reset) :
    _i2c_p(new I2C(p_sda, p_scl)), _i2c(*_i2c_p), _res(p_reset)
{
    chip_addr = BNO055_G_CHIP_ADDR;
    chip_mode = MODE_NDOF;
    initialize_reset_pin();
    initialize ();
}

BNO055::BNO055 (I2C& p_i2c, PinName p_reset, uint8_t addr, uint8_t mode) :
    _i2c(p_i2c), _res(p_reset)
{
    chip_addr = addr;
    chip_mode = mode;
    initialize_reset_pin();
    initialize ();
}

BNO055::BNO055 (I2C& p_i2c, PinName p_reset, uint8_t mode) :
    _i2c(p_i2c), _res(p_reset)
{
    chip_addr = BNO055_G_CHIP_ADDR;
    chip_mode = mode;
    initialize_reset_pin();
    initialize ();
}

/////////////// Read data & normalize /////////////////////
void BNO055::get_euler_angles(BNO055_EULER_TypeDef *result)
{
    int16_t h,p,r;

    select_page(0);
    dt[0] = BNO055_EULER_H_LSB;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 6, false);
    h = dt[1] << 8 | dt[0];
    r = dt[3] << 8 | dt[2];
    p = dt[5] << 8 | dt[4];

    if (use_degrees()) {
        result->h = (double)h / 16;
        result->p = (double)p / 16;
        result->r = (double)r / 16;
    } else {
        result->h = (double)h / 900;
        result->p = (double)p / 900;
        result->r = (double)r / 900;
    }
}

void BNO055::get_quaternion(BNO055_QUATERNION_TypeDef *result)
{
    if (cantReadDataCount > 0 && cantReadDataCount < 50) {
        cantReadDataCount++;
        return;
    } else if (cantReadDataCount >= 50) {
        cantReadDataCount = 1;
    }
    int16_t w,x,y,z;

    select_page(0);
    dt[0] = BNO055_QUATERNION_W_LSB;
    int writeResult = _i2c.write(chip_addr, dt, 1, true);
//    printf("IMU Write result: %i\n", writeResult);
    if (!writeResult)  {
        if (cantReadDataCount > 0) {
            reset();
            cantReadDataCount = 0;
        }
        _i2c.read(chip_addr, dt, 8, false);
        w = dt[1] << 8 | dt[0];
        x = dt[3] << 8 | dt[2];
        y = dt[5] << 8 | dt[4];
        z = dt[7] << 8 | dt[6];

        result->w = (double)w / 16384.0f;
        result->x = (double)x / 16384.0f;
        result->y = (double)y / 16384.0f;
        result->z = (double)z / 16384.0f;
    } else {
        cantReadDataCount++;
    }
}

void BNO055::get_angular_position_quat(IMU::EulerAngles *result){

    BNO055_QUATERNION_TypeDef q;
    get_quaternion(&q);

//    result -> roll  = (float)atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)) * 180 / PI;
//    result -> pitch = (float)asin(2 * q.w * q.y - q.x * q.z) * 180 / PI;
//    result -> yaw   = (float)atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) * 180 / PI;
    float roll  = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)) * 180 / PI;
    // imuAngles.roll  = result->roll;
    float pitch = asin(2 * q.w * q.y - q.x * q.z) * 180 / PI;
    // imuAngles.pitch = result->pitch;
    float yaw   = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) * 180 / PI;
    // imuAngles.yaw   = result->yaw;
    mutex_.lock();
    memcpy(&result->roll, &roll, sizeof(float));
    memcpy(&result->pitch, &pitch, sizeof(float));
    memcpy(&result->yaw, &yaw, sizeof(float));
    mutex_.unlock();
}

IMU::EulerAngles BNO055::read()
{
    get_angular_position_quat(&imuAngles);
    return imuAngles;
}

IMU::EulerAngles BNO055::getImuAngles()
{
    return imuAngles;
}

void BNO055::get_linear_accel(BNO055_VECTOR_TypeDef *result)
{
    int16_t x,y,z;

    select_page(0);
    dt[0] = BNO055_LINEAR_ACC_X_LSB;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 6, false);
    x = dt[1] << 8 | dt[0];
    y = dt[3] << 8 | dt[2];
    z = dt[5] << 8 | dt[4];

    if (use_mss()) {
        result->x = (double)x / 100;
        result->y = (double)y / 100;
        result->z = (double)z / 100;
    } else {
        result->x = (double)x;
        result->y = (double)y;
        result->z = (double)z;
    }
}

void BNO055::get_gravity(BNO055_VECTOR_TypeDef *result)
{
    int16_t x,y,z;

    select_page(0);
    dt[0] = BNO055_GRAVITY_X_LSB;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 6, false);
    x = dt[1] << 8 | dt[0];
    y = dt[3] << 8 | dt[2];
    z = dt[5] << 8 | dt[4];

    if (use_mss()) {
        result->x = (double)x / 100;
        result->y = (double)y / 100;
        result->z = (double)z / 100;
    } else {
        result->x = (double)x;
        result->y = (double)y;
        result->z = (double)z;
    }
}

void BNO055::get_mag(BNO055_VECTOR_TypeDef *result)
{
    int16_t x,y,z;

    select_page(0);
    dt[0] = BNO055_MAG_X_LSB;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 6, false);
    x = dt[1] << 8 | dt[0];
    y = dt[3] << 8 | dt[2];
    z = dt[5] << 8 | dt[4];

    result->x = (double)x;
    result->y = (double)y;
    result->z = (double)z;
}

void BNO055::get_accel(BNO055_VECTOR_TypeDef *result)
{
    int16_t x,y,z;

    select_page(0);
    dt[0] = BNO055_ACC_X_LSB;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 6, false);
    x = dt[1] << 8 | dt[0];
    y = dt[3] << 8 | dt[2];
    z = dt[5] << 8 | dt[4];

    if (use_mss()) {
        result->x = (double)x / 100;
        result->y = (double)y / 100;
        result->z = (double)z / 100;
    } else {
        result->x = (double)x;
        result->y = (double)y;
        result->z = (double)z;
    }
}

void BNO055::get_gyro(BNO055_VECTOR_TypeDef *result)
{
    int16_t x,y,z;

    select_page(0);
    dt[0] = BNO055_GYR_X_LSB;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 6, false);
    x = dt[1] << 8 | dt[0];
    y = dt[3] << 8 | dt[2];
    z = dt[5] << 8 | dt[4];
    if (use_dps()) {
        result->x = (double)x / 16;
        result->y = (double)y / 16;
        result->z = (double)z / 16;
    } else {
        result->x = (double)x / 900;
        result->y = (double)y / 900;
        result->z = (double)z / 900;
    }
}

void BNO055::get_chip_temperature(BNO055_TEMPERATURE_TypeDef *result)
{
    select_page(0);

    uint8_t use_celsius_result = use_celsius();

    dt[0] = BNO055_TEMP_SOURCE;
    dt[1] = 0;
    _i2c.write(chip_addr, dt, 2, false);
    ThisThread::sleep_for(1ms); // Do I need to wait?
    dt[0] = BNO055_TEMP;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 1, false);

    if (use_celsius_result)
        result -> acc_chip = (int8_t)dt[0];

    else
        result -> acc_chip = (int8_t)dt[0] * 2;


    dt[0] = BNO055_TEMP_SOURCE;
    dt[1] = 1;
    _i2c.write(chip_addr, dt, 2, false);
    ThisThread::sleep_for(1ms); // Do I need to wait?
    dt[0] = BNO055_TEMP;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 1, false);

    if (use_celsius_result)
        result -> gyr_chip = (int8_t)dt[0];

    else
        result -> gyr_chip = (int8_t)dt[0] * 2;
}

/////////////// Initialize ////////////////////////////////
void BNO055::initialize (void)
{
    _i2c.frequency(400000);

    page_flag = 0xff;
    select_page(0);

    // Check Acc & Mag & Gyro are available of not
    get_id();

    // Set initial data
    set_initial_dt_to_regs();

    // Unit selection
    unit_selection();

    // Set fusion mode
    change_fusion_mode(chip_mode);
}

void BNO055::initialize_reset_pin(void)
{
    _res = 1;
    ThisThread::sleep_for(700ms); // Need to wait at least 650mS
}

void BNO055::unit_selection(void)
{
    select_page(0);
    dt[0] = BNO055_UNIT_SEL;
    dt[1] = UNIT_ORI_WIN + UNIT_ACC_MSS + UNIT_GYR_DPS + UNIT_EULER_DEG + UNIT_TEMP_C;
    _i2c.write(chip_addr, dt, 2, false);
}

bool BNO055::use_degrees()
{
    if (unit_flag_is_set(UNIT_EULER_RAD)) {
        return false;
    }
    return true;
}

bool BNO055::use_mss()
{
    if (unit_flag_is_set(UNIT_ACC_MG)) {
        return false;
    }
    return true;
}

bool BNO055::use_dps()
{
    if (unit_flag_is_set(UNIT_GYR_RPS)) {
        return false;
    }
    return true;
}

bool BNO055::use_celsius()
{
    if (unit_flag_is_set(UNIT_TEMP_F)) {
        return false;
    }
    return true;
}

bool BNO055::unit_flag_is_set(uint8_t flag)
{
    select_page(0);
    dt[0] = BNO055_UNIT_SEL;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 1, false);
    if (dt[0] & flag) {
        return true;
    }
    return false;
}

uint8_t BNO055::select_page(uint8_t page)
{
    if (page != page_flag){
        dt[0] = BNO055_PAGE_ID;
        if (page == 1) {
            dt[1] = 1;  // select page 1
        } else {
            dt[1] = 0;  // select page 0
        }
        _i2c.write(chip_addr, dt, 2, false);
        dt[0] = BNO055_PAGE_ID;
        _i2c.write(chip_addr, dt, 1, true);
        _i2c.read(chip_addr, dt, 1, false);
        page_flag = dt[0];
    }
    return page_flag;
}

uint8_t BNO055::reset(void)
{
     _res = 0;
     ThisThread::sleep_for(1ms);   // Reset 1mS
     _res = 1;
     ThisThread::sleep_for(700ms); // Need to wait at least 650mS
#if defined(TARGET_STM32L152RE)
    _i2c.frequency(400000);
#else
    _i2c.frequency(400000);
#endif
    _i2c.stop();
    page_flag = 0xff;
    select_page(0);
    get_id();
    if (chip_id != I_AM_BNO055_CHIP){
        return 1;
    } else {
        initialize();
        return 0;
    }
}

////// Set initialize data to related registers ///////////
void BNO055::set_initial_dt_to_regs(void)
{
    // select_page(0);
    // current setting is only used default values
}

/////////////// Check Who am I? ///////////////////////////
void BNO055::get_id(void)
{
    select_page(0);
    // ID
    dt[0] = BNO055_CHIP_ID;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 7, false);
    chip_id = dt[0];
    if (chip_id == I_AM_BNO055_CHIP) {
        ready_flag = 1;
    } else {
        ready_flag = 0;
    }
    acc_id = dt[1];
    if (acc_id == I_AM_BNO055_ACC) {
        ready_flag |= 2;
    }
    mag_id = dt[2];
    if (mag_id == I_AM_BNO055_MAG) {
        ready_flag |= 4;
    }
    gyr_id = dt[3];
    if (mag_id == I_AM_BNO055_MAG) {
        ready_flag |= 8;
    }
    bootldr_rev_id = dt[5]<< 8 | dt[4];
    sw_rev_id = dt[6];
}

void BNO055::read_id_inf(BNO055_ID_INF_TypeDef *id)
{
    id->chip_id = chip_id;
    id->acc_id = acc_id;
    id->mag_id = mag_id;
    id->gyr_id = gyr_id;
    id->bootldr_rev_id = bootldr_rev_id;
    id->sw_rev_id = sw_rev_id;
}

/////////////// Check chip ready or not  //////////////////
uint8_t BNO055::chip_ready(void)
{
    if (ready_flag == 0x0f) {
        return 1;
    }
    return 0;
}

/////////////// Calibrate IMU  ////////////////////
void BNO055::calibrate()
{
    uint8_t d;
    Timer t;
    BNO055_VECTOR_TypeDef      gravity;

    printf("------ Enter BNO055 Manual Calibration Mode ------\r\n");

    //---------- Gyroscope Caliblation -----------------------------------------
    // (a) Place the device in a single stable position for a period of
    //     few seconds to allow the gyroscope to calibrate

    printf("Step1) Please wait few seconds\r\n");
    t.start();

    while (t.elapsed_time().count() / 1000000 < 10) {
        d = read_calib_status();
        printf("Calb dat = 0x%x target  = 0x30(at least)\r\n", d);
        if ((d & 0x30) == 0x30) {
            break;
        }
        ThisThread::sleep_for(1s);
    }

    printf("-> Step1) is done\r\n\r\n");

    //---------- Magnetometer Caliblation --------------------------------------
    // (a) Make some random movements (for example: writing the number ‘8’
    //     on air) until the CALIB_STAT register indicates fully calibrated.
    // (b) It takes more calibration movements to get the magnetometer
    //     calibrated than in the NDOF mode.

    printf("Step2) random moving (try to change the BNO055 axis)\r\n");
    t.start();

    while (t.elapsed_time().count() / 1000000 < 30) {
        d = read_calib_status();
        printf("Calb dat = 0x%x target  = 0x33(at least)\r\n", d);
        if ((d & 0x03) == 0x03) {
            break;
        }
        ThisThread::sleep_for(1s);
    }

    printf("-> Step2) is done\r\n\r\n");

    //---------- Magnetometer Caliblation --------------------------------------
    // a) Place the device in 6 different stable positions for a period of
    //    few seconds to allow the accelerometer to calibrate.
    // b) Make sure that there is slow movement between 2 stable positions
    //    The 6 stable positions could be in any direction, but make sure that
    //    the device is lying at least once perpendicular to the x, y and z axis

    printf("Step3) Change rotation each X,Y,Z axis KEEP SLOWLY!!");
    printf(" Each 90deg stay a 5 sec and set at least 6 position.\r\n");
    printf(" e.g. (1)ACC:X0,Y0,Z-9,(2)ACC:X9,Y0,Z0,(3)ACC:X0,Y0,Z9,");
    printf("(4)ACC:X-9,Y0,Z0,(5)ACC:X0,Y-9,Z0,(6)ACC:X0,Y9,Z0,\r\n");
    printf(" If you will give up, hit any key.\r\n");
    t.stop();


    while (true) {
        d = read_calib_status();
        get_gravity(&gravity);
        printf(
                "Calb dat = 0x%x target  = 0xff ACC:X %d, Y %d, Z %d\r\n",
                d, (int)gravity.x, (int)gravity.y, (int)gravity.z
        );
        if (d == 0xff)
            break;

        ThisThread::sleep_for(1s);
    }
    if (read_calib_status() == 0xff)
        printf("-> All of Calibration steps are done successfully!\r\n\r\n");

    else
        printf("-> Calibration steps are suspended!\r\n\r\n");

    t.stop();
}

/////////////// Read Calibration status  //////////////////
uint8_t BNO055::read_calib_status(void)
{
    select_page(0);
    dt[0] = BNO055_CALIB_STAT;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 1, false);
    return dt[0];
}

/////////////// Change Fusion mode  ///////////////////////
void BNO055::change_fusion_mode(uint8_t mode)
{
    uint8_t current_mode;

    select_page(0);
    current_mode = get_operating_mode();
    switch (mode) {
        case CONFIGMODE:
            dt[0] = BNO055_OPR_MODE;
            dt[1] = mode;
            _i2c.write(chip_addr, dt, 2, false);
            ThisThread::sleep_for(19ms);    // wait 19mS
            break;
        case MODE_IMU:
        case MODE_COMPASS:
        case MODE_M4G:
        case MODE_NDOF_FMC_OFF:
        case MODE_NDOF:
            if (current_mode != CONFIGMODE) {   // Can we change the mode directry?
                dt[0] = BNO055_OPR_MODE;
                dt[1] = CONFIGMODE;
                _i2c.write(chip_addr, dt, 2, false);
                ThisThread::sleep_for(19ms);    // wait 19mS
            }
            dt[0] = BNO055_OPR_MODE;
            dt[1] = mode;
            _i2c.write(chip_addr, dt, 2, false);
            ThisThread::sleep_for(7ms);    // wait 7mS
            break;
        default:
            break;
    }
}

uint8_t BNO055::get_operating_mode(void)
{
    select_page(0);
    dt[0] = BNO055_OPR_MODE;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 1, false);
    return dt[0];
}

/////////////// Set Mouting position  /////////////////////
void BNO055::set_mounting_position(uint8_t position)
{
    uint8_t remap_config;
    uint8_t remap_sign;
    uint8_t current_mode;

    current_mode = get_operating_mode();
    change_fusion_mode(CONFIGMODE);
    switch (position) {
        case MT_P0:
            remap_config = 0x21;
            remap_sign = 0x04;
            break;
        case MT_P2:
            remap_config = 0x24;
            remap_sign = 0x06;
            break;
        case MT_P3:
            remap_config = 0x21;
            remap_sign = 0x02;
            break;
        case MT_P4:
            remap_config = 0x24;
            remap_sign = 0x03;
            break;
        case MT_P5:
            remap_config = 0x21;
            remap_sign = 0x01;
            break;
        case MT_P6:
            remap_config = 0x21;
            remap_sign = 0x07;
            break;
        case MT_P7:
            remap_config = 0x24;
            remap_sign = 0x05;
            break;
        case MT_P1:
        default:
            remap_config = 0x24;
            remap_sign = 0x00;
            break;
    }
    dt[0] = BNO055_AXIS_MAP_CONFIG;
    dt[1] = remap_config;
    dt[2] = remap_sign;
    _i2c.write(chip_addr, dt, 3, false);
    change_fusion_mode(current_mode);
}

/////////////// I2C Freq. /////////////////////////////////
void BNO055::frequency(int hz)
{
    _i2c.frequency(hz);
}

/////////////// Read/Write specific register //////////////
uint8_t BNO055::read_reg0(uint8_t addr)
{
    select_page(0);
    dt[0] = addr;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 1, false);
    return (uint8_t)dt[0];
}

uint8_t BNO055::write_reg0(uint8_t addr, uint8_t data)
{
    uint8_t current_mode;
    uint8_t d;

    current_mode = get_operating_mode();
    change_fusion_mode(CONFIGMODE);
    dt[0] = addr;
    dt[1] = data;
    _i2c.write(chip_addr, dt, 2, false);
    d = dt[0];
    change_fusion_mode(current_mode);
    return d;
}

uint8_t BNO055::read_reg1(uint8_t addr)
{
    select_page(1);
    dt[0] = addr;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 1, false);
    return (uint8_t)dt[0];
}

uint8_t BNO055::write_reg1(uint8_t addr, uint8_t data)
{
    uint8_t current_mode;
    uint8_t d;

    current_mode = get_operating_mode();
    change_fusion_mode(CONFIGMODE);
    select_page(1);
    dt[0] = addr;
    dt[1] = data;
    _i2c.write(chip_addr, dt, 2, false);
    d = dt[0];
    change_fusion_mode(current_mode);
    return d;
}

#pragma clang diagnostic pop