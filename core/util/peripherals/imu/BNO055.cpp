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
//
///*
// * This library has been modified. Original can be found here: https://os.mbed.com/users/AlexanderLill/code/BNO055_fusion/
// */
//
//#include "mbed.h"
//#include "BNO055.h"
//
//#pragma clang diagnostic push
//#pragma ide diagnostic ignored "UnusedParameter"
//#pragma ide diagnostic ignored "UnusedLocalVariable"
//
//BNO055::BNO055 (PinName p_sda, PinName p_scl, PinName p_reset, uint8_t addr, uint8_t mode):
//    _i2c_p(new I2C(p_sda, p_scl)), _i2c(*_i2c_p), _res(p_reset)
//{
//    chip_addr = addr;
//    chip_mode = mode;
//    initialize_reset_pin();
//    initialize ();
//}
//
//BNO055::BNO055 (PinName p_sda, PinName p_scl, PinName p_reset) :
//    _i2c_p(new I2C(p_sda, p_scl)), _i2c(*_i2c_p), _res(p_reset)
//{
//    chip_addr = BNO055_G_CHIP_ADDR;
//    chip_mode = MODE_NDOF;
//    initialize_reset_pin();
//    initialize ();
//}
//
//BNO055::BNO055 (I2C& p_i2c, PinName p_reset, uint8_t addr, uint8_t mode) :
//    _i2c(p_i2c), _res(p_reset)
//{
//    chip_addr = addr;
//    chip_mode = mode;
//    initialize_reset_pin();
//    initialize ();
//}
//
//BNO055::BNO055 (I2C& p_i2c, PinName p_reset, uint8_t mode) :
//    _i2c(p_i2c), _res(p_reset)
//{
//    chip_addr = BNO055_G_CHIP_ADDR;
//    chip_mode = mode;
//    initialize_reset_pin();
//    initialize ();
//}
//
///////////////// Read data & normalize /////////////////////
//void BNO055::get_euler_angles(BNO055_EULER_TypeDef *result)
//{
//    int16_t h,p,r;
//
//    select_page(0);
//    dt[0] = BNO055_EULER_H_LSB;
//    _i2c.write(chip_addr, dt, 1, true);
//    _i2c.read(chip_addr, dt, 6, false);
//    h = dt[1] << 8 | dt[0];
//    r = dt[3] << 8 | dt[2];
//    p = dt[5] << 8 | dt[4];
//
//    if (use_degrees()) {
//        result->h = (double)h / 16;
//        result->p = (double)p / 16;
//        result->r = (double)r / 16;
//    } else {
//        result->h = (double)h / 900;
//        result->p = (double)p / 900;
//        result->r = (double)r / 900;
//    }
//}
//
//void BNO055::get_quaternion(BNO055_QUATERNION_TypeDef *result)
//{
//    if (cantReadDataCount > 0 && cantReadDataCount < 50) {
//        cantReadDataCount++;
//        return;
//    } else if (cantReadDataCount >= 50) {
//        cantReadDataCount = 1;
//    }
//    int16_t w,x,y,z;
//
//    select_page(0);
//    dt[0] = BNO055_QUATERNION_W_LSB;
//    int writeResult = _i2c.write(chip_addr, dt, 1, true);
////    printf("IMU Write result: %i\n", writeResult);
//    if (!writeResult)  {
//        if (cantReadDataCount > 0) {
//            reset();
//            cantReadDataCount = 0;
//        }
//        _i2c.read(chip_addr, dt, 8, false);
//        w = dt[1] << 8 | dt[0];
//        x = dt[3] << 8 | dt[2];
//        y = dt[5] << 8 | dt[4];
//        z = dt[7] << 8 | dt[6];
//
//        result->w = (double)w / 16384.0f;
//        result->x = (double)x / 16384.0f;
//        result->y = (double)y / 16384.0f;
//        result->z = (double)z / 16384.0f;
//    } else {
//        cantReadDataCount++;
//    }
//}
//
//void BNO055::get_angular_position_quat(BNO055_ANGULAR_POSITION_typedef *result){
//
//    BNO055_QUATERNION_TypeDef q;
//    get_quaternion(&q);
//
//    result -> roll  = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)) * 180 / PI;
//    result -> pitch = asin(2 * q.w * q.y - q.x * q.z) * 180 / PI;
//    result -> yaw   = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) * 180 / PI;
//    double target = multiturnYaw - ((int) multiturnYaw) % 360 + (int) (result -> yaw) % 360;
//    if (target - multiturnYaw > 180) {
//        multiturnYaw = target - 360;
//    } else if (target - multiturnYaw < -180) {
//        multiturnYaw = target + 360;
//    } else {
//        multiturnYaw = target;
//    }
//}
//
//void BNO055::get_linear_accel(BNO055_VECTOR_TypeDef *result)
//{
//    int16_t x,y,z;
//
//    select_page(0);
//    dt[0] = BNO055_LINEAR_ACC_X_LSB;
//    _i2c.write(chip_addr, dt, 1, true);
//    _i2c.read(chip_addr, dt, 6, false);
//    x = dt[1] << 8 | dt[0];
//    y = dt[3] << 8 | dt[2];
//    z = dt[5] << 8 | dt[4];
//
//    if (use_mss()) {
//        result->x = (double)x / 100;
//        result->y = (double)y / 100;
//        result->z = (double)z / 100;
//    } else {
//        result->x = (double)x;
//        result->y = (double)y;
//        result->z = (double)z;
//    }
//}
//
//void BNO055::get_gravity(BNO055_VECTOR_TypeDef *result)
//{
//    int16_t x,y,z;
//
//    select_page(0);
//    dt[0] = BNO055_GRAVITY_X_LSB;
//    _i2c.write(chip_addr, dt, 1, true);
//    _i2c.read(chip_addr, dt, 6, false);
//    x = dt[1] << 8 | dt[0];
//    y = dt[3] << 8 | dt[2];
//    z = dt[5] << 8 | dt[4];
//
//    if (use_mss()) {
//        result->x = (double)x / 100;
//        result->y = (double)y / 100;
//        result->z = (double)z / 100;
//    } else {
//        result->x = (double)x;
//        result->y = (double)y;
//        result->z = (double)z;
//    }
//}
//
//void BNO055::get_mag(BNO055_VECTOR_TypeDef *result)
//{
//    int16_t x,y,z;
//
//    select_page(0);
//    dt[0] = BNO055_MAG_X_LSB;
//    _i2c.write(chip_addr, dt, 1, true);
//    _i2c.read(chip_addr, dt, 6, false);
//    x = dt[1] << 8 | dt[0];
//    y = dt[3] << 8 | dt[2];
//    z = dt[5] << 8 | dt[4];
//
//    result->x = (double)x;
//    result->y = (double)y;
//    result->z = (double)z;
//}
//
//void BNO055::get_accel(BNO055_VECTOR_TypeDef *result)
//{
//    int16_t x,y,z;
//
//    select_page(0);
//    dt[0] = BNO055_ACC_X_LSB;
//    _i2c.write(chip_addr, dt, 1, true);
//    _i2c.read(chip_addr, dt, 6, false);
//    x = dt[1] << 8 | dt[0];
//    y = dt[3] << 8 | dt[2];
//    z = dt[5] << 8 | dt[4];
//
//    if (use_mss()) {
//        result->x = (double)x / 100;
//        result->y = (double)y / 100;
//        result->z = (double)z / 100;
//    } else {
//        result->x = (double)x;
//        result->y = (double)y;
//        result->z = (double)z;
//    }
//}
//
//void BNO055::get_gyro(BNO055_VECTOR_TypeDef *result)
//{
//    int16_t x,y,z;
//
//    select_page(0);
//    dt[0] = BNO055_GYR_X_LSB;
//    _i2c.write(chip_addr, dt, 1, true);
//    _i2c.read(chip_addr, dt, 6, false);
//    x = dt[1] << 8 | dt[0];
//    y = dt[3] << 8 | dt[2];
//    z = dt[5] << 8 | dt[4];
//    if (use_dps()) {
//        result->x = (double)x / 16;
//        result->y = (double)y / 16;
//        result->z = (double)z / 16;
//    } else {
//        result->x = (double)x / 900;
//        result->y = (double)y / 900;
//        result->z = (double)z / 900;
//    }
//}
//
//void BNO055::get_chip_temperature(BNO055_TEMPERATURE_TypeDef *result)
//{
//    select_page(0);
//
//    uint8_t use_celsius_result = use_celsius();
//
//    dt[0] = BNO055_TEMP_SOURCE;
//    dt[1] = 0;
//    _i2c.write(chip_addr, dt, 2, false);
//    ThisThread::sleep_for(1ms); // Do I need to wait?
//    dt[0] = BNO055_TEMP;
//    _i2c.write(chip_addr, dt, 1, true);
//    _i2c.read(chip_addr, dt, 1, false);
//
//    if (use_celsius_result)
//        result -> acc_chip = (int8_t)dt[0];
//
//    else
//        result -> acc_chip = (int8_t)dt[0] * 2;
//
//
//    dt[0] = BNO055_TEMP_SOURCE;
//    dt[1] = 1;
//    _i2c.write(chip_addr, dt, 2, false);
//    ThisThread::sleep_for(1ms); // Do I need to wait?
//    dt[0] = BNO055_TEMP;
//    _i2c.write(chip_addr, dt, 1, true);
//    _i2c.read(chip_addr, dt, 1, false);
//
//    if (use_celsius_result)
//        result -> gyr_chip = (int8_t)dt[0];
//
//    else
//        result -> gyr_chip = (int8_t)dt[0] * 2;
//}
//
///////////////// Initialize ////////////////////////////////
//void BNO055::initialize (void)
//{
//    _i2c.frequency(400000);
//
//    page_flag = 0xff;
//    select_page(0);
//
//    // Check Acc & Mag & Gyro are available of not
//    get_id();
//
//    // Set initial data
//    set_initial_dt_to_regs();
//
//    // Unit selection
//    unit_selection();
//
//    // Set fusion mode
//    change_fusion_mode(chip_mode);
//}
//
//void BNO055::initialize_reset_pin(void)
//{
//    _res = 1;
//    ThisThread::sleep_for(700ms); // Need to wait at least 650mS
//}
//
//void BNO055::unit_selection(void)
//{
//    select_page(0);
//    dt[0] = BNO055_UNIT_SEL;
//    dt[1] = UNIT_ORI_WIN + UNIT_ACC_MSS + UNIT_GYR_DPS + UNIT_EULER_DEG + UNIT_TEMP_C;
//    _i2c.write(chip_addr, dt, 2, false);
//}
//
//bool BNO055::use_degrees()
//{
//    if (unit_flag_is_set(UNIT_EULER_RAD)) {
//        return false;
//    }
//    return true;
//}
//
//bool BNO055::use_mss()
//{
//    if (unit_flag_is_set(UNIT_ACC_MG)) {
//        return false;
//    }
//    return true;
//}
//
//bool BNO055::use_dps()
//{
//    if (unit_flag_is_set(UNIT_GYR_RPS)) {
//        return false;
//    }
//    return true;
//}
//
//bool BNO055::use_celsius()
//{
//    if (unit_flag_is_set(UNIT_TEMP_F)) {
//        return false;
//    }
//    return true;
//}
//
//bool BNO055::unit_flag_is_set(uint8_t flag)
//{
//    select_page(0);
//    dt[0] = BNO055_UNIT_SEL;
//    _i2c.write(chip_addr, dt, 1, true);
//    _i2c.read(chip_addr, dt, 1, false);
//    if (dt[0] & flag) {
//        return true;
//    }
//    return false;
//}
//
//uint8_t BNO055::select_page(uint8_t page)
//{
//    if (page != page_flag){
//        dt[0] = BNO055_PAGE_ID;
//        if (page == 1) {
//            dt[1] = 1;  // select page 1
//        } else {
//            dt[1] = 0;  // select page 0
//        }
//        _i2c.write(chip_addr, dt, 2, false);
//        dt[0] = BNO055_PAGE_ID;
//        _i2c.write(chip_addr, dt, 1, true);
//        _i2c.read(chip_addr, dt, 1, false);
//        page_flag = dt[0];
//    }
//    return page_flag;
//}
//
//uint8_t BNO055::reset(void)
//{
//     _res = 0;
//     ThisThread::sleep_for(1ms);   // Reset 1mS
//     _res = 1;
//     ThisThread::sleep_for(700ms); // Need to wait at least 650mS
//#if defined(TARGET_STM32L152RE)
//    _i2c.frequency(400000);
//#else
//    _i2c.frequency(400000);
//#endif
//    _i2c.stop();
//    page_flag = 0xff;
//    select_page(0);
//    get_id();
//    if (chip_id != I_AM_BNO055_CHIP){
//        return 1;
//    } else {
//        initialize();
//        return 0;
//    }
//}
//
//////// Set initialize data to related registers ///////////
//void BNO055::set_initial_dt_to_regs(void)
//{
//    // select_page(0);
//    // current setting is only used default values
//}
//
///////////////// Check Who am I? ///////////////////////////
//void BNO055::get_id(void)
//{
//    select_page(0);
//    // ID
//    dt[0] = BNO055_CHIP_ID;
//    _i2c.write(chip_addr, dt, 1, true);
//    _i2c.read(chip_addr, dt, 7, false);
//    chip_id = dt[0];
//    if (chip_id == I_AM_BNO055_CHIP) {
//        ready_flag = 1;
//    } else {
//        ready_flag = 0;
//    }
//    acc_id = dt[1];
//    if (acc_id == I_AM_BNO055_ACC) {
//        ready_flag |= 2;
//    }
//    mag_id = dt[2];
//    if (mag_id == I_AM_BNO055_MAG) {
//        ready_flag |= 4;
//    }
//    gyr_id = dt[3];
//    if (mag_id == I_AM_BNO055_MAG) {
//        ready_flag |= 8;
//    }
//    bootldr_rev_id = dt[5]<< 8 | dt[4];
//    sw_rev_id = dt[6];
//}
//
//void BNO055::read_id_inf(BNO055_ID_INF_TypeDef *id)
//{
//    id->chip_id = chip_id;
//    id->acc_id = acc_id;
//    id->mag_id = mag_id;
//    id->gyr_id = gyr_id;
//    id->bootldr_rev_id = bootldr_rev_id;
//    id->sw_rev_id = sw_rev_id;
//}
//
///////////////// Check chip ready or not  //////////////////
//uint8_t BNO055::chip_ready(void)
//{
//    if (ready_flag == 0x0f) {
//        return 1;
//    }
//    return 0;
//}
//
///////////////// Calibrate IMU  ////////////////////
//void BNO055::calibrate()
//{
//    uint8_t d;
//    Timer t;
//    BNO055_VECTOR_TypeDef      gravity;
//
//    printf("------ Enter BNO055 Manual Calibration Mode ------\r\n");
//
//    //---------- Gyroscope Caliblation -----------------------------------------
//    // (a) Place the device in a single stable position for a period of
//    //     few seconds to allow the gyroscope to calibrate
//
//    printf("Step1) Please wait few seconds\r\n");
//    t.start();
//
//    while (t.elapsed_time().count() / 1000000 < 10) {
//        d = read_calib_status();
//        printf("Calb dat = 0x%x target  = 0x30(at least)\r\n", d);
//        if ((d & 0x30) == 0x30) {
//            break;
//        }
//        ThisThread::sleep_for(1s);
//    }
//
//    printf("-> Step1) is done\r\n\r\n");
//
//    //---------- Magnetometer Caliblation --------------------------------------
//    // (a) Make some random movements (for example: writing the number ‘8’
//    //     on air) until the CALIB_STAT register indicates fully calibrated.
//    // (b) It takes more calibration movements to get the magnetometer
//    //     calibrated than in the NDOF mode.
//
//    printf("Step2) random moving (try to change the BNO055 axis)\r\n");
//    t.start();
//
//    while (t.elapsed_time().count() / 1000000 < 30) {
//        d = read_calib_status();
//        printf("Calb dat = 0x%x target  = 0x33(at least)\r\n", d);
//        if ((d & 0x03) == 0x03) {
//            break;
//        }
//        ThisThread::sleep_for(1s);
//    }
//
//    printf("-> Step2) is done\r\n\r\n");
//
//    //---------- Magnetometer Caliblation --------------------------------------
//    // a) Place the device in 6 different stable positions for a period of
//    //    few seconds to allow the accelerometer to calibrate.
//    // b) Make sure that there is slow movement between 2 stable positions
//    //    The 6 stable positions could be in any direction, but make sure that
//    //    the device is lying at least once perpendicular to the x, y and z axis
//
//    printf("Step3) Change rotation each X,Y,Z axis KEEP SLOWLY!!");
//    printf(" Each 90deg stay a 5 sec and set at least 6 position.\r\n");
//    printf(" e.g. (1)ACC:X0,Y0,Z-9,(2)ACC:X9,Y0,Z0,(3)ACC:X0,Y0,Z9,");
//    printf("(4)ACC:X-9,Y0,Z0,(5)ACC:X0,Y-9,Z0,(6)ACC:X0,Y9,Z0,\r\n");
//    printf(" If you will give up, hit any key.\r\n");
//    t.stop();
//
//
//    while (true) {
//        d = read_calib_status();
//        get_gravity(&gravity);
//        printf(
//                "Calb dat = 0x%x target  = 0xff ACC:X %d, Y %d, Z %d\r\n",
//                d, (int)gravity.x, (int)gravity.y, (int)gravity.z
//        );
//        if (d == 0xff)
//            break;
//
//        ThisThread::sleep_for(1s);
//    }
//    if (read_calib_status() == 0xff)
//        printf("-> All of Calibration steps are done successfully!\r\n\r\n");
//
//    else
//        printf("-> Calibration steps are suspended!\r\n\r\n");
//
//    t.stop();
//}
//
///////////////// Read Calibration status  //////////////////
//uint8_t BNO055::read_calib_status(void)
//{
//    select_page(0);
//    dt[0] = BNO055_CALIB_STAT;
//    _i2c.write(chip_addr, dt, 1, true);
//    _i2c.read(chip_addr, dt, 1, false);
//    return dt[0];
//}
//
///////////////// Change Fusion mode  ///////////////////////
//void BNO055::change_fusion_mode(uint8_t mode)
//{
//    uint8_t current_mode;
//
//    select_page(0);
//    current_mode = get_operating_mode();
//    switch (mode) {
//        case CONFIGMODE:
//            dt[0] = BNO055_OPR_MODE;
//            dt[1] = mode;
//            _i2c.write(chip_addr, dt, 2, false);
//            ThisThread::sleep_for(19ms);    // wait 19mS
//            break;
//        case MODE_IMU:
//        case MODE_COMPASS:
//        case MODE_M4G:
//        case MODE_NDOF_FMC_OFF:
//        case MODE_NDOF:
//            if (current_mode != CONFIGMODE) {   // Can we change the mode directry?
//                dt[0] = BNO055_OPR_MODE;
//                dt[1] = CONFIGMODE;
//                _i2c.write(chip_addr, dt, 2, false);
//                ThisThread::sleep_for(19ms);    // wait 19mS
//            }
//            dt[0] = BNO055_OPR_MODE;
//            dt[1] = mode;
//            _i2c.write(chip_addr, dt, 2, false);
//            ThisThread::sleep_for(7ms);    // wait 7mS
//            break;
//        default:
//            break;
//    }
//}
//
//uint8_t BNO055::get_operating_mode(void)
//{
//    select_page(0);
//    dt[0] = BNO055_OPR_MODE;
//    _i2c.write(chip_addr, dt, 1, true);
//    _i2c.read(chip_addr, dt, 1, false);
//    return dt[0];
//}
//
///////////////// Set Mouting position  /////////////////////
//void BNO055::set_mounting_position(uint8_t position)
//{
//    uint8_t remap_config;
//    uint8_t remap_sign;
//    uint8_t current_mode;
//
//    current_mode = get_operating_mode();
//    change_fusion_mode(CONFIGMODE);
//    switch (position) {
//        case MT_P0:
//            remap_config = 0x21;
//            remap_sign = 0x04;
//            break;
//        case MT_P2:
//            remap_config = 0x24;
//            remap_sign = 0x06;
//            break;
//        case MT_P3:
//            remap_config = 0x21;
//            remap_sign = 0x02;
//            break;
//        case MT_P4:
//            remap_config = 0x24;
//            remap_sign = 0x03;
//            break;
//        case MT_P5:
//            remap_config = 0x21;
//            remap_sign = 0x01;
//            break;
//        case MT_P6:
//            remap_config = 0x21;
//            remap_sign = 0x07;
//            break;
//        case MT_P7:
//            remap_config = 0x24;
//            remap_sign = 0x05;
//            break;
//        case MT_P1:
//        default:
//            remap_config = 0x24;
//            remap_sign = 0x00;
//            break;
//    }
//    dt[0] = BNO055_AXIS_MAP_CONFIG;
//    dt[1] = remap_config;
//    dt[2] = remap_sign;
//    _i2c.write(chip_addr, dt, 3, false);
//    change_fusion_mode(current_mode);
//}
//
///////////////// I2C Freq. /////////////////////////////////
//void BNO055::frequency(int hz)
//{
//    _i2c.frequency(hz);
//}
//
///////////////// Read/Write specific register //////////////
//uint8_t BNO055::read_reg0(uint8_t addr)
//{
//    select_page(0);
//    dt[0] = addr;
//    _i2c.write(chip_addr, dt, 1, true);
//    _i2c.read(chip_addr, dt, 1, false);
//    return (uint8_t)dt[0];
//}
//
//uint8_t BNO055::write_reg0(uint8_t addr, uint8_t data)
//{
//    uint8_t current_mode;
//    uint8_t d;
//
//    current_mode = get_operating_mode();
//    change_fusion_mode(CONFIGMODE);
//    dt[0] = addr;
//    dt[1] = data;
//    _i2c.write(chip_addr, dt, 2, false);
//    d = dt[0];
//    change_fusion_mode(current_mode);
//    return d;
//}
//
//uint8_t BNO055::read_reg1(uint8_t addr)
//{
//    select_page(1);
//    dt[0] = addr;
//    _i2c.write(chip_addr, dt, 1, true);
//    _i2c.read(chip_addr, dt, 1, false);
//    return (uint8_t)dt[0];
//}
//
//uint8_t BNO055::write_reg1(uint8_t addr, uint8_t data)
//{
//    uint8_t current_mode;
//    uint8_t d;
//
//    current_mode = get_operating_mode();
//    change_fusion_mode(CONFIGMODE);
//    select_page(1);
//    dt[0] = addr;
//    dt[1] = data;
//    _i2c.write(chip_addr, dt, 2, false);
//    d = dt[0];
//    change_fusion_mode(current_mode);
//    return d;
//}
//
//#pragma clang diagnostic pop





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

#include <math.h>
#include <limits.h>
#include "mbed.h"

//#include "Adafruit_BNO055.h"
#include "BNO055.h"
#define PI 3.1415926535897932384626433

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates a new Adafruit_BNO055 class
*/
/**************************************************************************/
//Adafruit_BNO055::Adafruit_BNO055(int32_t sensorID, uint8_t address, I2C* i2c_ptr)
//BNO055::BNO055(int32_t sensorID, uint8_t address, I2C* i2c_ptr)
BNO055::BNO055(I2C& i2c_ptr, int32_t sensorID, uint8_t address) : i2c(i2c_ptr)
{
_sensorID = sensorID;
_address = address;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Sets up the HW
*/
/**************************************************************************/
//bool Adafruit_BNO055::begin(adafruit_bno055_opmode_t mode)
bool BNO055::begin(adafruit_bno055_opmode_t mode)
{
/* Enable I2C */
//printf("2\n");
i2c.frequency(400000);
//printf("3\n");
//i2c_dev->begin(false);
//
//#if defined(TARGET_RP2040)
//// philhower core seems to work with this speed?
//i2c_dev->setSpeed(50000);
//#endif
//
//// can take 850 ms to boot!
//int timeout = 850; // in ms
//while (timeout > 0) {
//if (i2c_dev->begin()) {
//break;
//}
//// wasnt detected... we'll retry!
//delay(10);
//timeout -= 10;
//}
//if (timeout <= 0)
//return false;

/* Make sure we have the right device */
uint8_t id = read8(BNO055_CHIP_ID_ADDR);
//printf("4\n");
if(id != BNO055_ID)
{
//ThisThread::sleep_for(1000ms); // hold on for boot
id = read8(BNO055_CHIP_ID_ADDR);
    if(id != BNO055_ID) {
    //printf("2\n");
    return false;  // still not? ok bail
    }
}

/* Switch to config mode (just in case since this is the default) */
setMode(OPERATION_MODE_CONFIG);
//printf("66\n");

/* Reset */
write8(BNO055_SYS_TRIGGER_ADDR, 0x20);
//printf("7\n");

while (read8(BNO055_CHIP_ID_ADDR) != BNO055_ID)
{
//ThisThread::sleep_for(10);
}
//printf("8\n");

//ThisThread::sleep_for(50);

/* Set to normal power mode */
write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
//printf("9\n");

ThisThread::sleep_for(10);

write8(BNO055_PAGE_ID_ADDR, 0);
//printf("10\n");

/* Set the output units */
/*
uint8_t unitsel = (0 << 7) | // Orientation = Android
                  (0 << 4) | // Temperature = Celsius
                  (0 << 2) | // Euler = Degrees
                  (1 << 1) | // Gyro = Rads
                  (0 << 0);  // Accelerometer = m/s^2
write8(BNO055_UNIT_SEL_ADDR, unitsel);
*/

write8(BNO055_SYS_TRIGGER_ADDR, 0x0);
//printf("11\n");

//ThisThread::sleep_for(10);
/* Set the requested operating mode (see section 3.3) */
//setMode(mode);
setMode(OPERATION_MODE_NDOF);
//ThisThread::sleep_for(20);

return true;
}

/**************************************************************************/
/*!
    @brief  Puts the chip in the specified operating mode
*/
/**************************************************************************/
//void Adafruit_BNO055::setMode(adafruit_bno055_opmode_t mode)
void BNO055::setMode(adafruit_bno055_opmode_t mode)
{
_mode = mode;
write8(BNO055_OPR_MODE_ADDR, _mode);
//ThisThread::sleep_for(30);
}

/**************************************************************************/
/*!
    @brief  Use the external 32.768KHz crystal
*/
/**************************************************************************/
//void Adafruit_BNO055::setExtCrystalUse(bool usextal)
void BNO055::setExtCrystalUse(bool usextal)
{
adafruit_bno055_opmode_t modeback = _mode;

/* Switch to config mode (just in case since this is the default) */
setMode(OPERATION_MODE_CONFIG);
ThisThread::sleep_for(25);
write8(BNO055_PAGE_ID_ADDR, 0);
if (usextal) {
write8(BNO055_SYS_TRIGGER_ADDR, 0x80);
} else {
write8(BNO055_SYS_TRIGGER_ADDR, 0x00);
}
ThisThread::sleep_for(10);
/* Set the requested operating mode (see section 3.3) */
setMode(modeback);
ThisThread::sleep_for(20);
}


/**************************************************************************/
/*!
    @brief  Gets the latest system status info
*/
/**************************************************************************/
//void Adafruit_BNO055::getSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error)
void BNO055::getSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error)
{
write8(BNO055_PAGE_ID_ADDR, 0);

/* System Status (see section 4.3.58)
   ---------------------------------
   0 = Idle
   1 = System Error
   2 = Initializing Peripherals
   3 = System Iniitalization
   4 = Executing Self-Test
   5 = Sensor fusio algorithm running
   6 = System running without fusion algorithms */

if (system_status != 0)
*system_status    = read8(BNO055_SYS_STAT_ADDR);

/* Self Test Results (see section )
   --------------------------------
   1 = test passed, 0 = test failed

   Bit 0 = Accelerometer self test
   Bit 1 = Magnetometer self test
   Bit 2 = Gyroscope self test
   Bit 3 = MCU self test

   0x0F = all good! */

if (self_test_result != 0)
*self_test_result = read8(BNO055_SELFTEST_RESULT_ADDR);

/* System Error (see section 4.3.59)
   ---------------------------------
   0 = No error
   1 = Peripheral initialization error
   2 = System initialization error
   3 = Self test result failed
   4 = Register map value out of range
   5 = Register map address out of range
   6 = Register map write error
   7 = BNO low power mode not available for selected operat ion mode
   8 = Accelerometer power mode not available
   9 = Fusion algorithm configuration error
   A = Sensor configuration error */

if (system_error != 0)
*system_error     = read8(BNO055_SYS_ERR_ADDR);

ThisThread::sleep_for(200);
}

/**************************************************************************/
/*!
    @brief  Gets the chip revision numbers
*/
/**************************************************************************/
//void Adafruit_BNO055::getRevInfo(adafruit_bno055_rev_info_t* info)
void BNO055::getRevInfo(adafruit_bno055_rev_info_t* info)
{
uint8_t a, b;

memset(info, 0, sizeof(adafruit_bno055_rev_info_t));

/* Check the accelerometer revision */
info->accel_rev = read8(BNO055_ACCEL_REV_ID_ADDR);

/* Check the magnetometer revision */
info->mag_rev   = read8(BNO055_MAG_REV_ID_ADDR);

/* Check the gyroscope revision */
info->gyro_rev  = read8(BNO055_GYRO_REV_ID_ADDR);

/* Check the SW revision */
info->bl_rev    = read8(BNO055_BL_REV_ID_ADDR);

a = read8(BNO055_SW_REV_ID_LSB_ADDR);
b = read8(BNO055_SW_REV_ID_MSB_ADDR);
info->sw_rev = (((uint16_t)b) << 8) | ((uint16_t)a);
}

/**************************************************************************/
/*!
    @brief  Gets current calibration state.  Each value should be a uint8_t
            pointer and it will be set to 0 if not calibrated and 3 if
            fully calibrated.
*/
/**************************************************************************/
//void Adafruit_BNO055::getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
void BNO055::getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
uint8_t calData = read8(BNO055_CALIB_STAT_ADDR);
if (sys != NULL) {
*sys = (calData >> 6) & 0x03;
}
if (gyro != NULL) {
*gyro = (calData >> 4) & 0x03;
}
if (accel != NULL) {
*accel = (calData >> 2) & 0x03;
}
if (mag != NULL) {
*mag = calData & 0x03;
}
}

/**************************************************************************/
/*!
    @brief  Gets the temperature in degrees celsius
*/
/**************************************************************************/
//int8_t Adafruit_BNO055::getTemp(void)
int8_t BNO055::getTemp(void)
{
int8_t temp = (int8_t)(read8(BNO055_TEMP_ADDR));
return temp;
}


/**************************************************************************/
/*!
    @brief  Gets a vector reading from the specified source
*/
/**************************************************************************/
//imu::Vector<3> Adafruit_BNO055::getVector(adafruit_vector_type_t vector_type)
imu::Vector<3> BNO055::getVector(adafruit_vector_type_t vector_type)
{
imu::Vector<3> xyz;
unsigned char buffer[6];
memset (buffer, 0, 6);

int16_t x, y, z;
x = y = z = 0;

/* Read vector data (6 bytes) */
//readLen((adafruit_bno055_reg_t)vector_type, (char*)buffer, 6);

readLen((adafruit_bno055_reg_t)0x08, (char*)buffer, 6);
printf("buf: %c %c; ",buffer[0], buffer[1]);
i2c.write((adafruit_bno055_reg_t )0x20, (char*)buffer, 1, true);
i2c.read((adafruit_bno055_reg_t)0x20, (char*)buffer, 1, false);
printf("x1 read: %c; ", buffer[0]);
i2c.write((adafruit_bno055_reg_t )0x21, (char*)buffer, 1, true);
i2c.read((adafruit_bno055_reg_t)0x21, ((char*)buffer) + 1, 1, false);
printf("x0 read: %d; ", buffer[1]);

x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
printf("x: %d\n", x);
y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

/* Convert the value to an appropriate range (section 3.6.4) */
/* and assign the value to the Vector type */
switch(vector_type)
{
case VECTOR_MAGNETOMETER:
/* 1uT = 16 LSB */
xyz[0] = ((double)x)/16.0;
xyz[1] = ((double)y)/16.0;
xyz[2] = ((double)z)/16.0;
break;
case VECTOR_GYROSCOPE:
/* 1rps = 900 LSB */
xyz[0] = ((double)x)/900.0;
xyz[1] = ((double)y)/900.0;
xyz[2] = ((double)z)/900.0;
break;
case VECTOR_EULER:
/* 1 degree = 16 LSB */
xyz[0] = ((double)x)/16.0;
xyz[1] = ((double)y)/16.0;
xyz[2] = ((double)z)/16.0;
break;
case VECTOR_ACCELEROMETER:
case VECTOR_LINEARACCEL:
case VECTOR_GRAVITY:
/* 1m/s^2 = 100 LSB */
xyz[0] = ((double)x)/100.0;
xyz[1] = ((double)y)/100.0;
xyz[2] = ((double)z)/100.0;
break;
}

return xyz;
}

/**************************************************************************/
/*!
    @brief  Gets a quaternion reading from the specified source
*/
/**************************************************************************/
//imu::Quaternion Adafruit_BNO055::getQuat(void)
imu::Quaternion BNO055::getQuat(void)
{
unsigned char buffer[8];
memset (buffer, 0, 8);

int x, y, z, w;
x = y = z = w = 0;

/* Read quat data (8 bytes) */
readLen(BNO055_QUATERNION_DATA_W_LSB_ADDR, (char*)buffer, 8);
i2c.write((adafruit_bno055_reg_t )0x08, (char*)buffer, 1, true);
i2c.read((adafruit_bno055_reg_t)0x08, (char*)buffer, 1, false);
i2c.write((adafruit_bno055_reg_t )0x08, (char*)buffer + 1, 1, true);
i2c.read((adafruit_bno055_reg_t)0x08, (char*)buffer + 1, 1, false);
w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
printf("w: %d \n", w);
x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);

/* Assign to Quaternion */
/* See http://ae-bst.resource.bosch.com/media/products/dokumente/bno055/BST_BNO055_DS000_12~1.pdf
   3.6.5.5 Orientation (Quaternion)  */
const double scale = (1.0 / (1<<14));
imu::Quaternion quat(scale * w, scale * x, scale * y, scale * z);
return quat;
}

/**************************************************************************/
/*!
    @brief  Provides the sensor_t data for this sensor
*/
/**************************************************************************/
//void Adafruit_BNO055::getSensor(sensor_t *sensor)
void BNO055::getSensor(sensor_t *sensor)
{
/* Clear the sensor_t object */
memset(sensor, 0, sizeof(sensor_t));

/* Insert the sensor name in the fixed length char array */
strncpy (sensor->name, "BNO055", sizeof(sensor->name) - 1);
sensor->name[sizeof(sensor->name)- 1] = 0;
sensor->version     = 1;
sensor->sensor_id   = _sensorID;
sensor->type        = SENSOR_TYPE_ORIENTATION;
sensor->min_delay   = 0;
sensor->max_value   = 0.0F;
sensor->min_value   = 0.0F;
sensor->resolution  = 0.01F;
}

/**************************************************************************/
/*!
    @brief  Reads the sensor and returns the data as a sensors_event_t
*/
/**************************************************************************/
//bool Adafruit_BNO055::getEvent(sensors_event_t *event)
bool BNO055::getEvent(sensors_event_t *event)
{
/* Clear the event */
//printf("sensor event size %d\n", sizeof(sensors_event_t));
memset(event, 0, sizeof(sensors_event_t));
//printf("check2 \n");
event->version   = sizeof(sensors_event_t);
event->sensor_id = _sensorID;
event->type      = SENSOR_TYPE_ORIENTATION;
event->timestamp = 0; //TODO: fix this with a millis() call
//printf("check3 \n");
/* Get a Euler angle sample for orientation */
//imu::Vector<3> euler = getVector(Adafruit_BNO055::VECTOR_EULER);
imu::Vector<3> euler = getVector(BNO055::VECTOR_EULER);
event->orientation.x = euler.x();
event->orientation.y = euler.y();
event->orientation.z = euler.z();
//printf("check4 \n");
return true;
}



//uint8_t BNO055::select_page(uint8_t page)
//{
//    if (page != page_flag){
//        dt[0] = BNO055_PAGE_ID;
//        if (page == 1) {
//            dt[1] = 1;  // select page 1
//        } else {
//            dt[1] = 0;  // select page 0
//        }
//        _i2c.write(0x28, dt, 2, false);
//        dt[0] = BNO055_PAGE_ID;
//        i2c.write(0x28, dt, 1, true);
//        i2c.read(0x28, dt, 1, false);
//        page_flag = dt[0];
//    }
//    return page_flag;
//}
/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C
*/
/**************************************************************************/
//bool Adafruit_BNO055::write8(adafruit_bno055_reg_t reg, char value)
bool BNO055::write8(adafruit_bno055_reg_t reg, char value)
{
char reg_to_write = (char)(reg);
i2c.write(_address<<1, &reg_to_write, 1, true);
//ThisThread::sleep_for(1);
i2c.write(_address<<1, &value, 1, false);
//ThisThread::sleep_for(1);

/* ToDo: Check for error! */
return true;
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
//char Adafruit_BNO055::read8(adafruit_bno055_reg_t reg )
char BNO055::read8(adafruit_bno055_reg_t reg )
{
char to_read = 0;
char to_write = (char)reg;
ThisThread::sleep_for(1);

i2c.write(_address<<1, &to_write, 1, true);
i2c.read(_address<<1, &to_read, 1, false);
ThisThread::sleep_for(1);
//i2c.write(_address, &to_write, 1, false);
//ThisThread::sleep_for(1);
//i2c.read(_address, &to_read, 1, false);
//ThisThread::sleep_for(1);
printf(" I2C Read : %d from addr: %d\r\n", to_read, to_write);
return to_read;
}

/**************************************************************************/
/*!
    @brief  Reads the specified number of bytes over I2C
*/
/**************************************************************************/
//bool Adafruit_BNO055::readLen(adafruit_bno055_reg_t reg, char* buffer, int len)
bool BNO055::readLen(adafruit_bno055_reg_t reg, char* buffer, int len)
{
char reg_to_write = (char)(reg);

i2c.write(_address<<1, &reg_to_write, 1, false);
ThisThread::sleep_for(1);
i2c.read(_address<<1, buffer, len, false);
ThisThread::sleep_for(1);

printf("I2C: Read %d bytes from address %d | ", len, reg_to_write);

for(int i = 0; i < len; i++){
    printf("%x ", buffer[i]);
}
printf("\n");

/* ToDo: Check for errors! */
return true;
}





//void BNO055::get_quaternion(BNO055_QUATERNION_TypeDef *result)
//{
//    if (cantReadDataCount > 0 && cantReadDataCount < 50) {
//        cantReadDataCount++;
//        return;
//    } else if (cantReadDataCount >= 50) {
//        cantReadDataCount = 1;
//    }
//    int16_t w,x,y,z;
//
//    select_page(0);
//    dt[0] = BNO055_QUATERNION_W_LSB;
//    int writeResult = _i2c.write(chip_addr, dt, 1, true);
////    printf("IMU Write result: %i\n", writeResult);
//    if (!writeResult)  {
//        if (cantReadDataCount > 0) {
//            reset();
//            cantReadDataCount = 0;
//        }
//        _i2c.read(chip_addr, dt, 8, false);
//        w = dt[1] << 8 | dt[0];
//        x = dt[3] << 8 | dt[2];
//        y = dt[5] << 8 | dt[4];
//        z = dt[7] << 8 | dt[6];
//
//        result->w = (double)w / 16384.0f;
//        result->x = (double)x / 16384.0f;
//        result->y = (double)y / 16384.0f;
//        result->z = (double)z / 16384.0f;
//    } else {
//        cantReadDataCount++;
//    }
//}
void BNO055::get_angular_position_quat(BNO055_ANGULAR_POSITION_typedef *result){

//    BNO055_QUATERNION_TypeDef q;
//    get_quaternion(&q);
    imu::Quaternion q = getQuat();

//    result -> roll  = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)) * 180 / PI;
//    result -> pitch = asin(2 * q.w * q.y - q.x * q.z) * 180 / PI;
//    result -> yaw   = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) * 180 / PI;
    result -> roll  = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y())) * 180 / PI;
    result -> pitch = asin(2 * q.w() * q.y() - q.x() * q.z()) * 180 / PI;
    result -> yaw   = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z())) * 180 / PI;//    double target = multiturnYaw - ((int) multiturnYaw) % 360 + (int) (result -> yaw) % 360;
//    if (target - multiturnYaw > 180) {
//        multiturnYaw = target - 360;
//    } else if (target - multiturnYaw < -180) {
//        multiturnYaw = target + 360;
//    } else {
//        multiturnYaw = target;
//    }
}


//void BNO055::get_quaternion(BNO055_QUATERNION_TypeDef *result)
//{
//    if (cantReadDataCount > 0 && cantReadDataCount < 50) {
//        cantReadDataCount++;
//        return;
//    } else if (cantReadDataCount >= 50) {
//        cantReadDataCount = 1;
//    }
//    int16_t w,x,y,z;
//
//    select_page(0);
//    dt[0] = BNO055_QUATERNION_W_LSB;
//    int writeResult = _i2c.write(chip_addr, dt, 1, true);
////    printf("IMU Write result: %i\n", writeResult);
//    if (!writeResult)  {
//        if (cantReadDataCount > 0) {
//            reset();
//            cantReadDataCount = 0;
//        }
//        _i2c.read(chip_addr, dt, 8, false);
//        w = dt[1] << 8 | dt[0];
//        x = dt[3] << 8 | dt[2];
//        y = dt[5] << 8 | dt[4];
//        z = dt[7] << 8 | dt[6];
//
//        result->w = (double)w / 16384.0f;
//        result->x = (double)x / 16384.0f;
//        result->y = (double)y / 16384.0f;
//        result->z = (double)z / 16384.0f;
//    } else {
//        cantReadDataCount++;
//    }
//}











//void BNO055::set_mounting_position(uint8_t position)
//{
//    uint8_t remap_config;
//    uint8_t remap_sign;
//    uint8_t current_mode;
//
//    current_mode = get_operating_mode();
//    change_fusion_mode(CONFIGMODE);
//    switch (position) {
//        case MT_P0:
//            remap_config = 0x21;
//            remap_sign = 0x04;
//            break;
//        case MT_P2:
//            remap_config = 0x24;
//            remap_sign = 0x06;
//            break;
//        case MT_P3:
//            remap_config = 0x21;
//            remap_sign = 0x02;
//            break;
//        case MT_P4:
//            remap_config = 0x24;
//            remap_sign = 0x03;
//            break;
//        case MT_P5:
//            remap_config = 0x21;
//            remap_sign = 0x01;
//            break;
//        case MT_P6:
//            remap_config = 0x21;
//            remap_sign = 0x07;
//            break;
//        case MT_P7:
//            remap_config = 0x24;
//            remap_sign = 0x05;
//            break;
//        case MT_P1:
//        default:
//            remap_config = 0x24;
//            remap_sign = 0x00;
//            break;
//    }
//    dt[0] = BNO055_AXIS_MAP_CONFIG;
//    dt[1] = remap_config;
//    dt[2] = remap_sign;
//    _i2c.write(chip_addr, dt, 3, false);
//    change_fusion_mode(current_mode);
//}
//
//uint8_t BNO055::get_operating_mode(void)
//{
//    select_page(0);
//    dt[0] = BNO055_OPR_MODE;
//    _i2c.write(chip_addr, dt, 1, true);
//    _i2c.read(chip_addr, dt, 1, false);
//    return dt[0];
//}