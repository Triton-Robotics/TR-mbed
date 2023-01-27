/*
 * mbed Application program for the mbed Nucleo series
 *  BNO055 Intelligent 9-axis absolute orientation sensor
 *  by Bosch Sensortec
 *
 * Copyright (c) 2015,'17,'20 Kenji Arai / JH1PJL
 *  http://www7b.biglobe.ne.jp/~kenjia/
 *  https://os.mbed.com/users/kenjiArai/
 *      Created: March     30th, 2015
 *      Revised: August     5th, 2020
 */

//  Include --------------------------------------------------------------------
#include    "mbed.h"
#include    "../util/imu/BNO055.h"
#include    "../util/imu/BNO055.cpp"

//  Definition -----------------------------------------------------------------
#define NUM_LOOP    100

I2C    i2c(PB_9, PB_8);                // SDA, SCL
BNO055 imu(i2c, PA_8);

Timer t;

//  RAM ------------------------------------------------------------------------
BNO055_ID_INF_TypeDef       bno055_id_inf;
BNO055_EULER_TypeDef        euler_angles;
BNO055_QUATERNION_TypeDef   quaternion;
BNO055_VECTOR_TypeDef      linear_acc;
BNO055_VECTOR_TypeDef      gravity;
BNO055_TEMPERATURE_TypeDef  chip_temp;

//  ROM / Constant data --------------------------------------------------------

//  Function prototypes --------------------------------------------------------

//------------------------------------------------------------------------------
//  Control Program
//------------------------------------------------------------------------------
// Calibration
//  Please refer
//      BNO055 Data sheet 3.10 Calibration & 3.6.4 Sensor calibration data



int main()
{

    imu.change_fusion_mode(MODE_IMU);
    imu.calibrate();
    imu.set_mounting_position(MT_P1);
    printf(
            "Bosch Sensortec BNO055 test program on " __DATE__ "/" __TIME__ "\r\n"
    );
    // Is BNO055 avairable?
    if (imu.chip_ready() == 0) {
        do {
            printf("Bosch BNO055 is NOT avirable!!\r\n Reset\r\n");
            ThisThread::sleep_for(100ms);
            ThisThread::sleep_for(20ms);
        } while(imu.reset());
    }

    printf("Bosch BNO055 is available now!!\r\n");
    printf("AXIS_REMAP_CONFIG:0x%02x, AXIS_REMAP_SIGN:0x%02x\r\n",
           imu.read_reg0(BNO055_AXIS_MAP_CONFIG),
           imu.read_reg0(BNO055_AXIS_MAP_SIGN)
    );

    imu.read_id_inf(&bno055_id_inf);
    printf("CHIP ID:0x%02x, ACC ID:0x%02x, MAG ID:0x%02x, GYR ID:0x%02x, ",
           bno055_id_inf.chip_id, bno055_id_inf.acc_id,
           bno055_id_inf.mag_id, bno055_id_inf.gyr_id
    );

    printf("SW REV:0x%04x, BL REV:0x%02x\r\n",
           bno055_id_inf.sw_rev_id, bno055_id_inf.bootldr_rev_id);
    ThisThread::sleep_for(3000ms);

    bno055_calbration();

    printf("[E]:Euler Angles[deg],[Q]:Quaternion[],[L]:Linear accel[m/s*s],");
    printf("[G]:Gravity vector[m/s*s],[T]:Chip temperature,Acc,Gyr[degC]");
    printf(",[S]:Status,[M]:time[mS]\r\n");
    t.start();

    while(true) {
        imu.get_euler_angles(&euler_angles);
        printf("[E],Y,%d,R,%d,P,%d,",
               (int)euler_angles.h, (int)euler_angles.r, (int)euler_angles.p);
        imu.get_quaternion(&quaternion);
        printf("[Q],W,%d,X,%d,Y,%d,Z,%d,",
               (int)quaternion.w, (int)quaternion.x, (int)quaternion.y, (int)quaternion.z);
        imu.get_linear_accel(&linear_acc);
        printf("[L],X,%d,Y,%d,Z,%d,",
               (int)linear_acc.x, (int)linear_acc.y, (int)linear_acc.z);
        imu.get_gravity(&gravity);
        printf("[G],X,%d,Y,%d,Z,%d,",
               (int)gravity.x, (int)gravity.y, (int)gravity.z);
        imu.get_chip_temperature(&chip_temp);
        printf("[T],%+d,%+d,",
               (int)chip_temp.acc_chip, (int)chip_temp.gyr_chip);
        printf("[S],0x%x,[M],%d\r\n",
               imu.read_calib_status(), (uint32_t)t.elapsed_time().count());
    }
}


// Diffrent output format as for your reference
#if 0
int main()
{
    uint8_t i;

    printf(
        "Bosch Sensortec BNO055 test program on " __DATE__ "/" __TIME__ "\r\n"
    );
    // Is BNO055 avairable?
    if (imu.chip_ready() == 0) {
        do {
            printf("Bosch BNO055 is NOT avirable!!\r\n");
            ThisThread::sleep_for(100ms);
            ThisThread::sleep_for(20ms);
        } while(imu.reset());
    }
    imu.set_mounting_position(MT_P6);
    printf("AXIS_REMAP_CONFIG:0x%02x, AXIS_REMAP_SIGN:0x%02x\r\n",
           imu.read_reg0(BNO055_AXIS_MAP_CONFIG),
           imu.read_reg0(BNO055_AXIS_MAP_SIGN)
          );
    imu.read_id_inf(&bno055_id_inf);
    printf("CHIP:0x%02x, ACC:0x%02x, MAG:0x%02x,",
           bno055_id_inf.chip_id, bno055_id_inf.acc_id, bno055_id_inf.mag_id
          );
    printf("GYR:0x%02x, , SW:0x%04x, , BL:0x%02x\r\n",
           bno055_id_inf.gyr_id, bno055_id_inf.sw_rev_id,
           bno055_id_inf.bootldr_rev_id
          );
    while(true) {
        printf("Euler Angles data\r\n");
        for (i = 0; i < NUM_LOOP; i++) {
            imu.get_Euler_Angles(&euler_angles);
            printf("Heading:%+6.1f [deg], Roll:%+6.1f [deg],",
                   euler_angles.h, euler_angles.r,);
            printf(" Pich:%+6.1f [deg], #%02d\r\n",
                   euler_angles.p, i);
            ThisThread::sleep_for(500ms);
        }
        printf("Quaternion data\r\n");
        for (i = 0; i < NUM_LOOP; i++) {
            imu.get_quaternion(&quaternion);
            printf("W:%d, X:%d, Y:%d, Z:%d, #%02d\r\n",
                   quaternion.w, quaternion.x, quaternion.y, quaternion.z, i);
            ThisThread::sleep_for(500ms);
        }
        printf("Linear accel data\r\n");
        for (i = 0; i < NUM_LOOP; i++) {
            imu.get_linear_accel(&linear_acc);
            printf(
                "X:%+6.1f[m/s*s], Y:%+6.1f[m/s*s], Z:%+6.1f[m/s*s], #%02d\r\n",
                linear_acc.x, linear_acc.y, linear_acc.z, i
            );
            ThisThread::sleep_for(500ms);
        }
        printf("Gravity vector data\r\n");
        for (i = 0; i < NUM_LOOP; i++) {
            imu.get_gravity(&gravity);
            printf(
                "X:%+6.1f[m/s*s], Y:%+6.1f[m/s*s], Z:%+6.1f[m/s*s], #%02d\r\n",
                gravity.x, gravity.y, gravity.z, i
            );
            ThisThread::sleep_for(500ms);
        }
        printf("Chip temperature data\r\n");
        for (i = 0; i < (NUM_LOOP / 4); i++) {
            imu.get_chip_temperature(&chip_temp);
            printf("Acc chip:%+d [degC], Gyr chip:%+d [degC], #%02d\r\n",
                   chip_temp.acc_chip, chip_temp.gyr_chip, i);
            ThisThread::sleep_for(500ms);
        }
    }
}
#endif