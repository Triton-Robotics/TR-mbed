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

#include    "mbed.h"
#include    "BNO055.h"
#include    "BNO055.cpp"


I2C    i2c(PB_9, PB_8);                // SDA, SCL
BNO055 imu(i2c, PA_8, MODE_NDOF);

BNO055_ID_INF_TypeDef           bno055_id_inf;
BNO055_ANGULAR_POSITION_typedef p;
BNO055_VECTOR_TypeDef           angular_acc;
BNO055_VECTOR_TypeDef           linear_acc;
BNO055_VECTOR_TypeDef           gravity;
BNO055_TEMPERATURE_TypeDef      chip_temp;

int main()
{

    imu.set_mounting_position(MT_P1);
    printf(
            "Bosch Sensortec BNO055 test program on " __DATE__ "/" __TIME__ "\r\n"
    );
    // Is BNO055 available?
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

    //imu.calibrate();

    while(true) {
        imu.get_angular_position_quat(&p);
        printf("[p],Y,%d,R,%d,P,%d,", (int)p.yaw, (int)p.roll, (int)p.pitch);

        imu.get_gyro(&angular_acc);
        printf("[A],X,%d,Y,%d,Z,%d,", (int)angular_acc.x, (int)angular_acc.y, (int)angular_acc.z);

        imu.get_linear_accel(&linear_acc);
        printf("[L],X,%d,Y,%d,Z,%d,", (int)linear_acc.x, (int)linear_acc.y, (int)linear_acc.z);

        imu.get_gravity(&gravity);
        printf("[G],X,%d,Y,%d,Z,%d,", (int)gravity.x, (int)gravity.y, (int)gravity.z);

        imu.get_chip_temperature(&chip_temp);
        printf("[T],%+d,%+d,", (int)chip_temp.acc_chip, (int)chip_temp.gyr_chip);

        ThisThread::sleep_for(1000ms);
    }
}