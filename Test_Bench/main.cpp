#include <cstdlib>
#include "mbed.h"
#include "../util/imu/BNO055.h"
#include "../util/imu/BNO055.cpp"

I2C    i2c(PB_9, PB_8);                // SDA, SCL
BNO055 imu(i2c, PA_8);

BNO055_ID_INF_TypeDef bno055_id_inf;
BNO055_QUATERNION_TypeDef   quat;

void getAnglesFromQuat(double &yaw, double &pitch, double &roll){

    imu.get_quaternion(&quat);
    double yy = quat.y * quat.y;

    roll = atan2(2 * (quat.w * quat.x + quat.y * quat.z), 1 - 2*(quat.x * quat.x + yy)) * 180 / PI;
    pitch = asin(2 * quat.w * quat.y - quat.x * quat.z) * 180 / PI;
    yaw = atan2(2 * (quat.w * quat.z + quat.x * quat.y), 1 - 2*(yy+quat.z * quat.z)) * 180 / PI;

}

void quatToEuler(double &yaw, double &pitch, double &roll){

    imu.get_quaternion(&quat);

    double ww = quat.w * quat.w;
    double xx = quat.x * quat.x;
    double yy = quat.y * quat.y;
    double zz = quat.z * quat.z;

    yaw = atan2(2 * (quat.x * quat.y + quat.z * quat.w), (xx - yy - zz + ww)) * 180/ PI;
    pitch = asin(-2 * (quat.x * quat.z - quat.y * quat.w) / (xx + yy + zz + ww)) * 180 / PI;
    roll = atan2(2 * (quat.y * quat.z + quat.x * quat.w), (-xx - yy + zz + ww)) * 180 / PI;


}

int main(){

    imu.change_fusion_mode(MODE_IMU);
    int i = 0;
    double yaw, pitch, roll;

    unsigned long timer = 0, sTimer = 0;

    printf("Bosch Sensortec BNO055 test program on " __DATE__ "/" __TIME__ "\n");

    if (imu.chip_ready() == 0)
        printf("Bosch BNO055 is NOT available!!\r\n");

    imu.read_id_inf(&bno055_id_inf);
    printf("CHIP:0x%02x, ACC:0x%02x, MAG:0x%02x, GYR:0x%02x, , SW:0x%04x, , BL:0x%02x\n",
           bno055_id_inf.chip_id, bno055_id_inf.acc_id, bno055_id_inf.mag_id,
           bno055_id_inf.gyr_id, bno055_id_inf.sw_rev_id, bno055_id_inf.bootldr_rev_id);

    for(i = 0; i < 10; i++){
        timer = us_ticker_read();
        getAnglesFromQuat(yaw, pitch, roll);
        printf("Heading:%ld [deg], Roll:%d [deg], Pitch:%d [deg]     ", (int)yaw, (int)roll, (int)pitch);
        timer = us_ticker_read() - timer;
        printf("Time:%ld [us]\n", timer);
        sTimer += timer;

//        timer = us_ticker_read();
//        quatToEuler(yaw, pitch, roll);
//        printf("Heading:%ld [deg], Roll:%d [deg], Pitch:%d [deg]     ", (int)yaw, (int)roll, (int)pitch);
//        timer = us_ticker_read()- timer;
//        printf("Time:%ld [us]\n\n", timer);

        ThisThread::sleep_for(100ms);
    }
    printf("Average time:%ld [us]\n", sTimer/i);
}

