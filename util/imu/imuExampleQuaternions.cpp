#include <cstdlib>
#include "mbed.h"
#include "BNO055.h"
#include "BNO055.cpp"

I2C    i2c(PB_9, PB_8);                // SDA, SCL
BNO055 imu(i2c, PA_8);

BNO055_ID_INF_TypeDef bno055_id_inf;
BNO055_QUATERNION_TypeDef   quat;

void getAnglesFromQuat(double &yaw, double &pitch, double &roll){

    imu.get_quaternion(&quat);
    double yy = quat.y * quat.y;        // 2 Uses below

    roll = atan2(2 * (quat.w * quat.x + quat.y * quat.z), 1 - 2*(quat.x * quat.x + yy));
    pitch = asin(2 * quat.w * quat.y - quat.x * quat.z);
    yaw = atan2(2 * (quat.w * quat.z + quat.x * quat.y), 1 - 2*(yy+quat.z * quat.z));

    /*  Convert Radians to Degrees */
    roll    *= 57.2958;
    pitch   *= 57.2958;
    yaw     *= 57.2958;

}

int main(){

    imu.change_fusion_mode(MODE_NDOF);

    double yaw, pitch, roll;

    printf("Bosch Sensortec BNO055 test program on " __DATE__ "/" __TIME__ "\n");

    if (imu.chip_ready() == 0)
        printf("Bosch BNO055 is NOT available!!\r\n");

    imu.read_id_inf(&bno055_id_inf);
    printf("CHIP:0x%02x, ACC:0x%02x, MAG:0x%02x, GYR:0x%02x, , SW:0x%04x, , BL:0x%02x\n",
           bno055_id_inf.chip_id, bno055_id_inf.acc_id, bno055_id_inf.mag_id,
           bno055_id_inf.gyr_id, bno055_id_inf.sw_rev_id, bno055_id_inf.bootldr_rev_id);

    while (true) {
        getAnglesFromQuat(yaw, pitch, roll);
        printf("Heading:%d [deg], Roll:%d [deg], Pitch:%d [deg]\n", (int)yaw, (int)roll, (int)pitch);

        ThisThread::sleep_for(100ms);
    }
}

