#include <cstdlib>
#include "mbed.h"
#include "util/peripherals/imu/BNO055.h"

I2C    i2c(PB_9, PB_8);                // SDA, SCL
BNO055 imu(i2c, PA_8, MODE_IMU);

BNO055_ID_INF_TypeDef bno055_id_inf;
BNO055_QUATERNION_TypeDef       quat;
BNO055_ANGULAR_POSITION_typedef p;

int main(){

    imu.set_mounting_position(MT_P1);

    printf("Bosch Sensortec BNO055 test program on " __DATE__ "/" __TIME__ "\n");

    if (imu.chip_ready() == 0)
        printf("Bosch BNO055 is NOT available!!\r\n");

    imu.read_id_inf(&bno055_id_inf);
    printf("CHIP:0x%02x, ACC:0x%02x, MAG:0x%02x, GYR:0x%02x, , SW:0x%04x, , BL:0x%02x\n",
           bno055_id_inf.chip_id, bno055_id_inf.acc_id, bno055_id_inf.mag_id,
           bno055_id_inf.gyr_id, bno055_id_inf.sw_rev_id, bno055_id_inf.bootldr_rev_id);

    while (true) {
        imu.get_angular_position_quat(&p);
        printf("Heading:%d [deg], Roll:%d [deg], Pitch:%d [deg]\n", (int)p.yaw, (int)p.roll, (int)p.pitch);

        ThisThread::sleep_for(100ms);
    }
}

