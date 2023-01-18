#include "mbed.h"
#include "BNO055.h"
#include "BNO055.cpp"

#include <stdio.h>

 I2C    i2c(PB_9, PB_8);                // SDA, SCL
 BNO055 imu(i2c, PA_8);

 BNO055_ID_INF_TypeDef bno055_id_inf;
 BNO055_EULER_TypeDef  euler_angles;

 int main() {

     ThisThread::sleep_for(3000ms);
     printf("Bosch Sensortec BNO055 test program on " __DATE__ "/" __TIME__ "\n");

     if (imu.chip_ready() == 0)
         printf("Bosch BNO055 is NOT available!!\r\n");

     imu.read_id_inf(&bno055_id_inf);
     printf("CHIP:0x%02x, ACC:0x%02x, MAG:0x%02x, GYR:0x%02x, , SW:0x%04x, , BL:0x%02x\n",
                bno055_id_inf.chip_id, bno055_id_inf.acc_id, bno055_id_inf.mag_id,
                bno055_id_inf.gyr_id, bno055_id_inf.sw_rev_id, bno055_id_inf.bootldr_rev_id);

     while(true) {
         imu.get_euler_angles(&euler_angles);
         printf("Heading:%f [deg], Roll:%f [deg], Pitch:%f [deg]\n", euler_angles.h, euler_angles.r, euler_angles.p);
         ThisThread::sleep_for(400ms);
     }
 }

