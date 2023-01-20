#include <cstdlib>
#include <iostream>
#include "mbed.h"
#include "../src/main.hpp"
#include "../util/imu/BNO055.h"
#include "../util/imu/BNO055.cpp"

I2C    i2c(PB_9, PB_8);                // SDA, SCL
BNO055 imu(i2c, PA_8);

BNO055_ID_INF_TypeDef bno055_id_inf;
BNO055_EULER_TypeDef  euler_angles;
BNO055_QUATERNION_TypeDef   quat;

// sentry only from 1230 1830

CANMotor m3508_1(1,NewCANHandler::CANBUS_1,M3508);
CANMotor m3508_2(2,NewCANHandler::CANBUS_1,M3508);
CANMotor m3508_3(3,NewCANHandler::CANBUS_1,M3508);
CANMotor m3508_4(4,NewCANHandler::CANBUS_1,M3508);
CANMotor m3508_5(5,NewCANHandler::CANBUS_1,M3508);
CANMotor m2006_7(7,NewCANHandler::CANBUS_1,M2006);
CANMotor gimbly8(4,NewCANHandler::CANBUS_1,GIMBLY);
CANMotor gimbly9(5,NewCANHandler::CANBUS_1,GIMBLY);
CANMotor gimbly10(6,NewCANHandler::CANBUS_1,GIMBLY);
CANMotor gimbly11(7,NewCANHandler::CANBUS_1,GIMBLY);


DigitalOut led(LED1);
DigitalIn button(BUTTON1);



CANMotor getMotor(int i){
    switch (i) {
        
        case 1:
            return m3508_1;
            
        case 2:
            return m3508_2;
            
        case 3:
            return m3508_3;
            
        case 4:
            return m3508_4;
            
        case 5:
            return m3508_5;
            
        case 6:
            return m3508_1;
            
        case 7:
            return m3508_1;
            
        case 8:
            return gimbly8;
            
        case 9:
            return gimbly9;
            
        case 10:
            return gimbly10;
            
        case 11:
            return gimbly11;
            
        default:
            return m3508_1;
            
    }
}

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
    CANMotor::setCANHandlers(&canHandler1,&canHandler2, false, false);

    double yaw, pitch, roll;
    int countLoops = 0, refLoop = 0;
    unsigned long loopTimer = us_ticker_read() / 1000;

    printf("Bosch Sensortec BNO055 test program on " __DATE__ "/" __TIME__ "\n");

     if (imu.chip_ready() == 0)
         printf("Bosch BNO055 is NOT available!!\r\n");

     imu.read_id_inf(&bno055_id_inf);
     printf("CHIP:0x%02x, ACC:0x%02x, MAG:0x%02x, GYR:0x%02x, , SW:0x%04x, , BL:0x%02x\n",
                bno055_id_inf.chip_id, bno055_id_inf.acc_id, bno055_id_inf.mag_id,
                bno055_id_inf.gyr_id, bno055_id_inf.sw_rev_id, bno055_id_inf.bootldr_rev_id);

    while (true) {
        unsigned long timeStart = us_ticker_read() / 1000;

        getAnglesFromQuat(yaw, pitch, roll);
        //imu.get_euler_angles(&euler_angles);
        //imu.get_quaternion(&quat);

        if(timeStart - loopTimer > 25){
            loopTimer = timeStart;

            led = !led;

            remoteRead();
            refLoop++;

            if(refLoop > 25)
                refLoop = 0;

            printf("Heading:%d [deg], Roll:%d [deg], Pitch:%d [deg]\n", (int)yaw, (int)roll,(int)pitch);
            //printf("Heading:%d [deg], Roll:%d [deg], Pitch:%d [deg]\n", (int)euler_angles.h, (int)euler_angles.r, (int)euler_angles.p);
            //printf("Quaternion Y:%d [deg], X:%d [deg], Z:%d [deg], W:%d [deg]\n", (int)quat.y, (int)quat.x, (int)quat.z, (int)quat.w);

            CANMotor::sendValues();
        }


        CANMotor::getFeedback();
        ThisThread::sleep_for(1ms);
        countLoops ++;
        
    }
}

