////#include "main.h"
////
////DigitalOut led(L27);
////DigitalOut led2(L26);
////DigitalOut led3(L25);
////DigitalOut ledbuiltin(LED1);
////DJIMotor indexer_L(8, CANHandler::CANBUS_2, C610);
////DJIMotor indexer_R(5, CANHandler::CANBUS_2, C610);
////DJIMotor pitch(6, CANHandler::CANBUS_2, GIMBLY); // right
////I2C i2c(I2C_SDA, I2C_SCL);
////BNO055 imu(i2c, IMU_RESET, MODE_IMU);
////
//////ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2286); // radius is 9 in
////DJIMotor wheel1(1, CANHandler::CANBUS_1, M3508);
////DJIMotor wheel2(2, CANHandler::CANBUS_1, M3508);
////DJIMotor wheel3(3, CANHandler::CANBUS_1, M3508);
////DJIMotor wheel4(4, CANHandler::CANBUS_1, M3508);
////
////
////
//////I2C i2c(I2C_SDA, I2C_SCL);
////
//////BNO055 imu(i2c, IMU_RESET, MODE_IMU);
//////ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2286); // radius is 9 in
////DJIMotor yaw(7, CANHandler::CANBUS_1, GIMBLY);
//////DJIMotor pitch(6, CANHandler::CANBUS_2, GIMBLY); // right
////// DJIMotor yaw2(7, CANHandler::CANBUS_2, GIMBLY); // left, not plugged in
////
//////DJIMotor indexer_L(8, CANHandler::CANBUS_2, C610);
////DJIMotor UP_FLYWHEEL_R(4, CANHandler::CANBUS_2, M3508);
////DJIMotor UP_FLYWHEEL_L(3, CANHandler::CANBUS_2, M3508);
////
//////DJIMotor indexer_R(7, CANHandler::CANBUS_2, C610);
////DJIMotor DOWN_FLYWHEEL_L(1, CANHandler::CANBUS_2, M3508);
////DJIMotor DOWN_FLYWHEEL_R(2, CANHandler::CANBUS_2, M3508);
////
////
////
////// DJIMotor testMot(4, CANHandler::CANBUS_1, M3508, "testbench_motor");
////
////int main(){
////
////    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
////    DJIMotor::s_sendValues();
////    DJIMotor::s_getFeedback();
////
////    unsigned long timeStart;
////    unsigned long loopTimer = us_ticker_read();
////    int refLoop = 0;
////
////   // testMot.setSpeedPID(1.5,0,0);
////
////    while(true){
////        timeStart = us_ticker_read();
////
////        if ((timeStart - loopTimer) / 1000 > 25){
////            loopTimer = timeStart;
////            led = !led;
////            ledbuiltin = !ledbuiltin;
////
////            refLoop++;
////            if (refLoop >= 5){
////                //refereeThread(&referee);
////                refLoop = 0;
////                led2 = !led2;
////                //printff("%d %d %d %d [%d %d %d %d]\n", pitch>>ANGLE, indexer_L>>ANGLE, indexer_R >> ANGLE,  yaw>>ANGLE, UP_FLYWHEEL_L>>ANGLE, UP_FLYWHEEL_R>>ANGLE, DOWN_FLYWHEEL_L>>ANGLE, DOWN_FLYWHEEL_R>>ANGLE);
////                //printff("%d\n", indexer_L >> ANGLE);
////
////                // wheel.setPower(5000);
////                //printff("%d %d %d %d %d\n", wheel1>>ANGLE, wheel2>>ANGLE, wheel3>>ANGLE, wheel4>>ANGLE, yaw>>ANGLE);
////                //printff("ang%f t%d d%f FF%f\n", (((pitch>>ANGLE) - InitialOffset_Ticks) / TICKS_REVOLUTION) * 360, pitch>>ANGLE, desiredPitch, K * sin((desiredPitch / 180 * PI) - pitch_phase)); //(desiredPitch / 360) * TICKS_REVOLUTION + InitialOffset_Ticks
////                // printff("pitch: %d\n", pitch>>ANGLE);
////                //indexer_R.setPower(5000);
////                //indexer_L.setPower(5000);
////                //printff("%d %d\n",indexer_R>>ANGLE, indexer_L>>ANGLE );
////                //printff("datum:%d %d %d\n", testMot>>ANGLE, testMot>>VELOCITY, remote.leftX());
////
////                printff("%d\n", pitch>>ANGLE);
////            }
////            remoteRead();
////            // try{
////            //     remoteRead();
////            // }catch(exception e){
////
////            // }
////
////
////            //testMot.setPower(remote.leftX() * 3);
////
////            DJIMotor::s_sendValues();
////        }
////        DJIMotor::s_getFeedback();
////        ThisThread::sleep_for(1ms);
////    }
////}
//
//
//#include "mbed.h"
//#include "TestBench.h"
//#include "subsystems/ChassisSubsystem.h"
//#include "../core/util/peripherals/imu/BNO055.h"
//
////Serial pc(USBTX, USBRX);
//
//#define D_SDA                  PB_7
//#define D_SCL                  PB_6
//// sda=PB7, scl=PB_6 Pins specific to Nucleo-F303K8
//// must change pins to match your board.
//
//I2C i2c(D_SDA, D_SCL);
//
//DigitalOut myled(LED1);
//
//int ack;
//int address;
//void scanI2C() {
//for(address=1;address<127;address++) {
//ack = i2c.write(address, "11", 1);
//if (ack == 0) {
////pc.printf("\tFound at %3d -- %3x\r\n", address,address);
//printf("\tFound at %3d -- %3x\r\n", address,address);
//} else {
//    printf("Not found \n");
//}
//ThisThread::sleep_for(0.05);
//}
//}
//
//
//
//int main() {
//    BNO055 imu = BNO055();
//
////pc.baud(9600);
////pc.printf("I2C scanner \r\n");
//printf("I2C scanner \r\n");
//scanI2C();
//printf("Finished Scan\r\n");
//// just blink to let us know the CPU is alive
//while(1) {
//ThisThread::sleep_for(5.0);
//myled = !myled;
//scanI2C();
//}
//}

#include "main.h"
#include "../core/util/peripherals/imu/BNO055.h"

DigitalOut led(L26);
DigitalOut led2(L27);
DigitalOut led3(L25);

I2C i2c(I2C_SDA, I2C_SCL);
BNO055 imuu = BNO055(i2c, int32_t(55), 0x29);
//DEFINE MOTORS, ETC
BNO055_ANGULAR_POSITION_typedef imuAngles;
//struct YesWei{
//    int16_t acc_x;
//    int8_t acc_x_msb;
//    int8_t acc_y_lsb;
//    int8_t acc_y_msb;
//    int8_t acc_z_lsb;
//    int8_t acc_z_msb;
//};
//I2C i2c(PB_7, PB_8);
int main(){
//assigning can handler objects to motor class.
    DJIMotor::s_setCANHandlers(&canHandler1,&canHandler2, false, false);

    //getting initial feedback.
    DJIMotor::s_getFeedback();

    unsigned long loopTimer_u = us_ticker_read();
    unsigned long timeEnd_u;
    unsigned long timeStart_u;

    int refLoop = 0;

    //while (1) {
    //}
    while (!imuu.begin()){
        printff("No imu detected.\n");
    }
    //while (1) {
    //printff("1\n");
    //}
    //DEFINE PIDs AND OTHER CONSTANTS
    //imuu.getEvent(&orientationData, BNO055::VECTOR_EULER);

    while(true){ //main loop
    //imu.get_angular_position_quat(&imuAngles);
        timeStart_u = us_ticker_read();
        sensors_event_t* orientationData;
        sensors_event_t accelerationData;
    //    imuu.getEvent(accelerationData, BNO055::VECTOR_LINEARACCEL);
    //    imuu.getEvent((&accelerationData));

        //opr mode
        char opr[2] = {0x3D, 0x00};
        i2c.write(0x29 << 1, opr, 2, true);
        //sys trigger
        opr[0] = 0x3F;
        opr[1] = 0x20;
        i2c.write(0x29 << 1, opr, 2, true);
        //powermode
        opr[0] = 0x3E;
        opr[1] = 0x00;
        i2c.write(0x29 << 1, opr, 2, true);
        //page id
        opr[0] = 0x07;
        opr[1] = 0x00;
        i2c.write(0x29 << 1, opr, 2, true);
//        sys trigger
        opr[0] = 0x3F;
        opr[1] = 0x00;
        i2c.write(0x29 << 1, opr, 2, true);
        // opr mode (setmode)
        opr[0] = 0x3D;
        opr[1] = 0x0C;
        i2c.write(0x29 << 1, opr, 2, true);
        //inner loop runs every 25ms
        if((timeStart_u - loopTimer_u) / 1000 > 25) {
            loopTimer_u = timeStart_u;
            led = !led; //led blink tells us how fast the inner loop is running

            if (refLoop >= 25) { //ref code runs 5 of every inner loop,
                refLoop = 0;
                refereeThread(&referee);

//                opr[0] = 0x3D;
//                opr[1] = 0x00;
//                i2c.write(0x29 << 1, opr, 2, true);
//                opr[1] = 0x0C;
//                i2c.write(0x29 << 1, opr, 2, true);



                int len = 32;
                char data[32] = {0};
                char data_gyro[32] = {0};


                opr[0] = 0x3D;
                i2c.write(0x29 << 1, opr, 1, true);
                ThisThread::sleep_for(20ms);
                i2c.read(0x29 << 1, opr, 2, false);
                ThisThread::sleep_for(20ms);
                printff("%d %d : ", opr[0], opr[1]);

                opr[0] = 0x07;
                i2c.write(0x29 << 1, opr, 1, true);
                ThisThread::sleep_for(20ms);
                i2c.read(0x29 << 1, opr, 2, false);
                ThisThread::sleep_for(20ms);
                printff("%d %d : ", opr[0], opr[1]);



                data[0] = 0x08;
                data[1] = 0;
                i2c.write(0x29 << 1, data, 1, true);
                ThisThread::sleep_for(20ms);
                i2c.read(0x29 << 1, data, len, false);
                ThisThread::sleep_for(20ms);
//                printff("%d %d : ", opr[0], opr[1]);
                for(int i = 0; i < 6; i += 2){
//                    printff("%x %x %.1f       ", data[i], data[i+1], (data[i + 1] & 0b10000000)? - float(data[i] | ((data[i + 1]&0b01111111) << 8)):float(data[i] | ((data[i + 1]&0b01111111) << 8)));
                    printff("%x %x %.1f       ", data[i], data[i+1], float(static_cast<int16_t>(data[0] << 8 | data[1])));
                }
                data_gyro[0] = 0x14;
                data_gyro[1] = 0;
                i2c.write(0x29 << 1, data_gyro, 1, true);
                ThisThread::sleep_for(20ms);
                i2c.read(0x29 << 1, data_gyro, len, false);
                ThisThread::sleep_for(20ms);
                for(int i = 0; i < 6; i += 2){
//                    printff("%x %x %.1f       ", data_gyro[i], data_gyro[i+1], float(data_gyro[i] | (data_gyro[i + 1] << 8)));
                    printff("%x %x %.1f       ", data_gyro[i], data_gyro[i+1], float(static_cast<int16_t>(data_gyro[0] << 8 | data_gyro[1])));

                }
                printff("\n");


            //    imuu.get_angular_position_quat(&imuAngles);
//                sensors_event_t orientationData;
//                if (imuu.getEvent((&accelerationData))){
//            //        printff("angle %f %f %f %f %f %f\n", imuAngles.yaw, imuAngles.roll, imuAngles.pitch, orientationData.orientation.x, orientationData.orientation.y, orientationData.orientation.z);
////                    printff("angle %f %f %f %f %f %f\n", accelerationData.acceleration.x, accelerationData.acceleration.y, accelerationData.acceleration.z, accelerationData.orientation.x, accelerationData.orientation.y, accelerationData.orientation.z);
//                }
    //    int write(int address, const char *data, int length, bool repeated = false);
    //    const char* data =
    //    char data[] = "hello world";
    //    char     dt[10];      // working buffer
    //    dt[0] = 0x20;
    //    dt[1] = 1;
    //    i2c.write(0x28, dt, 2, false);
    //    char data1[] = "nope       ";
    //    char dtt[10];
    //    dtt[0] = 0X08; //BNO055_ACCEL_DATA_X_LSB_ADDR
    //    dtt[1] = 0;
    //    i2c.write(0x28, dtt, 1, true);
    //    i2c.read(0x28, dtt, 6, false);
    ////    printff("Read test: %d %d %d \n", dtt[0] << 1, dtt[1] << 1, dtt[2] << 1);
    //    int16_t x = dtt[1] << 8 | dtt[0];
    //    int16_t y = dtt[3] << 8 | dtt[2];
    //    int16_t z = dtt[5] << 8 | dtt[4];
    ////    printff("Read test: %f %f %f  %f %d %d\n", dtt[0], dtt[1], dtt[2], dtt[3], dtt[4], dtt[5]);
    //    printff("Read test: %f %f %f\n", (double)x, (double)y, (double)z);

            }
            refLoop ++;

            remoteRead(); //reading data from remote

            //MAIN CODE
            //MOST CODE DOESNT NEED TO RUN FASTER THAN EVERY 25ms

            timeEnd_u = us_ticker_read();

            DJIMotor::s_sendValues();
        }

    //FEEDBACK CODE DOES NEED TO RUN FASTER THAN 1MS
    //OTHER QUICK AND URGENT TASKS GO HERE

        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}