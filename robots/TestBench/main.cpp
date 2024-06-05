#include "main.h"

DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);
DigitalOut ledbuiltin(LED1);
DJIMotor indexer_L(8, CANHandler::CANBUS_2, C610);
DJIMotor indexer_R(5, CANHandler::CANBUS_2, C610);
DJIMotor pitch(6, CANHandler::CANBUS_2, GIMBLY); // right
// I2C i2c(I2C_SDA, I2C_SCL);
// BNO055 imu(i2c, IMU_RESET, MODE_IMU);

// ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2286); // radius is 9 in
DJIMotor wheel1(1, CANHandler::CANBUS_1, M3508);
DJIMotor wheel2(2, CANHandler::CANBUS_1, M3508);
DJIMotor wheel3(3, CANHandler::CANBUS_1, M3508);
DJIMotor wheel4(4, CANHandler::CANBUS_1, M3508);



//I2C i2c(I2C_SDA, I2C_SCL);

//BNO055 imu(i2c, IMU_RESET, MODE_IMU);
//ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2286); // radius is 9 in
DJIMotor yaw(5, CANHandler::CANBUS_1, GIMBLY);
//DJIMotor pitch(6, CANHandler::CANBUS_2, GIMBLY); // right
// DJIMotor yaw2(7, CANHandler::CANBUS_2, GIMBLY); // left, not plugged in

//DJIMotor indexer_L(8, CANHandler::CANBUS_2, C610);
DJIMotor UP_FLYWHEEL_R(4, CANHandler::CANBUS_2, M3508);
DJIMotor UP_FLYWHEEL_L(3, CANHandler::CANBUS_2, M3508);

//DJIMotor indexer_R(7, CANHandler::CANBUS_2, C610);
DJIMotor DOWN_FLYWHEEL_L(1, CANHandler::CANBUS_2, M3508);
DJIMotor DOWN_FLYWHEEL_R(2, CANHandler::CANBUS_2, M3508);



// DJIMotor testMot(4, CANHandler::CANBUS_1, M3508, "testbench_motor");

int main(){

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    int refLoop = 0;

   // testMot.setSpeedPID(1.5,0,0);

    while(true){
        timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > 25){
            loopTimer = timeStart;
            led = !led;
            ledbuiltin = !ledbuiltin;

            refLoop++;
            if (refLoop >= 5){
                //refereeThread(&referee);
                refLoop = 0;
                led2 = !led2;
                //printff("%d %d %d %d [%d %d %d %d]\n", pitch>>ANGLE, indexer_L>>ANGLE, indexer_R >> ANGLE,  yaw>>ANGLE, UP_FLYWHEEL_L>>ANGLE, UP_FLYWHEEL_R>>ANGLE, DOWN_FLYWHEEL_L>>ANGLE, DOWN_FLYWHEEL_R>>ANGLE);
                // wheel.setPower(5000);
                printff("%d %d %d %d %d\n", wheel1>>ANGLE, wheel2>>ANGLE, wheel3>>ANGLE, wheel4>>ANGLE, yaw>>ANGLE);
                //printff("ang%f t%d d%f FF%f\n", (((pitch>>ANGLE) - InitialOffset_Ticks) / TICKS_REVOLUTION) * 360, pitch>>ANGLE, desiredPitch, K * sin((desiredPitch / 180 * PI) - pitch_phase)); //(desiredPitch / 360) * TICKS_REVOLUTION + InitialOffset_Ticks
                // printff("pitch: %d\n", pitch>>ANGLE);
                //indexer_R.setPower(5000);
                //indexer_L.setPower(5000);
                //printff("%d %d\n",indexer_R>>ANGLE, indexer_L>>ANGLE );
                //printff("datum:%d %d %d\n", testMot>>ANGLE, testMot>>VELOCITY, remote.leftX());

            }
            try{
                remoteRead();
            }catch(exception e){

            }
               

            //testMot.setPower(remote.leftX() * 3);
            
            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}