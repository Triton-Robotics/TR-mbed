#include "main.h"

DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);
//find new motor ids
//DJIMotor motor1(1, CANHandler::CANBUS_1, M3508);
//DJIMotor motor2(2, CANHandler::CANBUS_1, M3508);
//DJIMotor motor3(3, CANHandler::CANBUS_1, M3508);
DJIMotor motor1(1, CANHandler::CANBUS_1, M3508);
DJIMotor motor2(2, CANHandler::CANBUS_1, M3508);
DJIMotor motor3(3, CANHandler::CANBUS_1, M3508);
DJIMotor motor4(4, CANHandler::CANBUS_1, M3508);
DJIMotor motor5(5, CANHandler::CANBUS_1, M3508);
DJIMotor motor6(6, CANHandler::CANBUS_1, M3508);
DJIMotor motor7(7, CANHandler::CANBUS_2, GM6020);
DJIMotor motor8(8, CANHandler::CANBUS_1, M3508);
//I2C i2c(I2C_SDA, I2C_SCL);
//Chassis chassis(1, 2, 3, 4, &i2c);
//DJIMotor::initializedWarning = false;

void pitchSetPosition(){
    int pitchSetPoint = 2900-(2900-1340)/660.0*remote.rightY();
    if(pitchSetPoint>2900)
        pitchSetPoint = 2900;

    else if(pitchSetPoint < 1340)
        pitchSetPoint = 1340;

    motor7.setPosition(pitchSetPoint);
}


int main(){

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();

    DJIMotor::initializedWarning = false;

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    int refLoop = 0;

    uint16_t max_power;
    float ref_chassis_power;
    int ref_yaw;
    double rotationalPower;

//on can1 bus with receiver in can1
    //tune motors one at a time
//moto1 tuned
   motor1.setSpeedPID(.9, 1 ,1);
////5,0,0
    motor2.setSpeedPID(.9,1,1);
//    //pid looks good
    motor3.setSpeedPID(.8,0,0);
    motor4.setSpeedPID(.6,0,0);
    motor5.setSpeedPID(.9,0,0);
    //motor1.setPower(-1000);
//    motor7.setSpeedPID(.2, .0001, .001);
//    motor7.pidSpeed.setIntegralCap(250/.0001);
//    motor7.getData(VELOCITY);
//    motor7.setPower(10000);
    motor7.setPositionPID(30, .0001, 1000);
    motor7.pidSpeed.setIntegralCap(6000);
    //motor2.setPower(-1000);
    //motor2.setSpeed(-200);
    motor2.getData(VELOCITY);
    //motor3.setPower(-500);
    //motor3.setSpeed(-200);
    motor3.getData(VELOCITY);
    //motor4.setPower(-500);
    //motor4.setSpeed(-200);
    motor4.getData(VELOCITY);
    //first motor5 PID
    //motor5.setSpeedPID(.1,1,1);
    //second motor5 PID .5 .0001 .001
   motor5.setSpeedPID(.2, .0001, .001);
   //good integral cap is 300/.0001
   //motor 2 250/.0001
   motor5.pidSpeed.setIntegralCap(250/.0001);
    //motor5.setPower(0);
    //motor5.setSpeed(-1000);
    //motor5.setPosition(5);
    motor5.getData(VELOCITY);
//    motor6.setPower(-500);
//    motor6.getData(VELOCITY);
//    motor7.setPower(-500);
//    motor7.getData(VELOCITY);
//    motor8.setPower(-500);
//    motor8.getData(VELOCITY);
//    motor5.setSpeed(-900);
//    motor5.isConnected();




    while(true){
        timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > 25) {
            led = !led;

            refLoop++;
            if (refLoop >= 5) {
                refereeThread(&referee);
                refLoop = 0;
                led2 = !led2;
                //printff("%d %d %d\n",motor3>>VELOCITY,-100,motor3>>POWEROUT);
            }

            remoteRead();

            //printff("%u\n", (uint16_t)(motor5.getData(ANGLE)));
            ref_chassis_power = ext_power_heat_data.data.chassis_power;
            max_power = ext_game_robot_state.data.chassis_power_limit;
            ref_yaw = ext_game_robot_pos.data.yaw;
            printff("%d %d %d\n",motor7>>ANGLE,-100,motor7>>POWEROUT);
            if (remote.leftSwitch() == Remote::SwitchState::UP){
                //motor1.setPower(1500);
                //     motor2.setPower(1500);
                // motor3.setPower(1500);
                //motor4.setPower(1500);
                //motor1.getData(VELOCITY);
//                motor2.getData(VELOCITY);

//
               motor5.getData(VELOCITY);

//
//            motor1.setSpeed(100);
//            motor2.setSpeed(100);
//            motor3.setSpeed(100);
//            motor4.setSpeed(100);
            }//max 1340 min 2900
            if (remote.rightSwitch() == Remote::SwitchState::UP) {
                //motor7.setPosition(1500);
                //motor7.setPositionPID(18, .0001, 850);

                motor7.getData(VELOCITY);
                pitchSetPosition();
                //motor7.setSpeed(200);
                // motor2.setSpeedPID(4, 6 ,8);
                // motor3.setSpeedPID(8, 7 ,5);
                //motor4.setSpeedPID(5,  2,3);
            }
            if (remote.rightSwitch() == Remote::SwitchState::DOWN) {
               // motor7.setPosition(2500);
                //motor7.setPositionPID(.8, .0001, .001);
                //motor7.pidSpeed.setIntegralCap(250/.0001);
//                motor7.getData(VELOCITY);
//                motor7.setPower(5000);
                pitchSetPosition();
//                motor3.setSpeed(-900);
//                motor1.setSpeed(-900);
//                motor2.setSpeed(-900);
               motor5.setSpeed(-900);

//            motor2.setPower(-700);
//            motor3.setPower(-700);
//            motor4.setPower(-700);
            }else if((remote.rightSwitch() == Remote::SwitchState::MID) || (remote.rightSwitch() == Remote::SwitchState::UNKNOWN)){
                motor7.setPower(0);
//                motor1.setPower(0);
//                motor2.setPower(0);
//                motor3.setPower(0);
//                motor4.setPower(0);

            }
            if (remote.leftSwitch() == Remote::SwitchState::DOWN){
//                motor1.setPower(0);
//                motor2.setPower(0);
//                motor3.setPower(0);
//                motor4.setPower(0);


//            motor2.setPower(0);
//            motor3.setPower(0);
//            motor4.setPower(0);
//
//            motor1.setSpeed(100);
//            motor2.setSpeed(100);
//            motor3.setSpeed(100);
           motor5.setSpeed(100);
            }

            loopTimer = timeStart;
            //chassis.driveTurretRelativePower(ref_chassis_power, max_power, {-remote.leftX() * 5.0, -remote.leftY() * 5.0}, -yaw1.getData(MULTITURNANGLE) * 360.0 / 8192 + 90, int(loopEnd - timeStart), rotationalPower);
            //chassis.driveXYR1(4*remote.leftX(),4*remote.leftY(),4*remote.rightX());

            //printff("%d\n", chassis.getMotor(4).getData(POWEROUT));
            //printff("%d\n", motor5.getData(VELOCITY));
            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}