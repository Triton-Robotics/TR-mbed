////#include "main.h"
////
////
////DJIMotor wheel(3, CANHandler::CANBUS_1, M3508);
////DJIMotor wheel_2(1, CANHandler::CANBUS_1, M3508);
////DJIMotor indexer(7, CANHandler::CANBUS_2, C610);
////DJIMotor barrel(4, CANHandler::CANBUS_2, M2006);
////DJIMotor RFLYWHEEL(8, CANHandler::CANBUS_2, M3508);
////DJIMotor LFLYWHEEL(5, CANHandler::CANBUS_2, M3508);
////DigitalOut led(L27);
////
////int main(){
////    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
////
////
////    unsigned long timeStart;
////    unsigned long loopTimer = us_ticker_read();
////
////    while(true){
////        timeStart = us_ticker_read();
////
////        int i = 0;
////        //wheel.setSpeed(lY*5);
////        if ((timeStart - loopTimer)/ 1000 > 25){
////            //int++;
////            led = !led;
////            remoteRead();
////
////            indexer.setPower(lY*3);
////            if (lS == Remote::SwitchState::UP){
////                barrel.setPower(-1100);
////            } else {
////                barrel.setPower(1100);
////            }
////
////
////            /*
////            if (rS == Remote::SwitchState::UP){
////                RFLYWHEEL.setSpeed(80);
////                LFLYWHEEL.setSpeed(-80);
////            } else {
////                RFLYWHEEL.setSpeed(0);
////                LFLYWHEEL.setSpeed(0);
////            }*/
////
////
////            //printf("working %d\n ", indexer>>VELOCITY);
////            //if (i == )
////            //printf("%d-%d\n", ext_power_heat_data.data.shooter_id1_17mm_cooling_heat, ext_power_heat_data.data.shooter_id2_17mm_cooling_heat);
////
////
////            loopTimer = timeStart;
////            DJIMotor::s_sendValues();
////        }
////
////
////        DJIMotor::s_getFeedback();
////        ThisThread::sleep_for(1ms);
////    }
////}
////
//
////#include "main.h"
////
////
////DJIMotor yaw1(5, CANHandler::CANBUS_1, GIMBLY);
////DJIMotor yaw2(7, CANHandler::CANBUS_1, GIMBLY);
////DJIMotor pitch(6, CANHandler::CANBUS_2, GIMBLY);
////DJIMotor indexerL(7, CANHandler::CANBUS_2, C610);
////DJIMotor indexerR(8, CANHandler::CANBUS_2, C610);
////DJIMotor RTOPFLYWHEEL(1, CANHandler::CANBUS_2, M3508_FLYWHEEL);
////DJIMotor LTOPFLYWHEEL(2, CANHandler::CANBUS_2, M3508_FLYWHEEL);
////DJIMotor RBOTTOMFLYWHEEL(4, CANHandler::CANBUS_2, M3508_FLYWHEEL);
////DJIMotor LBOTTOMFLYWHEEL(3, CANHandler::CANBUS_2, M3508_FLYWHEEL);
////
////
////DigitalOut led(L26);
////
////int main(){
////
////    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
////
////    int lflypid = 0;
////    int rflypid = 0;
////    //int flywheelspd = 1000;
////
////
////    unsigned long timeStart;
////    unsigned long loopTimer = us_ticker_read();
////
////    //LFLYWHEEL.setSpeedPID(1, 0, 0);
////    //RFLYWHEEL.setSpeedPID(1, 0, 0);
////
////    RTOPFLYWHEEL.setSpeedPID(16, 0, 0);
////    LTOPFLYWHEEL.setSpeedPID(16, 0, 0);
////    RBOTTOMFLYWHEEL.setSpeedPID(16, 0, 0);
////    LBOTTOMFLYWHEEL.setSpeedPID(16, 0, 0);
////
////
////    while(true){
////        timeStart = us_ticker_read();
//
//
//        //wheel.setSpeed(lY*5);
//        if ((timeStart - loopTimer)/ 1000 > 25){
//            led = !led;
//            remoteRead();
//            printf("information: %d-%d\n", LTOPFLYWHEEL>>VELOCITY, LBOTTOMFLYWHEEL>>VELOCITY);
//            indexerL.setPower(lY*4);
//
//            //indexer.setPower(lY*5);
//
//
//
//            if (Remote::SwitchState::UP == lS){
////                RTOPFLYWHEEL.setSpeed(flywheelspd);
//                LTOPFLYWHEEL.setSpeed(-60);
////                RBOTTOMFLYWHEEL.setSpeed(-flywheelspd);
//                LBOTTOMFLYWHEEL.setSpeed(60);
////                RTOPFLYWHEEL.setPower(1000);
////               LTOPFLYWHEEL.setPower(-16000);
////                RBOTTOMFLYWHEEL.setPower(-flywheelspd);
////               LBOTTOMFLYWHEEL.setPower(16000);
//            } else {
//
//                RTOPFLYWHEEL.setPower(0);
//                LTOPFLYWHEEL.setPower(0);
//                RBOTTOMFLYWHEEL.setPower(0);
//                LBOTTOMFLYWHEEL.setPower( 0);
//            }
//            loopTimer = timeStart;
//            DJIMotor::s_sendValues();
//        }
//        DJIMotor::s_getFeedback();
//        ThisThread::sleep_for(1ms);
//    }
//}
#include "main.h"


//infantry

DJIMotor yaw(5, CANHandler::CANBUS_1, GIMBLY);
DJIMotor pitch(7, CANHandler::CANBUS_2, GIMBLY); // right
DJIMotor pitch2(6, CANHandler::CANBUS_2, GIMBLY); // left
DJIMotor indexer(7, CANHandler::CANBUS_2, C610);
DJIMotor RFLYWHEEL(8, CANHandler::CANBUS_2, M3508);
DJIMotor LFLYWHEEL(5, CANHandler::CANBUS_2, M3508);
DJIMotor gearSwap(4, CANHandler::CANBUS_2, M2006); // gear swap



DigitalOut led(L27);

int main(){

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);

//    int lflypid = 0;
//    int rflypid = 0;
    int flywheelspd = 5000;
    int refLoop = 0;

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();

    LFLYWHEEL.setSpeedPID(10, 0, 0.02);
    RFLYWHEEL.setSpeedPID(10, 0, 0.02);

    while(true){
        timeStart = us_ticker_read();



        //wheel.setSpeed(lY5);
        if ((timeStart - loopTimer)/ 1000 > 25){

            refLoop++;
            if (refLoop >= 5){
                refereeThread(&referee);
                refLoop = 0;
            }

            led =! led;

            //printff("%d %d %d %f\n", LFLYWHEEL>>VELOCITY, flywheelspd, LFLYWHEEL>>POWEROUT, ext_shoot_data.data.bullet_freq, ext_power_heat_data.data.shooter_id1_17mm_cooling_heat);
        printff("%d %d %d %d\n", indexer>>VELOCITY, RFLYWHEEL>>VELOCITY, LFLYWHEEL>>POWEROUT, RFLYWHEEL>>POWEROUT);
            remoteRead();
            if (rS == Remote::SwitchState::UP){
                gearSwap.setPower(1500);
            } else if (rS == Remote::SwitchState::DOWN){
                gearSwap.setPower(-1500);
            }

            indexer.setPower(lY*3);


            //indexer.setPower(lY*5);


            if (Remote::SwitchState::UP == lS){
                LFLYWHEEL.setSpeed(-flywheelspd);
                RFLYWHEEL.setSpeed(flywheelspd);
            } else if (Remote::SwitchState::DOWN == lS) {
                LFLYWHEEL.setPower(-16000);
                RFLYWHEEL.setPower(16000);
            }
            else {
                LFLYWHEEL.setSpeed(0);
                RFLYWHEEL.setSpeed(0);
            }


            //printf("working %d", indexer>>VELOCITY);

            loopTimer = timeStart;
            DJIMotor::s_sendValues();
        }


        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}
