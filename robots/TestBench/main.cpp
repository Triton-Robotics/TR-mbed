// #include "main.h"


// DJIMotor flywheelTop(4, CANHandler::CANBUS_1, M3508);
// DJIMotor flywheelBot(1, CANHandler::CANBUS_1, M3508);
// DJIMotor duck(5, CANHandler::CANBUS_1, M2006);
// DigitalOut led(L27);
// DigitalOut led2(L26);

// int main(){
//     DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);


//     unsigned long timeStart;
//     unsigned long loopTimer = us_ticker_read();

//     int refLoop = 0;

//     while(true){
//         timeStart = us_ticker_read();

//         remoteRead();
//         //wheel.setSpeed(lY*5);
//         if ((timeStart - loopTimer)/ 1000 > 25){
//             led = !led;

//             refLoop++;
//             if (refLoop >= 5){
//                 refereeThread(&referee);
//                 refLoop = 0;
//                 led2 = !led2;
//                 //printff("%d, %d RME%d\n", wheel>>VELOCITY, wheel_2>>ANGLE, lY);
//                 //printff("id[%d] lvl[%d] rem_HP[%d] max_HP[%d] \n",ext_game_robot_state.data.robot_id,ext_game_robot_state.data.robot_level,ext_game_robot_state.data.remain_HP,ext_game_robot_state.data.max_HP);
//                 //printff("maxSpd_17_1[%d] maxSpd_17_2[%d] maxSpd_42_1[%d]\n",ext_game_robot_state.data.shooter_id1_17mm_speed_limit,ext_game_robot_state.data.shooter_id2_17mm_speed_limit,ext_game_robot_state.data.shooter_id1_42mm_speed_limit);
//                 //printff("pwr_lim[%d] c_bool[%d] g_bool[%d] ab_bool[%d]\n",ext_game_robot_state.data.chassis_power_limit,ext_game_robot_state.data.mains_power_gimbal_output,ext_game_robot_state.data.mains_power_chassis_output,ext_game_robot_state.data.mains_power_shooter_output);
            
//                 //printff("pwr_lim[%d] c_bool[%d] g_bool[%d] ab_bool[%d]\n");
//                 printf("L:%d R:%d D:%d, lY[%d]\n",flywheelTop>>VELOCITY,flywheelBot>>VELOCITY,duck.powerOut, lY);
//             }

//             flywheelTop.setSpeed(((int)lS - 2) * 5000);
//             flywheelBot.setSpeed(((int)lS - 2) * -5000);
//             duck.setPower(rY * 10);

//             loopTimer = timeStart;
//             DJIMotor::s_sendValues();
//         }
//         DJIMotor::s_getFeedback();
//         ThisThread::sleep_for(1ms);   
//     }
// }

#include "main.h"


// DJIMotor yaw(5, CANHandler::CANBUS_1, GIMBLY);
// DJIMotor pitch(7, CANHandler::CANBUS_2, GIMBLY); // right
// DJIMotor pitch2(6, CANHandler::CANBUS_2, GIMBLY); // left
DJIMotor indexer(5, CANHandler::CANBUS_2, C610);
DJIMotor RFLYWHEEL(4, CANHandler::CANBUS_1, M3508);
DJIMotor LFLYWHEEL(1, CANHandler::CANBUS_1, M3508);
// DJIMotor gearSwap(4, CANHandler::CANBUS_2, M2006); // gear swap



DigitalOut led(L27);
DigitalOut led2(L25);

int main(){

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    int flywheelspd = 5000;
    int refLoop = 0;

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();

    LFLYWHEEL.setSpeedPID(10, 0, 0.02);
    RFLYWHEEL.setSpeedPID(10, 0, 0.02);

    while(true){
        timeStart = us_ticker_read();



        if ((timeStart - loopTimer)/ 1000 > 25){

            refLoop++;
            if (refLoop >= 5){
                refereeThread(&referee);
                refLoop = 0;
                led2 = !led2;
            }

            led =! led;

            printff("%d %d %d %d\n", indexer>>VELOCITY, RFLYWHEEL>>VELOCITY, LFLYWHEEL>>POWEROUT, RFLYWHEEL>>POWEROUT);
            remoteRead();
            // if (rS == Remote::SwitchState::UP){
            //     gearSwap.setPower(1500);
            // } else if (rS == Remote::SwitchState::DOWN){
            //     gearSwap.setPower(-1500);
            // }

            indexer.setPower(lY*3);

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



            loopTimer = timeStart;
            DJIMotor::s_sendValues();
        }


        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}