#include "main.h"


DJIMotor indexer(5, CANHandler::CANBUS_2, C610);
DJIMotor RFLYWHEEL(4, CANHandler::CANBUS_2, M3508);
DJIMotor LFLYWHEEL(1, CANHandler::CANBUS_2, M3508);

DigitalOut led(L27);

int main(){

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    int flywheelspd = 6500;
    int refLoop = 0;

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();

    double kP = 1.5;
    double kI = 0.0000021;
    double kD = 0.04;

    indexer.setPositionPID(1, 0, 0);
    indexer.useAbsEncoder = false;

    while(true){
        timeStart = us_ticker_read();

//        LFLYWHEEL.setSpeed(-5000);
//        RFLYWHEEL.setSpeed(5000);

        if ((timeStart - loopTimer)/ 1000 > 25){

            refLoop++;
            if (refLoop >= 5){
                refereeThread(&referee);
                refLoop = 0;
                printff("%d %d %d %d %d %d %f\n", LFLYWHEEL>>VELOCITY, ((int)remote.rightSwitch() - 1) * 4000, LFLYWHEEL>>POWEROUT, (int)(kP*1000), (int)(kI * 10000000), (int)(kD * 1000), ext_shoot_data.data.bullet_speed);

            }

            led =! led;
            remoteRead();

            LFLYWHEEL.setSpeed(((int)remote.leftSwitch()-1) * -4000);
            RFLYWHEEL.setSpeed(((int)remote.leftSwitch()-1) * 4000);
            indexer.setPower(remote.leftX()*10);


            // int id = 0;
            // uint8_t bytes[4] = {0,0,0,0};
            // int length = 4;
            // canHandler1.rawRead (&id, bytes,length);

            // if(id != 0)
            //     printff("0x%x [%hu,%hu,%hu,%hu] = %d\n", id, bytes[0], bytes[1], bytes[2], bytes[3],  ((int)bytes[0]) + ((int)bytes[1]<<8) + ((int)bytes[2]<<16) + ((int)bytes[3]<<24));

            // loopTimer = timeStart;
            DJIMotor::s_sendValues();
        }


        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}