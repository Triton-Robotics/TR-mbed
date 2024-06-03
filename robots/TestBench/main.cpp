#include "main.h"

DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);
DigitalOut ledbuiltin(LED1);
DJIMotor indexer_L(8, CANHandler::CANBUS_2, C610);
DJIMotor indexer_R(7, CANHandler::CANBUS_2, C610);


DJIMotor testMot(4, CANHandler::CANBUS_1, M3508, "testbench_motor");

int main(){

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    int refLoop = 0;

    testMot.setSpeedPID(1.5,0,0);

    while(true){
        timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > 25){
            loopTimer = timeStart;
            led = !led;
            ledbuiltin = !ledbuiltin;

            refLoop++;
            if (refLoop >= 5){
                refereeThread(&referee);
                refLoop = 0;
                led2 = !led2;
                indexer_R.setPower(5000);
                indexer_L.setPower(5000);
                printff("%d %d\n",indexer_R>>ANGLE, indexer_L>>ANGLE );
                //printff("datum:%d %d %d\n", testMot>>ANGLE, testMot>>VELOCITY, remote.leftX());
            }

            remoteRead();

            testMot.setPower(remote.leftX() * 3);
            
            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}