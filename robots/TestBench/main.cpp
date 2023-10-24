#include "main.h"


// DJIMotor yaw(5, CANHandler::CANBUS_1, GIMBLY);
// DJIMotor pitch(7, CANHandler::CANBUS_2, GIMBLY); // right
// DJIMotor pitch2(6, CANHandler::CANBUS_2, GIMBLY); // left
DJIMotor indexer(5, CANHandler::CANBUS_2, C610);
DJIMotor RFLYWHEEL(4, CANHandler::CANBUS_2, M3508);
DJIMotor LFLYWHEEL(1, CANHandler::CANBUS_2, M3508);
// DJIMotor gearSwap(4, CANHandler::CANBUS_2, M2006); // gear swap



DigitalOut led(L27);

int main(){

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    int flywheelspd = 6000;
    int refLoop = 0;

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();

    while(true){
        timeStart = us_ticker_read();
        printf("Hey\n");

        if ((timeStart - loopTimer) / 1000 > 25){
            led = !led;


        if ((timeStart - loopTimer)/ 1000 > 25){

            refLoop++;
            if (refLoop >= 5){
                refereeThread(&referee);
                refLoop = 0;
                //led2 = !led2;
            }

            led =! led;

            
            remoteRead();

            indexer.setPower(lY*3);

            if (Remote::SwitchState::UP == lS){
                LFLYWHEEL.setSpeed(flywheelspd);
                RFLYWHEEL.setSpeed(-flywheelspd);
                printff("%f\t%d\t%d\t%d\t%d\n",us_ticker_read()/1000.0, LFLYWHEEL>>VELOCITY, RFLYWHEEL>>VELOCITY, LFLYWHEEL>>POWEROUT, RFLYWHEEL>>POWEROUT);
            } else if (Remote::SwitchState::DOWN == lS) {
                LFLYWHEEL.setPower(16000);
                RFLYWHEEL.setPower(-16000);
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