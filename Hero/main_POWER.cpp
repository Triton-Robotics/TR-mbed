#include "../src/main.hpp"
#include <cstdlib>

#define PI 3.14159265

CANMotor LF(4,NewCANHandler::CANBUS_1,M3508); 
CANMotor RF(2,NewCANHandler::CANBUS_1,M3508); 
CANMotor LB(1,NewCANHandler::CANBUS_1,M3508); 
CANMotor RB(3,NewCANHandler::CANBUS_1,M3508);

CANMotor yaw(5, NewCANHandler::CANBUS_1, M3508);
CANMotor pitch(6, NewCANHandler::CANBUS_1, GIMBLY);

CANMotor indexer(7, NewCANHandler::CANBUS_1, GIMBLY);

PWMMotor RFLYWHEEL(D12); PWMMotor LFLYWHEEL(D11);

int main()
{
    threadingRemote.start(&remoteThread);
    //threadingReferee.start(&refereeThread);
    CANMotor::setCANHandlers(&canHandler1,&canHandler2);

    LF.outCap = 16000;   
    RF.outCap = 16000;
    LB.outCap = 16000;
    RB.outCap = 16000;

    int multiplier = 1;
    int indexJamTime = 0;
    int lastJam = 0;

    bool strawberryJam = false;

    while (true) {
        if(rS == 1){
            int LFa = lY + lX + Wh, RFa = lY - lX - Wh, LBa = lY - lX + Wh, RBa = lY + lX - Wh;
            LF.setPower(LFa*multiplier);
            RF.setPower(-RFa*multiplier);
            LB.setPower(LBa*multiplier);
            RB.setPower(-RBa*multiplier);

            pitch.setPower(rY * -20);
            printf("pitch%d\n",pitch.getData(MULTI));
        }else{
            LF.setPower(0);RF.setPower(0);LB.setPower(0);RB.setPower(0);
            pitch.setPower(0);
        }

        if(lS == 3){
            if(abs(indexer.getData(TORQUE)) > 1000 & abs(indexer.getData(VELOCITY)) < 20){ //intial jam detection
                if (lastJam == 0) {
                    indexJamTime = us_ticker_read() /1000; // start clock
                    lastJam = 1;
                    printf("jam detected!\n");
                }
            }
            else 
                lastJam = 0;
            
            if(lastJam && us_ticker_read() / 1000 - indexJamTime > 750){ // If jam for more than 250ms then reverse
                strawberryJam = true;
            }else
                indexer.setSpeed(2500); // No Jam, regular state
            if(strawberryJam && us_ticker_read() / 1000 - indexJamTime < 1500){
                indexer.setPower(15000); 
                printf("Shoving...%d\n",us_ticker_read() / 1000 - indexJamTime);
            }if(strawberryJam && us_ticker_read() / 1000 - indexJamTime < 2250){
                indexer.setPower(-7500); 
                printf("Reversing...%d\n",us_ticker_read() / 1000 - indexJamTime);
            }else if(strawberryJam && us_ticker_read() / 1000 - indexJamTime > 2250){
                strawberryJam = false;
                lastJam = 0;
            }
            LFLYWHEEL.set(60); RFLYWHEEL.set(60);
        }else if(lS == 2){
            indexer.setPower(0);
            LFLYWHEEL.set(40); RFLYWHEEL.set(40);
        }else{
            indexer.setPower(0);
            LFLYWHEEL.set(0); RFLYWHEEL.set(0);
        }
        ThisThread::sleep_for(1ms);
    }
}

