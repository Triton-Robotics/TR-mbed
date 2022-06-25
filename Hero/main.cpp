#include "../src/main.hpp"
#include <cstdlib>

#define PI 3.14159265

CANMotor LF(4,NewCANHandler::CANBUS_1,M3508); 
CANMotor RF(2,NewCANHandler::CANBUS_1,M3508); 
CANMotor LB(1,NewCANHandler::CANBUS_1,M3508); 
CANMotor RB(3,NewCANHandler::CANBUS_1,M3508);
float multiplier = 3;

float beybladespeedmult = 1;

CANMotor yaw(5, NewCANHandler::CANBUS_1, M3508);
CANMotor pitch(6, NewCANHandler::CANBUS_1, GIMBLY);
int pitchval = 0;
#define LOWERBOUND 1000
#define UPPERBOUND 2000

CANMotor indexer(7, NewCANHandler::CANBUS_1, GIMBLY);
int indexJamTime = 0;
int lastJam = 0;

PWMMotor RFLYWHEEL(D12); PWMMotor LFLYWHEEL(D11);

void setFlyWheelPwr(int pwrL, int pwrR) {
    LFLYWHEEL.set(pwrL);
    RFLYWHEEL.set(pwrR);
}

int main()
{
    threadingRemote.start(&remoteThread);
    //threadingReferee.start(&refereeThread);
    CANMotor::setCANHandlers(&canHandler1,&canHandler2);

    // LB.setSpeedPID(1.75, 0.351, 5.63);
    // RF.setSpeedPID(1.073, 0.556, 0);
    // RB.setSpeedPID(1.081, 0.247, 0.386); 
    LF.setSpeedPID(1.129, 0.292, 1.938);
    pitch.setPositionPID(.017,.001,.044);
    pitch.useAbsEncoder = 1;
    yaw.setSpeedPID(1.531,0,0); //.6*1.531 1.2*1.531/.234 0.075*1.531*.234
    //indexer.setSpeedPID(0.014, 0.001, 0.046);

    while (true) {

        LF.outCap = 16000;   
        RF.outCap = 16000;
        LB.outCap = 16000;
        RB.outCap = 16000;
        

        if(rS == 1){ // All non-serializer motors activated
            int LFa = lY + lX + Wh, RFa = lY - lX - Wh, LBa = lY - lX + Wh, RBa = lY + lX - Wh;
            LF.setSpeed(LFa * multiplier);
            RF.setSpeed(-RFa*multiplier);
            LB.setSpeed(LBa*multiplier);
            RB.setSpeed(-RBa*multiplier);
            
            if (pitchval < LOWERBOUND) // lowerbound
                pitchval = LOWERBOUND;
            if (pitchval > UPPERBOUND) //upperbound
                pitchval = UPPERBOUND;
            pitchval += (int)myremote.getStickData(RIGHTJOYY, 0, 3);
            pitch.setPosition(pitchval);
            printf("YAW:%d\tPITCH:%d\n",rX/10,pitchval);
            yaw.setSpeed(rX);
            

        }else if(rS == 2){ //disable all the non-serializer components
            LF.setPower(0);RF.setPower(0);LB.setPower(0);RB.setPower(0);
            yaw.setPower(0); pitch.setPower(0);
        }else if(rS == 3){ // beyblade mod
            remotePrint();
        }

        if (lS == 1) {
            indexer.setSpeed(rY*5);
            setFlyWheelPwr(40,40);
        } else if(lS == 2){ //disable serializer
            indexer.setPower(0);
            setFlyWheelPwr(0,0);
        }else if(lS == 3){
            setFlyWheelPwr(60,60);
                if(abs(indexer.getData(TORQUE)) > 500 & abs(indexer.getData(VELOCITY)) < 20){ //intial jam detection
                    if (lastJam == 0) {
                        indexJamTime = us_ticker_read() /1000; // start clock
                        lastJam = 1;
                        printf("jam detected!\n");
                    }
                }
                else 
                    lastJam = 0;
                
                if(lastJam && us_ticker_read() / 1000 - indexJamTime > 75){ // If jam for more than 250ms then reverse
                    indexer.setPower(7500); 
                    printf("Unjamming...\n");
                }else
                    indexer.setSpeed(2000); // No Jam, regular state
        }

        ThisThread::sleep_for(1ms);
    }
}

