#include "../src/main.hpp"
#include <cstdlib>

#define PI 3.14159265



CANMotor LF(4,NewCANHandler::CANBUS_1,M3508); 
CANMotor RF(2,NewCANHandler::CANBUS_1,M3508); 
CANMotor LB(1,NewCANHandler::CANBUS_1,M3508); 
CANMotor RB(3,NewCANHandler::CANBUS_1,M3508);
float multiplier = 1;

float speedMult = 0.3;

CANMotor yaw(5, NewCANHandler::CANBUS_1, M3508);
CANMotor pitch(6, NewCANHandler::CANBUS_1, GIMBLY);
CANMotor indexer(7, NewCANHandler::CANBUS_1, GM6020);
int pitchval = 0;
#define LOWERBOUND 1000
#define UPPERBOUND 2000

PWMMotor LFLYWHEEL(D12);
CANMotor RFLYWHEEL(8,NewCANHandler::CANBUS_1,M3508);

void setFlyWheelPwr(int pwrL, int pwrR) {
    LFLYWHEEL.set(pwrL);
    RFLYWHEEL.setPower(-pwrR);
}

/**
 * For m3508 16000 goes 8723 velocity
**/

int main()
{
    threadingRemote.start(&remoteThread);
    //threadingReferee.start(&refereeThread);
    CANMotor::setCANHandlers(&canHandler1,&canHandler2);

    // LB.setSpeedPID(1.75, 0.351, 5.63);
    // RF.setSpeedPID(2.744, 0.285, 4.192); //ziegler calculate with a p of 2742 
    // RB.setSpeedPID(2.572, 0.113, 1.723);
    //LF.setSpeedPID(1.129, 0.292, 1.938);
    pitch.setPositionPID(0,0,0);

    while (true) {

        LF.outCap = 16000;   
        RF.outCap = 16000;
        LB.outCap = 16000;
        RB.outCap = 16000;
        

        if(rS == 1){ // All non-serializer motors activated
            int LFa = lY + lX, RFa = lY - lX, LBa = lY - lX, RBa = lY + lX;
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
            // yaw.setPower(rX * 4);
            

        }else if(rS == 2){ //disable all the non-serializer components
            LF.setPower(0);RF.setPower(0);LB.setPower(0);RB.setPower(0);
            yaw.setPower(0); pitch.setPower(0);
            //remotePrint();
        }else if(rS == 3 && 0){ // beyblade mode
            
        }

        if (lS == 1) {
            indexer.setPower(rY * 3);
            setFlyWheelPwr(60,1000);
        } else if(lS == 2){ //disable serializer
            indexer.setPower(0);
            setFlyWheelPwr(0,0);
        }else if(lS == 3){
            int indexJamTime = 0;
            if(abs(indexer.getData(TORQUE)) > 1000 & abs(indexer.getData(VELOCITY)) < 20){ //jam
                indexJamTime = us_ticker_read() /1000;
            }
            if(us_ticker_read() / 1000 - indexJamTime < 500){
                indexer.setPower(-7000); //jam
                printf("JAMMMMM- ");
            }else if(us_ticker_read() / 1000 - indexJamTime < 750){
                indexer.setPower(7000); //jam
                printf("POWER FORWARD- ");
            }else{
                indexer.setSpeed(2000);
            }
            //printf("AUTO-PWR:%d Jam-Free:%dms TORQ:%d, VELO:%d\n",indexer.powerOut,us_ticker_read() / 1000 - indexJamTime, indexer.getData(TORQUE), indexer.getData(VELOCITY));
            setFlyWheelPwr(60,1000);
        }

        ThisThread::sleep_for(1ms);
    }
}

