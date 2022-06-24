#include "../src/main.hpp"
#include <cstdlib>

#define PI 3.14159265



CANMotor LF(4,NewCANHandler::CANBUS_1,M3508); 
CANMotor RF(2,NewCANHandler::CANBUS_1,M3508); 
CANMotor LB(1,NewCANHandler::CANBUS_1,M3508); 
CANMotor RB(3,NewCANHandler::CANBUS_1,M3508);
float multiplier = 1;

float speedMult = 0.3;

CANMotor yaw(5, NewCANHandler::CANBUS_1, GIMBLY);
CANMotor pitch(6, NewCANHandler::CANBUS_1, GIMBLY);
CANMotor indexer(7, NewCANHandler::CANBUS_1, C610);
int pitchval = 0;
#define LOWERBOUND 1000
#define UPPERBOUND 2000

PWMMotor RFLYWHEEL(D12); PWMMotor LFLYWHEEL(D11);
PWMMotor flyWheelMotors[] = {RFLYWHEEL, LFLYWHEEL};

void setFlyWheelPwr(int pwr) {
    for (int i = 0; i < 2; i++)
        flyWheelMotors[i].set(pwr);
}

/**
 * For m3508 16000 goes 8723 velocity
**/

int main()
{
    threadingRemote.start(&remoteThread);
    //threadingReferee.start(&refereeThread);
    CANMotor::setCANHandlers(&canHandler1,&canHandler2);

    LB.setSpeedPID(1.75, 0.351, 5.63);
    RF.setSpeedPID(2.744, 0.285, 4.192);
    RB.setSpeedPID(2.572, 0.113, 1.723);
    LF.setSpeedPID(.992, 0.164, 0.069);
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
        }else if(rS == 3 && 0){ // beyblade mode
            double angle = yaw.getData(ANGLE) / 8192.0 * PI * 2;
            angle -= PI/4;
            
            int speedMult = 2;

            float raw_x = lX * speedMult;
            float raw_y = lY * speedMult;

            float x = (float) (raw_x * cos(angle) - raw_y * sin(angle));
            float y = (float) (raw_x * sin(angle) + raw_y * cos(angle));

            int beyblade_rotation = 2000;
            LF.setSpeed(x + y + beyblade_rotation);
            RF.setSpeed(x - y + beyblade_rotation);
            LB.setSpeed(-x + y + beyblade_rotation);
            RB.setSpeed(-x - y + beyblade_rotation);
            yaw.setSpeed(20);
        }

        if (lS == 1) {
            indexer.setPower(rY * 3);
            setFlyWheelPwr(40);
        } else if(lS == 2){ //disable serializer
            indexer.setPower(0);
            setFlyWheelPwr(0);
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
                indexer.setSpeed(-700);
            }
            //printf("AUTO-PWR:%d Jam-Free:%dms TORQ:%d, VELO:%d\n",indexer.powerOut,us_ticker_read() / 1000 - indexJamTime, indexer.getData(TORQUE), indexer.getData(VELOCITY));
            setFlyWheelPwr(0);
        }

        ThisThread::sleep_for(1ms);
    }
}

