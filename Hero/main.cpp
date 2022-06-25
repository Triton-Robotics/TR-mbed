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
int pitchValue = 0;
int yawValue = 0;
#define LOWERBOUND 1000
#define UPPERBOUND 2000
#define LEFTBOUND -1000
#define RIGHTBOUND 1000

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
    threadingReferee.start(&refereeThread);
    CANMotor::setCANHandlers(&canHandler1,&canHandler2);

    // LB.setSpeedPID(1.75, 0.351, 5.63);
    // RF.setSpeedPID(1.073, 0.556, 0);
    // RB.setSpeedPID(1.081, 0.247, 0.386); 
    LF.setSpeedPID(1.129, 0.292, 1.938);
    //pitch.setPositionPID(.017,.001,.044);
    pitch.useAbsEncoder = 1;
    yaw.setSpeedPID(1.531,0,0); //.6*1.531 1.2*1.531/.234 0.075*1.531*.234
    //pitch.setSpeedPID(int speed)
    //pitch.pidPosition.feedForward = -20000;
    pitch.setPositionPID(0.02,0,0);
    //pitch.pidPosition.setOutputCap(-15000);
    //pitch.pidSpeed.feedForward = 3000;
    //indexer.setSpeedPID(0.014, 0.001, 0.046);

    while (true) {

        LF.outCap = 16000;   
        RF.outCap = 16000;
        LB.outCap = 16000;
        RB.outCap = 16000;
        int keyFwd = (myremote.getKeyState(W) - myremote.getKeyState(S)) * 1000;
        int keyTurn = (myremote.getKeyState(A) - myremote.getKeyState(D)) * -1000;
        int keyStrafe = (myremote.getKeyState(Q) - myremote.getKeyState(E)) * -1000;
        

        if(rS == 1){ // All non-serializer motors activated
            int LFa = keyFwd + keyStrafe + keyTurn, RFa = keyFwd - keyStrafe - keyTurn, LBa = keyFwd - keyStrafe + keyTurn, RBa = keyFwd + keyStrafe - keyTurn;
            LF.setSpeed(LFa * multiplier);
            RF.setSpeed(-RFa*multiplier);
            LB.setSpeed(LBa*multiplier);
            RB.setSpeed(-RBa*multiplier);
            
            pitchValue += myremote.getMouseData(SPEEDY) * 100;
            yawValue += myremote.getMouseData(SPEEDX);

            if (pitchValue < LOWERBOUND) // lowerbound
                pitchValue = LOWERBOUND;
            if (pitchValue > UPPERBOUND) //upperbound
                pitchValue = UPPERBOUND;
            if (yawValue < LEFTBOUND) // lowerbound
                yawValue = LEFTBOUND;
            if (yawValue > RIGHTBOUND) //upperbound
                yawValue = RIGHTBOUND;
            
            //pitch.setPosition(pitchValue);
            // yaw.setPosition(yawValue);
            printf("PITCH:%d\tD_PITCH:%d\n",pitch.getData(MULTI),pitchValue);
            //yaw.setSpeed(myremote.getMouseData(SPEEDX) * 15);
            yaw.setSpeed(myremote.getMouseData(SPEEDX) * -70);
            //pitch.setSpeed(myremote.getMouseData(SPEEDY) * 750);
            pitch.setPower(myremote.getMouseData(SPEEDY) * 200);
            

        }else if(rS == 2){ //disable all the non-serializer components
            LF.setPower(0);RF.setPower(0);LB.setPower(0);RB.setPower(0);
            yaw.setPower(0); pitch.setPower(0);
        }else if(rS == 3){ // beyblade mod
            LF.setPower(0);RF.setPower(0);LB.setPower(0);RB.setPower(0);
            yaw.setPower(0); pitch.setPower(0);
            if(ext_power_heat_data.data.chassis_power != 0)
                printf("CHASSIS POWER:%d\n",int(ext_power_heat_data.data.chassis_power));
        }

        bool strawberryJam = false;

        if (lS == 1) {
            indexer.setSpeed(rY*5);
            setFlyWheelPwr(40,40);
        } else if(lS == 2){ //disable serializer
            indexer.setPower(0);
            setFlyWheelPwr(0,0);
        }else if(lS == 3){
            setFlyWheelPwr(60,60);
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
        }

        ThisThread::sleep_for(1ms);
    }
}

