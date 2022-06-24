#include "../src/main.hpp"

int pitchval = 0;
CANMotor LF(4,NewCANHandler::CANBUS_1,M3508);
CANMotor RF(2,NewCANHandler::CANBUS_1,M3508);
CANMotor LB(1,NewCANHandler::CANBUS_1,M3508);
CANMotor RB(3,NewCANHandler::CANBUS_1,M3508);

CANMotor yaw(5, NewCANHandler::CANBUS_1, M3508);
CANMotor pitch(6, NewCANHandler::CANBUS_1, GM6020);
CANMotor indexer(7, NewCANHandler::CANBUS_1, GM6020);
int indexJamTime = 0;
bool lastJam = 0;
int indexUnJamTime = 0;

CANMotor RFLYWHEEL(8,NewCANHandler::CANBUS_1,M3508);
PWMMotor LFLYWHEEL(D11);
float speedMultplier = 1;

int main()
{
    
    NewCANHandler canHandler1(PA_11,PA_12);
    NewCANHandler canHandler2(PB_12,PB_13);
    threadingRemote.start(&remoteThread);
    CANMotor::setCANHandlers(&canHandler1,&canHandler2);

    while (true) {

        if(rS == 1){ // Everything non-chassis enable
            //yaw.setPower(rX*4);
            yaw.setSpeed(rX * 3);
            pitch.setPower(rY * 3);
                }
        }else if(rS == 3){ // Chassis enable 

            int LFa = lY + lX, RFa = lY - lX, LBa = lY - lX, RBa = lY + lX;
            LF.setSpeed(LFa*speedMultplier);
            RF.setSpeed(-RFa*speedMultplier);
            LB.setSpeed(LBa*speedMultplier);
            RB.setSpeed(-RBa*speedMultplier);
            yaw.setPower(rX * 4);
            //pitch.setSpeed(rY/4);
        
        }else if(rS == 2){ // Disable robot
            LF.setPower(0);RF.setPower(0);LB.setPower(0);RB.setPower(0);
            yaw.setPower(0); pitch.setPower(0); indexer.setPower(0);
            LFLYWHEEL.set(0);
            RFLYWHEEL.setPower(0);
        }
        
        if (rS != 2) {
            if (lS == 1) {
                indexer.setPower(rY * 8);
                //CANMotor::printChunk(CANHandler::CANBUS_1,1);
                //printf("MANUAL-PWR:%d VELO:%d\n", indexer.powerOut, indexer.getData(VELOCITY));
                LFLYWHEEL.set(60);
                RFLYWHEEL.setPower(-1000);
            }
            else if(lS == 2){
                indexer.setPower(0);
                LFLYWHEEL.set(0);
                RFLYWHEEL.setPower(0);

            }else if(lS == 3){ //Start serializing with anti-jam code
                printf("Torque:%d", indexer.getData(TORQUE));
                if(abs(indexer.getData(TORQUE)) > 2000 & abs(indexer.getData(VELOCITY)) < 20){ //jam
                    if (lastJam == 0) {
                        indexJamTime = us_ticker_read() /1000;
                        lastJam = 1;
                        printf("jam detected!\n");
                    }
                }
                else {
                    lastJam = 0;
                }
                if(lastJam && us_ticker_read() / 1000 - indexJamTime > 250){
                    indexer.setPower(-7500); //jam
                    printf("Jammed\n");
                }else{
                    indexer.setSpeed(50);
                    printf("setting speed\n");
                }
                //printf("AUTO-PWR:%d Jam-Free:%dms TORQ:%d, VELO:%d\n",indexer.powerOut,us_ticker_read() / 1000 - indexJamTime, indexer.getData(TORQUE), indexer.getData(VELOCITY));
                //setFlyWheelPwr(100);
            }
        }

        // RFLYWHEEL.setPower(1000);

        // printf("Flywheeeeeee:%d\n",RFLYWHEEL.getData(VELOCITY));

        ThisThread::sleep_for(1ms); 
    }
}
