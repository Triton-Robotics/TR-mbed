#include "../src/main.hpp"
#include <cstdlib>

CANMotor chassis1(3,NewCANHandler::CANBUS_1,M3508);
CANMotor chassis2(4,NewCANHandler::CANBUS_1,M3508);

CANMotor yaw(3,NewCANHandler::CANBUS_1,GM6020);
CANMotor pitch(6,NewCANHandler::CANBUS_1,GM6020);

PWMMotor leftFlywheelTop(PA_5);
PWMMotor rightFlywheelTop(PA_6);
PWMMotor leftFlywheelBot(PB_6);
PWMMotor rightFlywheelBot(PA_7);

CANMotor indexer(5,NewCANHandler::CANBUS_1,M2006);

int main()
{
    threadingRemote.start(&remoteThread);
    CANMotor::setCANHandlers(&canHandler1,&canHandler2);
    //yaw.pidPosition.debug = true;
    //pitch.zeroPos();
    //yaw.setPositionPID(1.25, 0, 0);

    pitch.outCap = 30000;
    

    while (true) {
        
        if(rS == 2){
            yaw.setPower(0);
            pitch.setPower(0);
        }else if(rS == 1) {
            chassis1.setPower(lX);
            chassis2.setPower(lX);
            yaw.setPower(rX * 2);
            pitch.setPower(rY * 20);
        }
        int indexJamTime = 0;
        if(lS == 2){
            indexer.setPower(0);

            leftFlywheelTop.set(0);
            rightFlywheelTop.set(0);
            leftFlywheelBot.set(0);
            rightFlywheelBot.set(0);
        }else if(lS == 3){
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
                indexer.setSpeed(1700);
            }
            printf("AUTO-PWR:%d Jam-Free:%dms TORQ:%d, VELO:%d\n",indexer.powerOut,us_ticker_read() / 1000 - indexJamTime, indexer.getData(TORQUE), indexer.getData(VELOCITY));
            
            leftFlywheelTop.set(60);
            rightFlywheelTop.set(60);
            leftFlywheelBot.set(60);
            rightFlywheelBot.set(60);
        }else if(lS == 1){
            indexer.setPower(rY * 3);
            //CANMotor::printChunk(CANHandler::CANBUS_1,1);
            //printf("MANUAL-PWR:%d VELO:%d\n", indexer.powerOut, indexer.getData(VELOCITY));
            leftFlywheelTop.set(60);
            rightFlywheelTop.set(60);
            leftFlywheelBot.set(60);
            rightFlywheelBot.set(60);
        }

        // if(lS == 2)
        //     yaw.setSpeed(rX/10);
        // else if(lS == 1)
        //     yaw.setPower(0);
        //printf("RAD:%d ANGL:%d YAW:%d ATAN(%d):%d\n", int(1000 * tAngle), int(angle * 1000), yaw.getData(ANGLE), int((double)(lY)/lX * 1000), int(std::atan2(lY,lX) * 1000));
        //CANMotor::printChunk(CANHandler::CANBUS_1,0);
        //printf("jAngle:%d\n",int(jAngle * 1000));


        // printf("%d %d %d %d\n",
        //     LF.getData(VELOCITY),
        //     RF.getData(VELOCITY),
        //     LB.getData(VELOCITY),
        //     RB.getData(VELOCITY));

        

        //for(int i = 0; i < 12; i)

        // else {
        //     //yawval+= (int)myremote.getStickData(LEFTJOYY, 0, maxpitchyawchange);
        //     pitchval+=(int)myremote.getStickData(LEFTJOYY, 0, maxpitchyawchange);

        //     if (pitchval > 240)
        //         pitchval = 240;
        //     if (pitchval < 180)
        //         pitchval = 180;
            
        // }
        
        // if (lS == 1) {
        //     LFLYWHEEL.set(revval);
        //     RFLYWHEEL.set(revval);
        //     serializer.setPower(-70);
        // }
        // else if (lS == 2) {
        //     serializer.setPower(90);
        // }
        // else if (lS == 3) {
        //     LFLYWHEEL.set(neutralval);
        //     RFLYWHEEL.set(neutralval);
        //     serializer.setPower(0);
        // }

        //yaw.setDesiredPos(yawval);
        //pitch.setPower(220);

        //CANMotor::tick(lS == 2, rS == 2);
        ThisThread::sleep_for(1ms);
    }
}

