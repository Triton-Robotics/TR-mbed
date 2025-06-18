#include "main.h"

DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);
DigitalOut ledbuiltin(LED1);DJIMotor testMot(7, CANHandler::CANBUS_1, GM6020, "testbench_motor");

#define IMPULSE_DT 100
#define IMPULSE_STRENGTH 16383

int main(){    

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();   

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    unsigned long loopTimer2 = us_ticker_read();
    int powerCnt = 0;
    int refLoop = 0;    
    
    testMot.setSpeedPID(250,0,0);
    PID yawBeyblade(0.04,0,4);
    yawBeyblade.setOutputCap(60);
    int yawVelo = 0;
    float yaw_desired_angle = (testMot>>ANGLE) * 360.0 / TICKS_REVOLUTION;
    float yaw_current_angle = (testMot>>ANGLE) * 360.0 / TICKS_REVOLUTION;

    unsigned long timeSure;
    unsigned long prevTimeSure;

    bool prevL = false;
    bool switL = false;
    int motorSpeed = 0;    
    
    while(true){
        timeStart = us_ticker_read();       

        if ((timeStart - loopTimer2) / 1000 > 5000 && (powerCnt < 16000)){
            loopTimer2 = timeStart;
            if(switL){
                testMot.setPower(powerCnt);
                powerCnt += 1000;
            }
        }        

        if ((timeStart - loopTimer) / 1000 > 15){
            
            float elapsedms = (timeStart - loopTimer) / 1000;
            loopTimer = timeStart;
            led = !led;
            ledbuiltin = !ledbuiltin;            
            
            refLoop++;
            if (refLoop >= 5){                
                
                refereeThread(&referee);
                refLoop = 0;
                led2 = !led2;            
            
            }
            prevL = switL;            
            
            remoteRead();
            switL = (remote.leftSwitch() == Remote::SwitchState::UP);            
            
            // testMot.setPower(remote.leftX() * 3);
            // if(!prevL && switL){
            //     motorSpeed += 10;
            // }
            
            // //Regular Yaw Code
            // yaw_desired_angle = remote.leftX() * 5;
            // prevTimeSure = timeSure;
            // timeSure = us_ticker_read();
            // motorSpeed = yawBeyblade.calculatePeriodic(DJIMotor::s_calculateDeltaPhase(yaw_desired_angle, testMot>>ANGLE, 8192), timeSure - prevTimeSure);            
            
            // int dir = 0;
            // if(motorSpeed > 0){
            //     dir = 1;
            // }else if(motorSpeed < 0){
            //     dir = -1;            }
            // testMot.pidSpeed.feedForward = dir * ((15.4 + abs(motorSpeed)) / 0.0083);

            // if(remote.rightSwitch() == Remote::SwitchState::UP) {
            //     testMot.setSpeed(motorSpeed);
            // } else {
            //     testMot.setPower(0);            
            // }

            //printff("%d\t%d\t%d\t%d\t%d\n", yaw_desired_angle, testMot>>ANGLE, motorSpeed, testMot>>VELOCITY, testMot>>POWEROUT);            
            printff("%d,%d\n", testMot>>VELOCITY, testMot>>POWEROUT);            
            

            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}