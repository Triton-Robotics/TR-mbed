#include "Infantry.h"

int main(){
    init();
    imuThread.start(imuthread);
    refThread.start(refthread);
    
    while(true){
        timeStartCV = us_ticker_read();        
        if((timeStartCV - loopTimerCV) / 1000 > OUTER_LOOP_DT_MS) { //1 with sync or 2 without
            loopTimerCV = timeStartCV;
            jetson_executor();
        }
        // printff("%dus\n", (us_ticker_read() - timeStartCV));

        timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > OUTER_LOOP_DT_MS){
            elapsedms = (timeStart - loopTimer) / 1000;
            loopTimer = timeStart;
            led = !led;


            Chassis.periodic();
            cs = Chassis.getChassisSpeeds();
            
            if (remoteTimer > 10) {
                remoteTimer = 0;
                remoteRead();
            }
            remoteTimer += 1;

            //Chassis Code - 100-150 us
            chassis_executor();
            

            // YAW + PITCH - 150us
            gimbal_executor();
            
            // dexer + flywheel - 100us
            shoot_executor();

            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        printff("Time:%dus\n", (us_ticker_read() - timeStartCV));
        ThisThread::sleep_for(1ms);
    }
}