#include "Hero.h"

int main(){
    //GENERAL VARIABLES
    init();
    refThread.start(refthread);
    imuThread.start(imuthread);

    while(true){
        timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > OUTER_LOOP_DT_MS){
            elapsedms = (timeStart - loopTimer) / 1000;
            loopTimer = timeStart;
            led = !led;
            
            Chassis.periodic();
            cs = Chassis.getChassisSpeeds();

            remoteRead();
            previous_mode = remote.leftSwitch();
            prevM = remote.getMouseL();

            chassis_executor();

            gimbal_executor();

            shoot_executor();

            DJIMotor::s_sendValues();
        }
        canHandler1.registerCallback(0x201, 0x20D, DJIMotor::s_getCan1Feedback);
        canHandler2.registerCallback(0x201, 0x20D, DJIMotor::s_getCan2Feedback);
        ThisThread::sleep_for(1ms);
    }
}