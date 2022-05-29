#include "../src/main.hpp"

//CANHandler canPorts(PA_11,PA_12,PB_12,PB_13);

//DJIRemote myremote(PA_0, PA_1);
//CANHandler canPorts(PA_11,PA_12,PB_12,PB_13);

NewChassisSubsystem chassis(4,2,1,3, CANHandler::CANBUS_1, C620);

//Motor yaw(5, CANHandler::CANBUS_1, GIMBLY);
CANMotor pitch(6, CANHandler::CANBUS_1, GIMBLY);
CANMotor serializer(7, CANHandler::CANBUS_1, C610);

int neutralval = 0;
int revval = 75;


PWMMotor RFLYWHEEL(D12, neutralval);
PWMMotor LFLYWHEEL(D11, neutralval);

int yawval = 0;
int pitchval = 220;
int maxspeed = 300;
int maxpitchyawchange = 5;

// bool checkJam(Motor *mymotor) {
//     static int mycount = 0;

//     if (mymotor->getData(VELOCITY) == 0) 
//         mycount++;
//     else
//         mycount = 0;
    
//     if (mycount > 10)
//         return true;
//     else
//         return false;
// }

int main()
{
    //Motor::setCANHandler(&canPorts);
    CANMotor::setCANHandlers(&canHandler1, &canHandler2);
    //pitch.zeroPos();
    threadingRemote.start(&remoteThread);
    while (true) {
        

        // if (myremote.getSwitchData(RSWITCH) == 2) {
        //     chassis.move(lY,lX,rX);
        // }
        //chassis.move(3000,0,0);
        chassis.move(lY,lX,rX);

        remotePrint();

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

        CANMotor::tick(lS == 2, rS == 2);

    }
}

