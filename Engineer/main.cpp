#include "../src/main.hpp"

//DJIRemote myremote(PA_0, PA_1);
//CANHandler canPorts(PA_11,PA_12,PB_12,PB_13);

ChassisSubsystem awesomechassis(4,2,1,3, CANHandler::CANBUS_1, C620);

//Motor yaw(5, CANHandler::CANBUS_1, GIMBLY);
Motor pitch(6, CANHandler::CANBUS_1, GIMBLY);
Motor serializer(7, CANHandler::CANBUS_1, C610);

int neutralval = 0;
int revval = 75;


PWMMotor RFLYWHEEL(D12, neutralval);
PWMMotor LFLYWHEEL(D11, neutralval);

int yawval = 0;
int pitchval = 220;
int maxspeed = 300;
int maxpitchyawchange = 5;

bool checkJam(Motor *mymotor) {
    static int mycount = 0;

    if (mymotor->getData(VELOCITY) == 0) 
        mycount++;
    else
        mycount = 0;
    
    if (mycount > 10)
        return true;
    else
        return false;
}

int main()
{
    Motor::setCANHandler(&canPorts);
    pitch.zeroPos();

    while (true) {
        myremote.remoteUpdate();

        if (myremote.getSwitchData(RSWITCH) == 2) {
            awesomechassis.move(myremote.getStickData(LEFTJOYY, 0, maxspeed), myremote.getStickData(LEFTJOYX, 0, maxspeed), myremote.getStickData(RIGHTJOYX, 0, maxspeed));
        }
        else {
            //yawval+= (int)myremote.getStickData(LEFTJOYY, 0, maxpitchyawchange);
            pitchval+=(int)myremote.getStickData(LEFTJOYY, 0, maxpitchyawchange);

            if (pitchval > 240)
                pitchval = 240;
            if (pitchval < 180)
                pitchval = 180;
            
        }
        
        if (myremote.getSwitchData(LSWITCH) == 1) {
            LFLYWHEEL.set(revval);
            RFLYWHEEL.set(revval);
            serializer.setDesiredSpeed(-70);
        }
        else if (myremote.getSwitchData(LSWITCH) == 2) {
            serializer.setDesiredSpeed(90);
        }
        else if (myremote.getSwitchData(LSWITCH) == 3) {
            LFLYWHEEL.set(neutralval);
            RFLYWHEEL.set(neutralval);
            serializer.setDesiredSpeed(0);
        }

        //yaw.setDesiredPos(yawval);
        pitch.setDesiredPos(220);

        

    }
}

