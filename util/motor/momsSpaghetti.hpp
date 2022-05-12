#include "mbed.h"
#include "motor.hpp"
#include "../algorithms/pid.hpp"

#ifndef canmotor_hpp
#define canmotor_hpp

//#define sendIDs {0x200,0x1FF,0x2FF}

enum errorCodes{
    MOTOR_DISABLED,
    OUTSIDE_OF_BOUNDS,
}

class CANMotor{
    public:

        static CANMotor motors[12];

        static motorType mTypes[12] = {NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE};

        short motorNumber; //the number of motor this is, canID - 1, because canID is 1-8, arrays are 0-7

        int gearRatio = 1; //the gear ratio of the motor to encoder

        CANHandler::CANBus canBus; //the CANBus this motor is on

        motorType type = NONE; //mode of the motor

        motorMode mode = DISABLED; //mode of the motor

        int bounds[2] = {0,0};

        int16_t angle = 0;
        int16_t velocity = 0;
        int16_t torque = 0;
        int16_t temperature = 0;

        int multiturnAngle = 0;

        PID pidSpeed;
        PID pidPosition;

        int value;

        CANMotor(short canID, CANHandler::CANBus bus, motorType mType = STANDARD, int ratio = 1){
            motorNumber = canID - 1;
            canBus = bus;
            gearRatio = ratio;

            
            type = mType;

            if(type == GM6020){
                motorNumber += 4; 
            }

            if(type == GM6020 && canID <= 4) // Check for them fucking gimblies
                printf("ERROR. IT IS HIGHLY DISCOURAGED OF YOU TO USE CAN BUSSES 1-4 FOR THE GM6020s. YOU WILL HAVE ERRORS.\n YOU DUMB BITCH WHY WOULD YOU (WHO IS LIKELY ME) DO THIS I HAVENT CODED THIS IN DONT MAKE ME CODE THIS IN PLEASE\n");
            if (canID > 8 || canID < 1)
                printf("canID not within correct bounds\n");
            
            value = 0;
            motors[motorNumber] = this;
        }

        ~CANMotor(){
            type = NONE;
        }

        void setValue(int val){
            value = val;
        }

        void tick(){
            
        }




};

#endif