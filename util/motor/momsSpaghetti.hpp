#include "mbed.h"
#include "motor.hpp"
#include "../algorithms/pid.hpp"
#include "../communications/causeSpaghettiComesOnceInALifetime.hpp"
//TR-mbed6/util/communications/causeSpaghettiComesOnceInALifetime.hpp

// #pragma once
// namespace{
//     class MotorHandler{
//         public:
//             void send();
//     };
// }

// #ifndef motorhandler_hpp
//#include "youOnlyGetOneSpaghetti.hpp"
// #endif

//class MotorHandler;

#ifndef canmotor_hpp
#define canmotor_hpp

//#define sendIDs {0x200,0x1FF,0x2FF}

enum errorCodes{
    NO_ERROR,
    MOTOR_CONFLICT,
    MOTOR_DISABLED,
    OUTSIDE_OF_BOUNDS,
};

class CANMotor{

    public:

        

        enum motorMoveMode{
            OFF = 0, 
            POS = 1, 
            SPD = 2, 
            POW = 3,
            ERR = 4
        };

        static CANMotor* allMotors[2][3][4];

        //static int y;

        //CANMotor* CANMotor::allMotors[];

        static NewCANHandler* canHandlers[2];

        short motorNumber; //the number of motor this is, canID - 1, because canID is 1-8, arrays are 0-7

        int gearRatio = 1; //the gear ratio of the motor to encoder

        CANHandler::CANBus canBus = CANHandler::NOBUS; //the CANBus this motor is on

        motorType type = NONE; //mode of the motor

        motorMoveMode mode = OFF; //mode of the motor

        //static MotorHandler motorHandler;

        int bounds[2] = {0,0};

        int16_t angle = 0;
        int16_t velocity = 0;
        int16_t torque = 0;
        int16_t temperature = 0;

        int multiturnAngle = 0;

        PID pidSpeed;
        PID pidPosition;

        int value;
        int16_t powerOut;

        bool conflict; //check for a conflict when running motors

        CANMotor(bool isErroneousMotor = false){
            
            motorNumber = -1;
            canBus = CANHandler::NOBUS;
            gearRatio = 1;
            value = 0;

            type = NONE;
            mode = OFF;

            conflict = isErroneousMotor;
        }

        CANMotor(short canID, CANHandler::CANBus bus, motorType mType = STANDARD){
            
            motorNumber = canID - 1;
            canBus = bus;
            if(mType == GM6020)
                gearRatio = 1;
            else if(mType == M3508)
                gearRatio = 19;
            else if(mType == M2006)
                gearRatio = 36;
            else
                gearRatio = 1;
            value = 0;

            type = mType;
            
            if(type == GM6020){
                motorNumber += 4; 
            }

            //printf("sendID:%d,0x%x\n",motorNumber/4,sendIDs[motorNumber/4]);
            //printf("sendSlot:%d\n",motorNumber%4);

            //printf("allMotors[bus][%d][%d]->motorNumber = %d\n",motorNumber/4,motorNumber%4, allMotors[bus][motorNumber/4][motorNumber%4]);

            if(allMotors[bus][motorNumber/4][motorNumber%4] == 0){
                allMotors[bus][motorNumber/4][motorNumber%4] = this;
            }else{
                CANMotor mot(true);
                allMotors[bus][motorNumber/4][motorNumber%4] = &mot;
                printf("ERROR. THERES A CONFLICT. YOU WILL HAVE ERRORS.\n YOU DUMB BITCH WHY WOULD YOU (WHO IS LIKELY ME) DO THIS. FIX YOUR MOTOR IDS YOU IDIOT.\n");
            }
            

            // if(type == GM6020 && canID <= 4) // Check for them fucking gimblies
            //     printf("ERROR. IT IS HIGHLY DISCOURAGED OF YOU TO USE CAN BUSSES 1-4 FOR THE GM6020s. YOU WILL HAVE ERRORS.\n YOU DUMB BITCH WHY WOULD YOU (WHO IS LIKELY ME) DO THIS I HAVENT CODED THIS IN DONT MAKE ME CODE THIS IN PLEASE\n");
            if (canID > 8 || canID < 1)
                printf("canID not within correct bounds\n");
            
            // for(int  i = 0 ; i < 3; i ++){
            //     for(int j = 0; j < 4; j ++){
            //         printf("%d ",allMotors[0][i][j]->motorNumber);
            //     }
            //     printf("\n");
            // }
        }

        ~CANMotor(){
            type = NONE;
        }

        static void setCANHandlers(PinName can1Tx, PinName can1Rx, PinName can2Tx, PinName can2Rx){
            NewCANHandler can1(can1Tx,can1Rx);
            NewCANHandler can2(can1Tx,can1Rx);
            CANMotor::canHandlers[0] = &can1;
            CANMotor::canHandlers[1] = &can2;
        }

        void setValue(int val){
            value = val;
        }

        void setPower(int power){
            setValue(power);
            mode = POW; 
            setOutput();
        }

        void setSpeed(int speed){
            setValue(speed);
            mode = SPD; 
            setOutput();
        }

        void setPostiion(int position){
            setValue(position);
            mode = POS; 
            setOutput();
        }

        void setOutput(){
            static unsigned long lastTime = 0;
            unsigned long time = us_ticker_read() / 1000;
            if(mode == POW){
                powerOut = value;
            }else if(mode == SPD){
                powerOut += pidSpeed.calculate(value, velocity, time - lastTime);
            }else if(mode == POS){
                powerOut = pidPosition.calculate(value, angle, time - lastTime);
            }
            lastTime = time;

        }

        void getFeedback(){

        }

        // static void sendOneID(CANHandler::CANBus bus, short sendIDindex){
        //     int8_t bytes[] = {
        //         int8_t((allMotors[bus][sendIDindex][0]->powerOut >> 8)),
        //         int8_t((allMotors[bus][sendIDindex][0]->powerOut)),
        //         int8_t((allMotors[bus][sendIDindex][1]->powerOut >> 8)),
        //         int8_t((allMotors[bus][sendIDindex][1]->powerOut)),
        //         int8_t((allMotors[bus][sendIDindex][2]->powerOut >> 8)),
        //         int8_t((allMotors[bus][sendIDindex][2]->powerOut)),
        //         int8_t((allMotors[bus][sendIDindex][3]->powerOut >> 8)),
        //         int8_t((allMotors[bus][sendIDindex][3]->powerOut))};
        //     canHandlers[bus]->rawSend(sendIDs[sendIDindex], bytes);
        // }

        // static void tick(){
        //     for(int i = 0; i < 3; i ++)
        //         sendOneID(CANHandler::CANBUS_1,i);
        // }
        static void tick(){

        }

};

#endif