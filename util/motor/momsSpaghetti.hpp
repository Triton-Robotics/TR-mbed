#include "mbed.h"
#include "motor.hpp"
#include "../algorithms/pid.hpp"
#include "../communications/causeSpaghettiComesOnceInALifetime.hpp"
#include "../helperFunctions.hpp"
#include <cmath>
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

#define CAN_HANDLER_NUMBER 2 //Number of can handlers

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

    

    private:

        enum motorMoveMode{
            OFF = 0, 
            POS = 1, 
            SPD = 2, 
            POW = 3,
            ERR = 4
        };

        static CANMotor* allMotors[CAN_HANDLER_NUMBER][3][4];

        //static int y;

        //CANMotor* CANMotor::allMotors[];

        static NewCANHandler* canHandlers[CAN_HANDLER_NUMBER];
        //static NewCANHandler canHandlers[CAN_HANDLER_NUMBER];

        static bool motorsExist[CAN_HANDLER_NUMBER][3][4];

        short motorNumber; //the number of motor this is, canID - 1, because canID is 1-8, arrays are 0-7

        int gearRatio = 1; //the gear ratio of the motor to encoder

        CANHandler::CANBus canBus = CANHandler::NOBUS; //the CANBus this motor is on

        motorType type = NONE; //mode of the motor

        motorMoveMode mode = OFF; //mode of the motor

        //static MotorHandler motorHandler;
    public:
        int bounds[2] = {0,0};

        int16_t angle = 0;
        int16_t velocity = 0;
        int16_t torque = 0;
        int16_t temperature = 0;

        int multiturnAngle = 0;

        PID pidSpeed;
        PID pidPosition;

        int value = 0;
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
            if(isErroneousMotor)
                mode = ERR;

            powerOut = 0;
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

            //printf("allMotors[bus][%d][%d]-> = %d\n",motorNumber/4,motorNumber%4, allMotors[bus][motorNumber/4][motorNumber%4]);

            if(motorsExist[bus][motorNumber/4][motorNumber%4] == false /**allMotors[bus][motorNumber/4][motorNumber%4]->motorNumber == -1**/){
                allMotors[bus][motorNumber/4][motorNumber%4] = this;
                motorsExist[bus][motorNumber/4][motorNumber%4] = true;
            }else{
                CANMotor mot(true);
                allMotors[bus][motorNumber/4][motorNumber%4] = &mot;
                printf("[ERROR] THERES A CONFLICT. YOU WILL HAVE ERRORS.\n YOU DUMB BITCH WHY WOULD YOU (WHO IS LIKELY ME) DO THIS. FIX YOUR MOTOR IDS YOU IDIOT.\n");
            }
            

            // if(type == GM6020 && canID <= 4) // Check for them fucking gimblies
            //     printf("ERROR. IT IS HIGHLY DISCOURAGED OF YOU TO USE CAN BUSSES 1-4 FOR THE GM6020s. YOU WILL HAVE ERRORS.\n YOU DUMB BITCH WHY WOULD YOU (WHO IS LIKELY ME) DO THIS I HAVENT CODED THIS IN DONT MAKE ME CODE THIS IN PLEASE\n");
            if (canID > 8 || canID < 1)
                printf("[ERROR] The canID [%d] not within correct bounds\n", canID);
            
            // for(int  i = 0 ; i < 3; i ++){
            //     for(int j = 0; j < 4; j ++){
            //         printf("%d ",allMotors[0][i][j]->motorNumber);
            //     }
            //     printf("\n");
            // }
            // printf("\n---------\n");

            powerOut = 0;
        }

        ~CANMotor(){
            type = NONE;
            mode = OFF;
            motorsExist[canBus][motorNumber/4][motorNumber%4] = false;
            motorNumber = -1;
            canBus = CANHandler::NOBUS;
        }

        // static void setCANHandlers(PinName can1Tx, PinName can1Rx, PinName can2Tx, PinName can2Rx){
        //     canHandlers[0].updateCANs(PinName canRx, PinName canTx);
        //     canHandlers[1]* = &can2;
        //     for(int i = 0; i < 3; i ++){
        //         for(int j = 0; j < 4; j++){
        //             for(int k = 0; k < 2; k++){
        //                 CANMotor m;
        //                 allMotors[k][i][j] = &m;
        //             }
        //         }
        //     }
        // }

        static void setCANHandlers(NewCANHandler* bus_1, NewCANHandler* bus_2){
            canHandlers[0] = bus_1;
            canHandlers[1] = bus_2;
            // for(int i = 0; i < 3; i ++){
            //     for(int j = 0; j < 4; j++){
            //         for(int k = 0; k < 2; k++){
            //             CANMotor m;
            //             allMotors[k][i][j] = &m;
            //         }
            //     }
            // }
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

        void setPosition(int position){
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
            }else if(mode == OFF){
                powerOut = 0;
            }else if(mode == ERR){
                printf("[ERROR] THIS IS AN ERRONEOUS MOTOR. DO NOT ATTEMPT TO SEND IT DATA, DO NOT PASS GO, FIX THIS!\n");
            }
            lastTime = time;

        }

        static void sendOneID(CANHandler::CANBus bus, short sendIDindex){
            int8_t bytes[8]  = {0,0,0,0,0,0,0,0};
            printf("0x%x:\t",sendIDs[sendIDindex]);
            for(int i = 0; i < 4; i++){
                //printf("AL%d:\t",allMotors[bus][sendIDindex][i]);
                if(motorsExist[bus][sendIDindex][i] == true/**allMotors[bus][sendIDindex][i]->motorNumber != -1**/){
                    allMotors[bus][sendIDindex][i]->setOutput();
                    int16_t pO = allMotors[bus][sendIDindex][i]->powerOut;
                    //printf("%d\t",pO);
                    
                    bytes[2*i] = int8_t(pO >> 8);
                    //printf("%d ",pO >> 8);
                    
                    bytes[2*i + 1] = int8_t(pO);
                    //printf("%d ",pO & 0xFF);

                }else{
                    //printf("NA\t");
                }  
            }
            //printf("\n");
            //printf("0x%x:\t",sendIDs[sendIDindex]);
            //printArray(bytes, 8);
            //printf("meh1%d, meh2%d\n",canHandlers[0]->exists,canHandlers[1]->exists);
            //printf("canhandler id is%d\n",canHandlers[bus]);
            if(/**canHandlers[bus] != 0**/canHandlers[bus]->exists == true){
                canHandlers[bus]->rawSend(sendIDs[sendIDindex], bytes);
            }else{
                printf("[ERROR] YOUR CANHANDLERS ARE NOT DEFINED YET. DO THIS BEFORE YOU CALL ANY MOTORS,\n USING [(CANMotor::setCANHandlers(PA_11,PA_12,PB_12,PB_13)], WHERE PA_11, PA_12 ARE TX, RX\n");
            }
        }

        static void getFeedback(){
            for(int i = 0; i < CAN_HANDLER_NUMBER; i ++){
                uint8_t recievedBytes[8] = {0,0,0,0,0,0,0,0};
                int msgID;
                printf("canhandler id is%d\n",canHandlers[i]);
                if(canHandlers[i]->getFeedback(&msgID,recievedBytes)) {
                    int mNum = msgID - 0x201;
                    if(motorsExist[i][mNum/4][mNum%4]){
                        allMotors[i][mNum/4][mNum%4]->angle = (recievedBytes[0]<<8) | recievedBytes[1];
                        allMotors[i][mNum/4][mNum%4]->velocity = (recievedBytes[2]<<8) | recievedBytes[3];
                        allMotors[i][mNum/4][mNum%4]->torque = (recievedBytes[4]<<8) | recievedBytes[5];
                        allMotors[i][mNum/4][mNum%4]->temperature = ((int16_t) recievedBytes[6]);
                    }else{
                        printf("[WARNING] YOU HAVE A MOTOR [0x%x] ATTACHED THAT IS NOT INITIALIZED.. WHY\n",msgID);
                    }
                }
            }
        }

        static void tick(){
            for(int i = 0; i < 3; i ++)
                sendOneID(CANHandler::CANBUS_1,i);
            for(int i = 0; i < 3; i ++)
                sendOneID(CANHandler::CANBUS_2,i);
        }
        // static void tick(){

        // }

};

#endif

#ifndef canmotor_statics
#define canmotor_statics
CANMotor* CANMotor::allMotors[2][3][4];
NewCANHandler* CANMotor::canHandlers[2];
bool CANMotor::motorsExist[2][3][4];
#endif