#include "mbed.h"
#include "../algorithms/pid.hpp"
#include "../communications/causeSpaghettiComesOnceInALifetime.hpp"
#include "../helperFunctions.hpp"
#include <cmath>
#include "../communications/canHandler.hpp"
//TR-mbed6/util/communications/causeSpaghettiComesOnceInALifetime.hpp

///////////////////////////////////////////////////////////////////////
//  __  __           _      ___                 _        _   _   _ 
// |  \/  |___ _ __ ( )___ / __|_ __  __ _ __ _| |_  ___| |_| |_(_)
// | |\/| / _ \ '  \|/(_-< \__ \ '_ \/ _` / _` | ' \/ -_)  _|  _| |
// |_|  |_\___/_|_|_| /__/ |___/ .__/\__,_\__, |_||_\___|\__|\__|_|
//                             |_|        |___/                    
//
///////////////////////////////////////////////////////////////////////

#define CAN_HANDLER_NUMBER 2 //Number of can handlers
#ifndef canmotor_hpp
#define canmotor_hpp

static int sendIDs[3] = {0x200,0x1FF,0x2FF}; //IDs to send data
static Thread motorupdatethread(osPriorityHigh); //threading for Motor::tick()

enum errorCodes{
    NO_ERROR,
    MOTOR_CONFLICT,
    MOTOR_DISABLED,
    OUTSIDE_OF_BOUNDS,
};

enum motorDataType {
    ANGLE = 0,
    VELOCITY = 1,
    TORQUE = 2,
    TEMPERATURE = 3,
    MULTITURNANGLE = 4,
};


enum motorType {
    NONE = 0,
    STANDARD = 1, //identifier for all motors that use the standard can protocol, used by the C610 and C620
    
    C610 = 4,
    M2006 = 4,
    
    C620 = 3,
    M3508 = 3, 
    //keep in mind that in the constructor, this is only used to 
    //set the default pid values and gear ratio. The motortype will 
    //be changed to STANDARD, because thats what the code uses.

    GIMBLY = 2, //Gimblyyyyyyyyyy
    GM6020 = 2
};

static double defaultGimblyPosSettings[5] = {15,1.797,10,8000,500}; 
static double defautlGimblySpeedSettings[5] = {41.980, 6.311, 19.960, 15000, 1000};
static double defautM3508PosSettings[5] = {.128, 1.029, 15.405, 3000, 300};
static double defautM3508SpeedSettings[5] = {3.091, 0.207, 4.707, 15000, 500};

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

        //angle | velocity | torque | temperature
        int16_t motorData[4] = {0,0,0,0};
        int multiTurn = 0;
        int lastMotorAngle = 0;

        PID pidSpeed;
        PID pidPosition;

        int value = 0;
        int16_t powerOut;

        bool conflict; //check for a conflict when running motors

        unsigned long lastTime = 0;

        int16_t outCap = 800;

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
                pidSpeed.setPID(defautlGimblySpeedSettings[0],defautlGimblySpeedSettings[1],defautlGimblySpeedSettings[2]);
                pidSpeed.setOutputCap(defautlGimblySpeedSettings[3]);
                pidSpeed.setIntegralCap(defautlGimblySpeedSettings[4]);
                
                pidPosition.setPID(defaultGimblyPosSettings[0],defaultGimblyPosSettings[1],defaultGimblyPosSettings[2]);
                pidPosition.setOutputCap(defaultGimblyPosSettings[3]);
                pidPosition.setIntegralCap(defaultGimblyPosSettings[4]);
            }else if(type == M3508){
                pidSpeed.setPID(1,0,0);
                pidSpeed.setOutputCap(defautM3508SpeedSettings[3]);
                pidSpeed.setIntegralCap(defautM3508SpeedSettings[4]);
                
                pidPosition.setPID(defautM3508PosSettings[0],defautM3508PosSettings[1],defautM3508PosSettings[2]);
                pidPosition.setOutputCap(defautM3508PosSettings[3]);
                pidPosition.setIntegralCap(defautM3508PosSettings[4]);
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

        static void setCANHandlers(NewCANHandler* bus_1, NewCANHandler* bus_2, bool thread = true){
            canHandlers[0] = bus_1;
            canHandlers[1] = bus_2;
            if(thread)
                motorupdatethread.start(tickThread);
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
            unsigned long time = us_ticker_read() / 1000;
            if(mode == POW){
                powerOut = value;
            }else if(mode == SPD){
                powerOut = value + pidSpeed.calculate(value, getData(VELOCITY), time - lastTime);
                //printFloat(powerOut, 2, 1);
            }else if(mode == POS){
                powerOut = pidPosition.calculate(value, multiTurn, time - lastTime);
                //printf("DES:%d,ACT:%d\t",value,multiTurn);
            }else if(mode == OFF){
                powerOut = 0;
            }else if(mode == ERR){
                printf("[ERROR] THIS IS AN ERRONEOUS MOTOR. DO NOT ATTEMPT TO SEND IT DATA, DO NOT PASS GO, DO NOT COLLECT $200, FIX THIS!\n");
            }

            if(powerOut > outCap)
                powerOut = outCap;
            else if(powerOut < -outCap)
                powerOut = -outCap;
            lastTime = time;

        }

        int getData(motorDataType data) {
            if (data != MULTITURNANGLE)
                return motorData[data];
            else 
                return multiTurn;

        }

        void printAllMotorData() {
            printf("ANGL:%d MLTI:%d VELO:%d TORQ:%d TEMP:%d\n", getData(ANGLE), getData(MULTITURNANGLE), getData(VELOCITY), getData(TORQUE), getData(TEMPERATURE));
        }

        static void updateMultiTurnPosition() {
            int Threshold = 3000; // From 0 - 8191
            int curAngle, lastAngle, deltaAngle, speed;
            CANMotor *curMotor;
            for(int x = 0; x < CAN_HANDLER_NUMBER; x++){
                for (int y = 0; y < 3; y++) {
                    for (int z = 0; z < 4; z++) {
                        if (motorsExist[x][y][z]) {
                            curMotor = allMotors[x][y][z];
                            
                            curAngle = curMotor->getData(ANGLE);
                            lastAngle = curMotor->lastMotorAngle;
                            speed = curMotor->getData(VELOCITY);
                            deltaAngle = curAngle - lastAngle;

                            if (abs(speed) < 100) {
                                if (curAngle > (8191 - Threshold) && lastAngle < Threshold)
                                    curMotor->multiTurn -= deltaAngle + 8191;
                                else if (curAngle < Threshold && lastAngle > (8191 - Threshold))
                                    curMotor->multiTurn += deltaAngle + 8191;
                                else
                                    curMotor->multiTurn += deltaAngle;
                            }
                            else {
                                if (speed < 0 && deltaAngle > 0)  // neg skip
                                    curMotor->multiTurn += deltaAngle - 8191;
                                else if (speed > 0 && deltaAngle < 0) // pos skip
                                    curMotor->multiTurn += deltaAngle + 8191;
                                else 
                                    curMotor->multiTurn += deltaAngle; 
                            }
                            curMotor->lastMotorAngle = curAngle;
                        }
                    }
                }
            }
        }

        static void sendOneID(CANHandler::CANBus bus, short sendIDindex, bool debug = false){
            int8_t bytes[8]  = {0,0,0,0,0,0,0,0};
            if(debug) printf("0x%x:\t",sendIDs[sendIDindex]);
            for(int i = 0; i < 4; i++){
                //printf("AL%d:\t",allMotors[bus][sendIDindex][i]);
                if(motorsExist[bus][sendIDindex][i] == true/**allMotors[bus][sendIDindex][i]->motorNumber != -1**/){
                    allMotors[bus][sendIDindex][i]->setOutput();
                    int16_t pO = allMotors[bus][sendIDindex][i]->powerOut;
                    if(debug) printf("%d\t",pO);
                    
                    bytes[2*i] = int8_t(pO >> 8);
                    //printf("%d ",pO >> 8);
                    
                    bytes[2*i + 1] = int8_t(pO);
                    //printf("%d ",pO & 0xFF);

                }else{
                    if(debug) printf("NA\t");
                }  
            }
            //if(debug) printf("\n");
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

        static void getFeedback(bool printFeedback = false){
            for(int i = 0; i < CAN_HANDLER_NUMBER; i ++){
                uint8_t recievedBytes[8] = {0,0,0,0,0,0,0,0};
                int msgID;
                // printf("canhandler id is%d\n",canHandlers[i]);
                if(canHandlers[i]->getFeedback(&msgID,recievedBytes)) {
                    int mNum = msgID - 0x201;
                    if(motorsExist[i][mNum/4][mNum%4]){
                        allMotors[i][mNum/4][mNum%4]->motorData[ANGLE] = (recievedBytes[0]<<8) | recievedBytes[1];
                        allMotors[i][mNum/4][mNum%4]->motorData[VELOCITY] = (recievedBytes[2]<<8) | recievedBytes[3];
                        allMotors[i][mNum/4][mNum%4]->motorData[TORQUE] = (recievedBytes[4]<<8) | recievedBytes[5];
                        allMotors[i][mNum/4][mNum%4]->motorData[TEMPERATURE] = ((int16_t) recievedBytes[6]);
                        if(printFeedback)
                            allMotors[i][mNum/4][mNum%4]->printAllMotorData();
                    }
                }
            }
            updateMultiTurnPosition();
        }

        /**
        * @brief the thread that runs the ever-necessary Motor::tick()
        */
        static void tickThread() {
            while (true) {
                tick();
                ThisThread::sleep_for(1ms);
            }
        }

        static void tick(bool debug = false, bool printFeedback = false){
            getFeedback(printFeedback);
            //updateMultiTurnPosition();
            for(int i = 0; i < 3; i ++)
                sendOneID(CANHandler::CANBUS_1,i,debug);
            if(debug) printf("\n");
            for(int i = 0; i < 3; i ++)
                sendOneID(CANHandler::CANBUS_2,i,debug);
            if(debug) printf("\n");
        }

};

#endif

#ifndef canmotor_statics
#define canmotor_statics
CANMotor* CANMotor::allMotors[2][3][4];
NewCANHandler* CANMotor::canHandlers[2];
bool CANMotor::motorsExist[2][3][4];
#endif