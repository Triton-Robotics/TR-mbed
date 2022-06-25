#include "mbed.h"
#include "../algorithms/pid.hpp"
#include "../communications/newCANHandler.hpp"
#include "../helperFunctions.hpp"
#include "../algorithms/speedtocurrent.hpp"
#include <cmath>
//#include "../communications/canHandler.hpp"
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
static Thread motorFeedbackThread(osPriorityAboveNormal); //threading for Motor::tick()
static Thread motorSendThread(osPriorityNormal); //threading for Motor::tick()

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
    MULTI  = 4,
    POWEROUT = 5,
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


double defaultGimblyPosSettings[5] = {10.88,1.2,18.9,8000,500}; 
double defautlGimblySpeedSettings[5] = {0.13, 8.8, 0, 7500, 1000};
double defautM3508PosSettings[5] = {.48, 0.0137, 4.2, 3000, 300};
double defautM3508SpeedSettings[5] = {1.79, 0.27, 10.57, 15000, 500};

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

        NewCANHandler::CANBus canBus = NewCANHandler::NOBUS; //the CANBus this motor is on

        motorType type = NONE; //mode of the motor

        motorMoveMode mode = OFF; //mode of the motor

        unsigned long timeOfLastFeedback = 0;
        unsigned long timeOfLastPID = 0;

        //static MotorHandler motorHandler;
    public:
        unsigned long timeSinceLastFeedback = 0;

        int maxSpeed = 8723;

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

        int outCap = 16000;

        bool useAbsEncoder = 0;
        static bool sendDebug;
        static bool feedbackDebug;

        CANMotor(bool isErroneousMotor = false){
            
            motorNumber = -1;
            canBus = NewCANHandler::NOBUS;
            gearRatio = 1;
            value = 0;

            type = NONE;
            mode = OFF;

            conflict = isErroneousMotor;
            if(isErroneousMotor)
                mode = ERR;

            powerOut = 0;
        }

        CANMotor(short canID, NewCANHandler::CANBus bus, motorType mType = STANDARD){
            
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
                pidSpeed.setPID(defautM3508SpeedSettings[0],defautM3508SpeedSettings[1],defautM3508SpeedSettings[2]);
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
                printf("[ERROR] THERES A CONFLICT ON BUS [%d] ID [%d]. YOU WILL HAVE ERRORS.\n YOU DUMB BITCH WHY WOULD YOU (WHO IS LIKELY ME) DO THIS. FIX YOUR MOTOR IDS YOU IDIOT.\n",motorNumber/4, motorNumber%4);
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
            canBus = NewCANHandler::NOBUS;
        }

        static void printChunk(NewCANHandler::CANBus bus, short sendID, motorDataType data = POWEROUT){
            printf("Bus:");
            if(bus == NewCANHandler::CANBUS_1)
                printf("BUS_1 |");
            else if(bus == NewCANHandler::CANBUS_2)
                printf("BUS_2 |");
            printf(" sendID:0x%x ",sendIDs[sendID]);
            for(int i = 0; i < 4; i ++){
                if(motorsExist[bus][sendID][i])
                    if(data == POWEROUT)
                        printf("%d ",allMotors[bus][sendID][i]->powerOut);
                    else
                        printf("%d ",allMotors[bus][sendID][i]->getData(data));
                else
                    printf("NA ");
            }
            printf("\n");
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
            if(thread){
                //motorupdatethread.start(tickThread);
                motorSendThread.start(sendThread);
                motorFeedbackThread.start(feedbackThread);

                // canHandlers[0]->attach(&getFeedback);
                // canHandlers[1]->attach(&getFeedback);

                // CAN *can1, *can2;
                // canHandlers[0]->getCAN(can1);
                // canHandlers[1]->getCAN(can2);
                // can1->attach(&getFeedback);
                // can2->attach(&getFeedback);

                // canHandlers[0]->can.attach(&getFeedback);
                // canHandlers[1]->can.attach(&getFeedback);
            }
        }

        void setValue(int val){
            value = val;
        }

        int getValue() {
            return value;
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

        void setPositionPID(float kP, float kI, float kD) {pidPosition.setPID(kP, kI, kD);}
        void setPositionIntegralCap(double cap){pidPosition.setIntegralCap(cap);}
        void setPositionOutputCap(double cap){pidPosition.setOutputCap(cap);}

        void setSpeedPID(float kP, float kI, float kD) {pidSpeed.setPID(kP, kI, kD);}
        void setSpeedIntegralCap(double cap){pidSpeed.setIntegralCap(cap);}
        void setSpeedOutputCap(double cap){pidSpeed.setOutputCap(cap);}

        void setOutput(){
            unsigned long time = us_ticker_read() / 1000;
            if(mode == POW){
                powerOut = value;
            }else if(mode == SPD){
                powerOut = pidSpeed.calculate(value, getData(VELOCITY), time - lastTime);
            }else if(mode == POS){
                if (!useAbsEncoder)
                    powerOut = pidSpeed.calculate(pidPosition.calculate(value, getData(MULTITURNANGLE), time - lastTime), getData(VELOCITY), time - lastTime);
                else
                    powerOut = pidSpeed.calculate(pidPosition.calculate(value, getData(ANGLE), time - lastTime), getData(VELOCITY), time - lastTime);
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

        int getPowerOut() {
            return powerOut;
        }

        int getData(motorDataType data) {
            if(data == POWEROUT)
                return powerOut;
            else if (data != MULTITURNANGLE)
                return motorData[data];
            else if(data == MULTITURNANGLE) 
                return multiTurn;
            return 0;
        }

        void printAllMotorData() {
            printf("ANGL:%d MLTI:%d VELO:%d TORQ:%d TEMP:%d | PWR:%d\n", getData(ANGLE), getData(MULTITURNANGLE), getData(VELOCITY), getData(TORQUE), getData(TEMPERATURE), powerOut);
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
                                if (curAngle > (8191 - Threshold) && lastAngle < Threshold) {
                                    curMotor->multiTurn -= deltaAngle - 8191;
                                }else if (curAngle < Threshold && lastAngle > (8191 - Threshold)) {
                                    curMotor->multiTurn += deltaAngle + 8191;
                                }else
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

        static void sendOneID(NewCANHandler::CANBus bus, short sendIDindex, bool debug = false){
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

        static void getFeedback(){
            //unsigned long time = us_ticker_read() / 1000;
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
                        if(feedbackDebug)
                            allMotors[i][mNum/4][mNum%4]->printAllMotorData();
                        allMotors[i][mNum/4][mNum%4]->timeSinceLastFeedback = us_ticker_read() / 1000 - allMotors[i][mNum/4][mNum%4]->timeOfLastFeedback;
                        allMotors[i][mNum/4][mNum%4]->timeOfLastFeedback = us_ticker_read() / 1000;
                    }else{
                        //printf("[WARNING] YOU HAVE A MOTOR [0x%x] ATTACHED THAT IS NOT INITIALIZED.. WHY\n",msgID);
                    }
                }
            }
            updateMultiTurnPosition();
        }

        void zeroPos(){
            multiTurn = 0;
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

        /**
        * @brief the thread that runs the ever-necessary Motor::tick()
        */
        static void feedbackThread() {
            while (true) {
                getFeedback();
                ThisThread::sleep_for(1ms);
            }
        }
        

        /**
        * @brief the thread that runs the ever-necessary Motor::tick()
        */
        static void sendThread() {
            while (true) {
                for(int i = 0; i < 3; i ++)
                    sendOneID(NewCANHandler::CANBUS_1,i,sendDebug);
                if(sendDebug) printf("\n");
                for(int i = 0; i < 3; i ++)
                    sendOneID(NewCANHandler::CANBUS_2,i,sendDebug);
                if(sendDebug) printf("\n");
                ThisThread::sleep_for(1ms);
            }
        }

        static void tick(bool debug = false, bool printFeedback = false){
            getFeedback();
            //updateMultiTurnPosition();
            for(int i = 0; i < 3; i ++)
                sendOneID(NewCANHandler::CANBUS_1,i,debug);
            if(debug) printf("\n");
            for(int i = 0; i < 3; i ++)
                sendOneID(NewCANHandler::CANBUS_2,i,debug);
            if(debug) printf("\n");
        }

};

#endif

#ifndef canmotor_statics
#define canmotor_statics
CANMotor* CANMotor::allMotors[2][3][4];
NewCANHandler* CANMotor::canHandlers[2];
bool CANMotor::motorsExist[2][3][4];
bool CANMotor::sendDebug = false;
bool CANMotor::feedbackDebug = false;
#endif