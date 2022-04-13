#include "mbed.h"
#include "../algorithms/pid.hpp"
#include <cstdint>
#ifndef motor_hpp
#define motor_hpp
#include "../communications/CANMsg.h"
#include "../communications/canHandler.hpp"

//////////////////////////////////////////////
//VERY IMPORTANT TO SET FREQUENCY HERE AND NOW
//////////////////////////////////////////////

//static CAN can1(PA_11,PA_12,1000000);
//static CAN can1(PB_12,PB_13,1000000);
//static CAN can2(PB_12,PB_13,1000000);

enum motorMode {DISABLED, POSITION, SPEED, CURRENT};

enum motorType {
    NONE = 0,
    STANDARD = 1,
    M2006 = 1,
    C620 = 1,
    M3508 = 1,
    GIMBLY = 2,
    GM6020 = 2
};

static int sendIDs[3] = {0x200,0x1FF,0x2FF}; //IDs to send data

static int16_t feedback[2][8][4] = 
{{{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}},
{{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}}}; //Array holding the feedback values of the individual motors

static int totalMotors; //total number of motors in play

static bool motorExists[8] = {0,0,0,0,0,0,0,0};

static int motorOut[2][8] = {{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}}; //All motor output values, depending on what mode they're in.

static int canOutput = 1;

static bool motorDebug = 0;

static motorType types[] = {NONE,NONE,NONE,NONE,NONE,NONE,NONE,NONE};

static CANMsg txMsg(0x000,CANStandard); //Message object reused to send messages to motors
static CANMsg rxMsg; //Message object reused to recieve messages from motors

static motorMode mode[8] = {DISABLED, DISABLED, DISABLED, DISABLED, DISABLED, DISABLED, DISABLED, DISABLED};

//static double PIDValuesPosition[8][3] = {{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0}};
//static double PIDValuesSpeed[8][3] = {{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0}};

static int multiTurnPositionAngle[8] = {0,0,0,0,0,0,0,0};

static PID pidPos[8];
static PID pidSpeed[8];

static CANHandler* canHandles;

class Motor{

    public:
        
    int motorNumber;

    int gearRatio = 19;

    bool isInverted = false;

    CANHandler::CANBus currentBus;

    /**
     * @brief Construct a new Motor object
     * 
     * @param canNum is a number from 1-8 signifying which CAN id is attached, blinking LED on motor controller will show this
     */
    Motor(int canID, CANHandler::CANBus bus, motorType type = STANDARD, int ratio = 19, int inverted = false)
    {
        isInverted = inverted;
        if(type == GM6020){
            gearRatio = 1; //TODO FIND THE ACTUAL GEAR RATIO
            if(canID <= 4){
                printf("ERROR. IT IS HIGHLY DISCOURAGED OF YOU TO USE CAN BUSSES 1-4 FOR THE GM6020s. YOU WILL HAVE ERRORS. DO NOT\n");
            }
        }else{
            gearRatio = ratio;
        }
        motorNumber = canID - 1; //Changes range from 1-8 to 0-7
        totalMotors++;
        motorExists[motorNumber] = 1;
        types[motorNumber] = type;
        //TODO Throw error when motorNumber isnt within the range [0,7]
    }

    ~Motor(){ //DESTRUCTOR
        totalMotors --;
        motorExists[motorNumber] = 0;
        types[motorNumber] = NONE;
    }
    
        /**
     * @brief Set the desired value of this motor
     * 
     * @param value
     * @return int value
     */
    int setDesiredValue(int value){
        if((*canHandles).exists){
            motorOut[currentBus][motorNumber] = value;
            return motorOut[currentBus][motorNumber];
        }else{
            return NULL;
        }
    }

    static void setCANHandler(CANHandler* handle){
        canHandles = handle;
    }

    void setDesiredCurrent(int value) {
        if (isInverted)
            value = -value;
        setDesiredValue(value);
        mode[motorNumber] = CURRENT;
    }

    void setDesiredSpeed(int value) {
        setDesiredValue(value);
        mode[motorNumber] = SPEED;
    }

    /**
     * @brief Set the desired position of this motor in degrees
     * 
     * @param value
     */
    void setDesiredPos(int value) {
        setDesiredValue(value * 8191 * gearRatio / 360);
        mode[motorNumber] = POSITION;
    }

    /**
     * @brief Get the desired value of the motor
     * 
     * @return int value
     */
    int getDesiredValue(){
        return motorOut[currentBus][motorNumber];
    }

    
    /**
     * @brief Returns angle of motor
     * 
     * @return int 
     */
    int getAngle(){
        return feedback[currentBus][motorNumber][0];
    }

    int getMultiTurnAngle(){
        return multiTurnPositionAngle[motorNumber];
    }

    static int staticAngle(CANHandler::CANBus bus, int motorID){
        return feedback[bus][motorID][0];
    }

    void zeroPos() {
        multiTurnPositionAngle[motorNumber] = 0;
    }

    static void staticZeroPos(int motorID) {
        multiTurnPositionAngle[motorID] = 0;
    }

    /**
     * @brief Returns speed of motor
     * 
     * @return int 
     */
    int getSpeed(){
        return feedback[currentBus][motorNumber][1];
    }

    static int staticSpeed(CANHandler::CANBus bus, int motorID) {
        return feedback[bus][motorID][1];
    }

    /**
     * @brief Returns torque of motor
     * 
     * @return int 
     */
    int getTorque(){
        return feedback[currentBus][motorNumber][2];
    }

    /**
     * @brief Returns temperature of motor
     * 
     * @return int 
     */
    int getTemperature(){
        return feedback[currentBus][motorNumber][3];
    }

    void setPositionPID(double Kp, double Ki, double Kd){
        pidPos[motorNumber].setPID(Kp,Ki,Kd);
    }

    void setPositionIntegralCap(double cap) {
        pidPos[motorNumber].setIntegralCap(cap);
    }

    void setPositionOutputCap(double cap) {
        pidPos[motorNumber].setOutputCap(cap);
    }

    void setSpeedPID(double Kp, double Ki, double Kd){
        pidSpeed[motorNumber].setPID(Kp,Ki,Kd);
    }

    void setSpeedIntegralCap(double cap) {
        pidSpeed[motorNumber].setIntegralCap(cap);
    }

    void setSpeedOutputCap(double cap) {
        pidSpeed[motorNumber].setOutputCap(cap);
    }
    
    /**
     * @brief Prints a CANMessage nicely
     * 
     * @param msg 
     */
    static void printMsg(CANMessage& msg)
    {
        printf("  ID      = 0x%.3x\r\n", msg.id);
        printf("  Type    = %d\r\n", msg.type);
        printf("  Format  = %d\r\n", msg.format);
        printf("  Length  = %d\r\n", msg.len);
        printf("  Data    =");
        for (int i = 0; i < msg.len; i++)
            printf(" 0x%.2X", msg.data[i]);
        printf("\r\n");
    }

    /**
     * @brief Get feedback back from the motor
     * 
     */
    static void getFeedback(CANHandler::CANBus bus){
        uint8_t recievedBytes[8] = {0,0,0,0,0,0,0,0};
        if ((*canHandles).getFeedback(recievedBytes,bus)) {
            int motorID = rxMsg.id-0x201;
            if(motorID >= 8){
                motorID -= 4;
            }
   
            feedback[bus][motorID][0] = 0 | (recievedBytes[0]<<8) | recievedBytes[1];
            feedback[bus][motorID][1] = 0 | (recievedBytes[2]<<8) | recievedBytes[3];
            feedback[bus][motorID][2] = 0 | (recievedBytes[4]<<8) | recievedBytes[5];
            feedback[bus][motorID][3] = ((int16_t) recievedBytes[6]);

            //printf("Motor 0x%x:\tAngle (0,8191):%d\tSpeed  ( RPM ):%d\tTorque ( CUR ):%d\tTemperature(C):%d \n",rxMsg.id,feedback[motorID][0],feedback[motorID][1],feedback[motorID][2],feedback[motorID][3]);
        }
        //CAN Recieving from feedback IDs
    }

    /**
     * @brief Updates global array for multiTurnPositionontrol
     */
    static void multiTurnPositionControl(CANHandler::CANBus bus) {
        int Threshold = 3000;

        static int lastMotorAngle[8] = {0,0,0,0,0,0,0,0};

        for (int i = 0; i < 7; i++) {
            if (abs(staticSpeed(bus,i)) < 100) {
                if ( staticAngle(bus,i) > (8191 - Threshold) && lastMotorAngle[i] < Threshold)
                    multiTurnPositionAngle[i] += -(staticAngle(bus,i) - 8191) - lastMotorAngle[i];

                else if (staticAngle(bus,i) < Threshold && lastMotorAngle[i] > (8191 - Threshold))
                    multiTurnPositionAngle[i] -= -(staticAngle(bus,i) - 8191) - lastMotorAngle[i];
                else 
                    multiTurnPositionAngle[i] += staticAngle(bus,i) - lastMotorAngle[i];
            }
            else {
                int delta = staticAngle(bus,i) - lastMotorAngle[i]; // 0 to 199 POS// 8000 to 128 NEG
                if(staticSpeed(bus,i) < 0 && delta > 0){ //neg skip
                    multiTurnPositionAngle[i] += (delta - 8191);
                }else if(staticSpeed(bus,i) > 0 && delta < 0){ //pos skip
                    multiTurnPositionAngle[i] += (delta + 8191);
                }else { //pos no skip or neg no skip same case
                    multiTurnPositionAngle[i] += delta;
                }
            }
            lastMotorAngle[i] = staticAngle(bus,i);

        }
      
    }

    /**
     * @brief Returns specified data of specified motor
     * canBus is a field between 1 and 8, specifing the can bus of the motor
     * dataNumber is the element of data you want
     * Angle: 0
     * Speed: 1
     * Torque: 2
     * Temperature: 3
     * 
     * @return int 
     */
    static int getData(CANHandler::CANBus bus, int canBus, int dataNumber){
        return feedback[bus][canBus -1][dataNumber];
    }

    /**
     * @brief send all motor values after setting them in setDesiredValues
     * 
     */
    static void sendValues(CANHandler::CANBus bus){
        //CAN Sending to the two sending IDs
        static unsigned long lastTime[8] = {0};
        unsigned long Time = us_ticker_read() / 1000;
        if(motorExists[0] || motorExists[1] || motorExists[2] || motorExists[3]){
            int16_t outputArray[4] = {0, 0, 0, 0};
            for (int i = 0; i < 4; i++) {
                unsigned long timeDifference = Time - lastTime[i];
                if (mode[i] == DISABLED)
                    outputArray[i] = 0;
                else if (mode[i] == POSITION)
                    outputArray[i] = pidPos[i].calculate(motorOut[bus][i],multiTurnPositionAngle[i],timeDifference);
                    //-PIDPositionError(motorOut1[i], i);
                else if (mode[i] == SPEED)
                    outputArray[i] += pidSpeed[i].calculate(motorOut[bus][i],multiTurnPositionAngle[i],timeDifference);
                    //-PIDSpeedError(motorOut1[i], i);
                else if (mode[i] == CURRENT) {
                    outputArray[i] = motorOut[bus][i];
                }
            }
            rawSend(sendIDs[0], outputArray[0], outputArray[1], outputArray[2], outputArray[3], bus);
        }
        if(motorExists[4] || motorExists[5] || motorExists[6] || motorExists[7]){
            int16_t outputArray[4] = {0, 0, 0, 0};
            int16_t outputArrayGM6020[4] = {0, 0, 0, 0};
            bool doSend[2] = {false,false};
            for (int i = 0; i < 4; i++) {
                unsigned long timeDifference = Time - lastTime[i+4];
                if(types[i+4] == STANDARD){
                    if (mode[i+4] == DISABLED){
                        outputArray[i] = 0;
                    }else if (mode[i+4] == POSITION){
                        outputArray[i] = pidPos[i+4].calculate(motorOut[bus][i+4],multiTurnPositionAngle[i+4],timeDifference);
                        //-PIDPositionError(motorOut2[i], i+4);
                        doSend[0] = true;
                    }else if (mode[i+4] == SPEED){
                        outputArray[i] += pidSpeed[i+4].calculate(motorOut[bus][i+4],staticSpeed(bus,i+4),timeDifference);
                        //-PIDSpeedError(motorOut2[i], i+4);
                        doSend[0] = true;
                    }else if (mode[i+4] == CURRENT) {
                        outputArray[i] = motorOut[bus][i+4];
                        doSend[0] = true;
                    }
                }else if(types[i+4] == GM6020){
                    if (mode[i+4] == DISABLED){
                        outputArrayGM6020[i] = 0;
                    }else if (mode[i+4] == POSITION){
                        outputArrayGM6020[i] = pidPos[i+4].calculate(motorOut[bus][i+4],multiTurnPositionAngle[i+4],timeDifference);
                        doSend[1] = true;
                    }else if (mode[i+4] == SPEED){
                        outputArrayGM6020[i] += pidSpeed[i+4].calculate(motorOut[bus][i+4],staticSpeed(bus,i+4),timeDifference);
                        printf("\t\t\t\tCurrent given:%d\n",outputArrayGM6020[i]);
                        doSend[1] = true;
                    }else if (mode[i+4] == CURRENT) {
                        outputArrayGM6020[i] = motorOut[bus][i+4];
                        doSend[1] = true;
                    }
                }
            }
            if(doSend[0])
                rawSend(sendIDs[1], outputArray[0], outputArray[1], outputArray[2], outputArray[3], bus);
            if(doSend[1])
                rawSend(sendIDs[2], outputArrayGM6020[0], outputArrayGM6020[1], outputArrayGM6020[2], outputArrayGM6020[3], bus);
        }
    }

    /**
     * @brief Raw sending of CAN Messages
     * 
     * @param id the CAN ID you're sending to
     * @param data1 data to the first motor
     * @param data2 data to the second motor
     * @param data3 data to the third motor
     * @param data4 data to the fourth
     */
    static void rawSend(int id, int data1, int data2, int data3, int data4, CANHandler::CANBus bus){
        int motorSending[4] = {data1,data2,data3,data4};

        int8_t sentBytes1[8] = {0,0,0,0,0,0,0,0};
        for(int i = 0; i < 4;  i ++){
            sentBytes1[(2*i)+1] = motorSending[i] & (0xFF);
            sentBytes1[2*i] = (motorSending[i] >> 8) & (0xFF);
        }
        canHandles->rawSend(id,sentBytes1,bus);
    }

    /**
     * @brief Function that should be called by the user every tick, runs necessary elements for CAN motors to work.
     * 
     */
    static void tick(){
        multiTurnPositionControl(CANHandler::CANBUS_1);
        getFeedback(CANHandler::CANBUS_1);
        sendValues(CANHandler::CANBUS_1);
        
        multiTurnPositionControl(CANHandler::CANBUS_2);
        getFeedback(CANHandler::CANBUS_2);
        sendValues(CANHandler::CANBUS_2);
    }

    /**
     * @brief equivalent to tick just for Ming
     * 
     */
    static void update(){
        tick();
    }


};

#endif //motor_h