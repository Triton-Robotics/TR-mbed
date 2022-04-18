#include "mbed.h"
#include "../algorithms/pid.hpp"
#include <cstdint>
#ifndef motor_hpp
#define motor_hpp
#include "../communications/CANMsg.h"
#include "../communications/canHandler.hpp"

enum motorMode {DISABLED, POSITION, SPEED, CURRENT};

enum motorType {
    NONE = 0,
    STANDARD = 1, //all motors that use the standard can protocol, used by the C610 and C620
    
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

enum dataType {
    ANGLE = 0,
    VELOCITY = 1,
    TORQUE = 2,
    TEMPERATURE = 3,
    MULTITURNANGLE = 5
};

static int sendIDs[3] = {0x200,0x1FF,0x2FF}; //IDs to send data

static int16_t feedback[2][8][4] = 
{{{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}},
{{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}}}; //Array holding the feedback values of the individual motors

static int totalMotors; //total number of motors in play

static bool motorExists[2][8] = {{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};

static int motorOut[2][8] = {{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}}; //All motor output values, depending on what mode they're in.

static bool motorDebug = 0;

static motorType types[2][8] = {{NONE,NONE,NONE,NONE,NONE,NONE,NONE,NONE},{NONE,NONE,NONE,NONE,NONE,NONE,NONE,NONE}};

static CANMsg txMsg(0x000,CANStandard); //Message object reused to send messages to motors
static CANMsg rxMsg; //Message object reused to recieve messages from motors

static motorMode mode[2][8] = {{DISABLED, DISABLED, DISABLED, DISABLED, DISABLED, DISABLED, DISABLED, DISABLED},{DISABLED, DISABLED, DISABLED, DISABLED, DISABLED, DISABLED, DISABLED, DISABLED}};

//static double PIDValuesPosition[8][3] = {{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0}};
//static double PIDValuesSpeed[8][3] = {{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0}};

static int multiTurnPositionAngle[2][8] = {{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};

static PID pidPos[2][8];
static PID pidSpeed[2][8];

static double defaultGimblyPosPID[3] = {5,0,3}; // pos output cap of 8000
static double defaultGimblySpeedPID[3] = {50,10,0}; //speed icap of 1000

//static double defaultM3508PosPID[3] = {?, ?, ?}; 
static double defaultM3508SpeedPID[3] = {.5, .2, 7}; //ICAP OF 400 REQUIRED

static CANHandler* canHandles;

static Thread thread(osPriorityHigh);


/**
     * @brief Construct a new Motor object
     * 
     * @param canID is a number from 1-8 signifying which CAN id is attached, blinking LED on motor controller will show this
     * @param bus bus that the motor is attached to. This is either CANBUS_1 or CANBUS_2 
     * @param type Type of motor controller.  
     * @param ratio Gear ratio inbetween the motor -> output shaft. 
     *   Ex: The M3508s have a 19:1 planetary gearbox, so this value would be 19.
     * @param inverted Inverts the output direction of the motor shaft.
     */ 
class Motor{
    public:
        
    int motorNumber;

    int gearRatio;

    bool isInverted = false;

    CANHandler::CANBus currentBus;

    Motor(int canID, CANHandler::CANBus bus, motorType type = STANDARD, int ratio = -1, int inverted = false)
    {
        isInverted = inverted;
        motorNumber = canID - 1; //Changes range from 1-8 to 0-7
        totalMotors++;
        motorExists[bus][motorNumber] = 1;
        types[bus][motorNumber] = type;
        currentBus = bus;

        if(ratio == -1){
            ratio = 1; //default gearbox ratio is no gearbox ratio, 1
        }

        if(type == GM6020) {
            ratio = 1;
            setPositionPID(defaultGimblyPosPID[0], defaultGimblyPosPID[1], defaultGimblyPosPID[2]);
            //setSpeedIntegralCap(?);
            setPositionOutputCap(8000);

            setSpeedPID(defaultGimblySpeedPID[0], defaultGimblySpeedPID[1], defaultGimblySpeedPID[2]);
            setSpeedOutputCap(20000);
            setSpeedIntegralCap(1000);
        }else if(type == M3508){
            ratio = 19;
            types[bus][motorNumber] = STANDARD;

            //setPositionPID(defaultM3508PosPID[0], defaultM3508PosPID[1], defaultM3508PosPID[2]);
            //setSpeedIntegralCap(?);
            setPositionOutputCap(8000);

            setSpeedPID(defaultM3508SpeedPID[0], defaultM3508SpeedPID[1], defaultM3508SpeedPID[2]);
            setSpeedIntegralCap(1000);
            setSpeedOutputCap(8000);
        }else if(type == M2006){
            ratio = 36;
            types[bus][motorNumber] = STANDARD;

            //setPositionPID(defaultM2006PosPID[0], defaultM2006PosPID[1], defaultM2006PosPID[2]);
            //setSpeedIntegralCap(?);
            //setPositionOutputCap(?);

            //setSpeedPID(defaultM2006SpeedPID[0], defaultM2006SpeedPID[1], defaultM2006SpeedPID[2]);
            //setSpeedIntegralCap(?);
            //setSpeedOutputCap(?);
        }

        
        gearRatio = ratio;

        if(type == GM6020 && canID <= 4) // Check for them fucking gimblies
            printf("ERROR. IT IS HIGHLY DISCOURAGED OF YOU TO USE CAN BUSSES 1-4 FOR THE GM6020s. YOU WILL HAVE ERRORS.\n YOU DUMB BITCH WHY WOULD YOU (WHO IS LIKELY ME) DO THIS I HAVENT CODED THIS IN DONT MAKE ME CODE THIS IN PLEASE\n");
        if (canID > 8 || canID < 1)
            printf("canID not within correct bounds\n");

        
        
    }

    ~Motor(){ //DESTRUCTOR
        totalMotors --;
        motorExists[currentBus][motorNumber] = 0;
        types[currentBus][motorNumber] = NONE;
    }
    
    /**
     * @brief Set the desired value of this motor
     * 
     * @param value 
     * @return int value
     */
    int setDesiredValue(int value){
        if (isInverted)
            value = -value;
        if(canHandles->exists){
            motorOut[currentBus][motorNumber] = value;
            return motorOut[currentBus][motorNumber];
        }else{
            printf("ERROR: CAN Bus does not exist!! \n");
            return NULL;
        }
    }

    /**
     * @brief Import function to setup the CANHandler constructor. Must be run at the start of the main.cpp file
     * 
     * @param CANPorts An address to the CANHandler class
     */
    static void setCANHandler(CANHandler* CANPorts){
        canHandles = CANPorts;
        thread.start(tickThread);
    }

    void setDesiredCurrent(int value) {
        setDesiredValue(value);
        mode[currentBus][motorNumber] = CURRENT;
    }

    void setDesiredSpeed(int value) {
        setDesiredValue(value * gearRatio);
        mode[currentBus][motorNumber] = SPEED;
    }

    /**
     * @brief Set the desired position of the motor's output shaft in DEGREEs
     * 
     * @param value
     */
    void setDesiredPos(int value) {
        setDesiredValue(value * 8191 * gearRatio / 360);
        mode[currentBus][motorNumber] = POSITION;
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
     * @brief Returns data from motor
     * @param data Either ANGLE, MULTITURNANGLE, VELOCITY, TORQUE, or TEMPERATURE
     * @return integer data 
     */

    int getData(dataType data) {
        if (data != MULTITURNANGLE) {
            return feedback[currentBus][motorNumber][data];
        }
        else 
            return multiTurnPositionAngle[currentBus][motorNumber];
    }

    /**
     * @brief Returns data from motor
     * @param data Either ANGLE, VELOCITY, TORQUE, or TEMPERATURE
     * @return integer data 
     */
    static int getStaticData(CANHandler::CANBus bus, int motorID, dataType data){
        return feedback[bus][motorID][data];
    }

    void zeroPos() {
        multiTurnPositionAngle[currentBus][motorNumber] = 0;
    }

    static void staticZeroPos(int motorID,CANHandler::CANBus bus) {
        multiTurnPositionAngle[bus][motorID] = 0;
    }

    void setPositionPID(double Kp, double Ki, double Kd){
        pidPos[currentBus][motorNumber].setPID(Kp,Ki,Kd);
    }

    void setPositionIntegralCap(double cap) {
        pidPos[currentBus][motorNumber].setIntegralCap(cap);
    }

    void setPositionOutputCap(double cap) {
        pidPos[currentBus][motorNumber].setOutputCap(cap);
    }

    void setSpeedPID(double Kp, double Ki, double Kd){
        pidSpeed[currentBus][motorNumber].setPID(Kp,Ki,Kd);
    }

    void setSpeedIntegralCap(double cap) {
        pidSpeed[currentBus][motorNumber].setIntegralCap(cap);
    }

    void setSpeedOutputCap(double cap) {
        pidSpeed[currentBus][motorNumber].setOutputCap(cap);
    }

    /**
     * @brief Get feedback back from the motors attached to a CANBUS
     * @param bus BUS that you want to get motor data from. Either CANBUS_1 or CANBUS_2 
     */
    static void getFeedback(CANHandler::CANBus bus){
        uint8_t recievedBytes[8] = {0,0,0,0,0,0,0,0};
        int msgID;
        if(canHandles->getFeedback(&msgID,recievedBytes,bus)) { // As long as theres a new message, keep on runnin!
            
            int motorID = msgID-0x201;
            if(motorID >= 8){ // Shifting IDs to automatically handle gimblies 
                motorID -= 4;
            }
            
            feedback[bus][motorID][0] = (recievedBytes[0]<<8) | recievedBytes[1];
            feedback[bus][motorID][1] = (recievedBytes[2]<<8) | recievedBytes[3];
            feedback[bus][motorID][2] = (recievedBytes[4]<<8) | recievedBytes[5];
            feedback[bus][motorID][3] = ((int16_t) recievedBytes[6]);

            // for (int i = 0; i < 8; i++)
            //      printf(" 0x%.2X", recievedBytes[i]);
            //printf("bus: %d motorID: %d \n", bus, motorID);
            //printf("Motor 0x%x:\tAngle (0,8191):%d\tSpeed  ( RPM ):%d\tTorque ( CUR ):%d\tTemperature(C):%d \n",rxMsg.id,feedback[motorID][0],feedback[motorID][1],feedback[motorID][2],feedback[motorID][3]);
        }
    }

    /**
     * @brief Updates global array for multiTurnPositionontrol
     * @param bus BUS that you want to get update multiTurnPosition from. Either CANBUS_1 or CANBUS_2  
     */
    static void multiTurnPositionControl(CANHandler::CANBus bus) {
        int Threshold = 3000; // From 0 - 8191

        static int lastMotorAngle[2][8] = {{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};

        for (int i = 0; i < 7; i++) {
            if (abs(getStaticData(bus,i, VELOCITY)) < 100) { // Check for slow speeds DJI's speed readout is shit when slow rpm
                if ( getStaticData(bus,i, ANGLE) > (8191 - Threshold) && lastMotorAngle[bus][i] < Threshold) // Check to see whether the encoder reading "looped"
                    multiTurnPositionAngle[bus][i] += -(getStaticData(bus,i, ANGLE) - 8191) - lastMotorAngle[bus][i];

                else if (getStaticData(bus,i, ANGLE) < Threshold && lastMotorAngle[bus][i] > (8191 - Threshold))
                    multiTurnPositionAngle[bus][i] -= -(getStaticData(bus,i, ANGLE) - 8191) - lastMotorAngle[bus][i];
                else 
                    multiTurnPositionAngle[bus][i] += getStaticData(bus,i, ANGLE) - lastMotorAngle[bus][i];
                //printf("\t\t\t Using slower multiturnposition control\n");
            }
            else {
                int delta = getStaticData(bus,i, ANGLE) - lastMotorAngle[bus][i]; // 0 to 199 POS// 8000 to 128 NEG
                if(getStaticData(bus,i, VELOCITY) < 0 && delta > 0){ //neg skip
                    multiTurnPositionAngle[bus][i] += (delta - 8191);
                }else if(getStaticData(bus,i, VELOCITY) > 0 && delta < 0){ //pos skip
                    multiTurnPositionAngle[bus][i] += (delta + 8191);
                }else { //pos no skip or neg no skip same case
                    multiTurnPositionAngle[bus][i] += delta;
                }
            }
            lastMotorAngle[bus][i] = getStaticData(bus,i, ANGLE);

        }
      
    }

    /**
     * @brief send all motor values after setting them in setDesiredValues
     * 
     */
    static void sendValues(CANHandler::CANBus bus){
        //CAN Sending to the two sending IDs
        static unsigned long lastTime[8] = {0,0,0,0,0,0,0,0};
        unsigned long Time = us_ticker_read() / 1000;
        if(motorExists[bus][0] || motorExists[bus][1] || motorExists[bus][2] || motorExists[bus][3]){
            int16_t outputArray[4] = {0, 0, 0, 0};
            for (int i = 0; i < 4; i++) {
                unsigned long timeDifference = Time - lastTime[i];
                if (mode[bus][i] == DISABLED)
                    outputArray[i] = 0;
                else if (mode[bus][i] == POSITION)
                    outputArray[i] = pidPos[bus][i].calculate(motorOut[bus][i],multiTurnPositionAngle[bus][i],timeDifference);
                    //-PIDPositionError(motorOut1[i], i);
                else if (mode[bus][i] == SPEED)
                    outputArray[i] += pidSpeed[bus][i].calculate(motorOut[bus][i],getStaticData(bus,i, VELOCITY),timeDifference);
                    //-PIDSpeedError(motorOut1[i], i);
                else if (mode[bus][i] == CURRENT) {
                    outputArray[i] = motorOut[bus][i];
                }
                lastTime[i] = Time;
            }
            rawSend(sendIDs[0], outputArray[0], outputArray[1], outputArray[2], outputArray[3], bus);
        }
        if(motorExists[bus][4] || motorExists[bus][5] || motorExists[bus][6] || motorExists[bus][7]){
            int16_t outputArray[4] = {0, 0, 0, 0};
            int16_t outputArrayGM6020[4] = {0, 0, 0, 0};
            bool doSend[2] = {false,false};
            for (int i = 0; i < 4; i++) {
                unsigned long timeDifference = Time - lastTime[i+4];
                if(types[bus][i+4] == STANDARD){
                    if (mode[bus][i+4] == DISABLED){
                        outputArray[i] = 0;
                    }else if (mode[bus][i+4] == POSITION){
                        outputArray[i] = pidPos[bus][i+4].calculate(motorOut[bus][i+4],multiTurnPositionAngle[bus][i+4],timeDifference);
                        //-PIDPositionError(motorOut2[i], i+4);
                        doSend[0] = true;
                    }else if (mode[bus][i+4] == SPEED){
                        outputArray[i] += pidSpeed[bus][i+4].calculate(motorOut[bus][i+4],getStaticData(bus,i+4, VELOCITY),timeDifference);
                        //-PIDSpeedError(motorOut2[i], i+4);
                        doSend[0] = true;
                    }else if (mode[bus][i+4] == CURRENT) {
                        outputArray[i] = motorOut[bus][i+4];
                        doSend[0] = true;
                    }
                }else if(types[bus][i+4] == GM6020){
                    if (mode[bus][i+4] == DISABLED){
                        outputArrayGM6020[i] = 0;
                    }else if (mode[bus][i+4] == POSITION){
                        outputArrayGM6020[i] = pidPos[bus][i+4].calculate(motorOut[bus][i+4],multiTurnPositionAngle[bus][i+4],timeDifference);
                        doSend[1] = true;
                    }else if (mode[bus][i+4] == SPEED){
                        outputArrayGM6020[i] += pidSpeed[bus][i+4].calculate(motorOut[bus][i+4],getStaticData(bus,i+4, VELOCITY),timeDifference);
                        doSend[1] = true;
                    }else if (mode[bus][i+4] == CURRENT) {
                        outputArrayGM6020[i] = motorOut[bus][i+4];
                        doSend[1] = true;
                    }
                }
                lastTime[i+4] = Time;
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

    static void tickThread() {
        while (true) {
            tick();
            ThisThread::sleep_for(1);
        }
            
    }

};

#endif //motor_h