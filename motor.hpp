#include "mbed.h"
#include <cstdint>
#ifndef motor_hpp
#define motor_hpp
#include "CANMsg.h"

//////////////////////////////////////////////
//VERY IMPORTANT TO SET FREQUENCY HERE AND NOW
//////////////////////////////////////////////

CAN             can1(PA_11,PA_12,1000000);
CAN             can2(PB_12,PB_13,1000000);

enum motorMode {DISABLED, POSITION, SPEED, CURRENT};

enum motorType {
    NONE = 0,
    STANDARD = 1,
    C620 = 1,
    M3508 = 1,
    GIMBLY = 2,
    GM6020 = 2
};

static int feedbackIDs[12] = {0x201,0x202,0x203,0x204,0x205,0x206,0x207,0x208,0x209,0x20a,0x20b,0x20c}; //IDs to recieve data back from the motors

static int sendIDs[3] = {0x200,0x1FF,0x2FF}; //IDs to send data

static int16_t feedback[12][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}}; //Array holding the feedback values of the individual motors

static int totalMotors; //total number of motors in play

static bool motorExists[8] = {0,0,0,0,0,0,0,0};

static int motorOut1[4] = {0,0,0,0}; //First four motors in can, controlled through ID 0x200

static int motorOut2[4] = {0,0,0,0}; //Second four motors in can, controlled through ID 0x1FF

static bool motorDebug = 0;

static motorType types[] = {NONE,NONE,NONE,NONE,NONE,NONE,NONE,NONE};

static CANMsg txMsg(0x000,CANStandard); //Message object reused to send messages to motors
static CANMsg rxMsg; //Message object reused to recieve messages from motors

static motorMode mode[8] = {DISABLED, DISABLED, DISABLED, DISABLED, DISABLED, DISABLED, DISABLED, DISABLED};

static double PIDValuesPosition[8][3] = {{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0}};
static double PIDValuesSpeed[8][3] = {{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0}};

static int multiTurnPositionAngle[8] = {0,0,0,0,0,0,0,0};

class CanMotor {

    private:
    /**
     * @brief Set the desired value of this motor
     * 
     * @param value
     * @return int value
     */
    int setDesiredValue(int value){
        if(motorNumber < 4){
            motorOut1[motorNumber] = value;
            return motorOut1[motorNumber];
        }else if(motorNumber >= 4){
            motorOut2[motorNumber-4] = value;
            return motorOut2[motorNumber-4];
        }
        return -1;
    }

    public:

    int motorNumber;

    int gearRatio = 19;

    bool isInverted = false;

    /**
     * @brief Construct a new Motor object
     * 
     * @param canNum is a number from 1-8 signifying which CAN id is attached, blinking LED on motor controller will show this
     */
    CanMotor(int canNum)
    {
        gearRatio = 19;
        motorNumber = canNum - 1; //Changes range from 1-8 to 0-7
        totalMotors++;
        motorExists[motorNumber] = 1;
        types[motorNumber] = STANDARD;
        //TODO Throw error when motorNumber isnt within the range [0,7]
    }

    CanMotor(int canNum, int ratio = 19, int inverted = false)
    {
        isInverted = inverted;
        gearRatio = ratio;
        motorNumber = canNum - 1; //Changes range from 1-8 to 0-7
        totalMotors++;
        motorExists[motorNumber] = 1;
        types[motorNumber] = STANDARD;
        //TODO Throw error when motorNumber isnt within the range [0,7]
    }

    CanMotor(int canNum, motorType type = STANDARD, int ratio = 19, int inverted = false)
    {
        isInverted = inverted;
        if(type == GM6020){
            gearRatio = 1; //TODO FIND THE ACTUAL GEAR RATIO
            if(canNum <= 4){
                printf("ERROR. IT IS HIGHLY DISCOURAGED OF YOU TO USE CAN BUSSES 1-4 FOR THE GM6020s. YOU WILL HAVE ERRORS. DO NOT\n");
            }
        }else{
            gearRatio = ratio;
        }
        motorNumber = canNum - 1; //Changes range from 1-8 to 0-7
        totalMotors++;
        motorExists[motorNumber] = 1;
        types[motorNumber] = type;
        //TODO Throw error when motorNumber isnt within the range [0,7]
    }

    ~CanMotor(){ //DESTRUCTOR
        totalMotors --;
        motorExists[motorNumber] = 0;
        types[motorNumber] = NONE;
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
        if(motorNumber < 4){
            return motorOut1[motorNumber];
        }else if(motorNumber >= 4){
            return motorOut2[motorNumber-4];
        }
        return -1;
    }

    
    /**
     * @brief Returns angle of motor
     * 
     * @return int 
     */
    int getAngle(){
        return feedback[motorNumber][0];
    }

    int getMultiTurnAngle(){
        return multiTurnPositionAngle[motorNumber];
    }

    static int staticAngle(int motorID){
        return feedback[motorID][0];
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
        return feedback[motorNumber][1];
    }

    static int staticSpeed(int motorID) {
        return feedback[motorID][1];
    }

    /**
     * @brief Returns torque of motor
     * 
     * @return int 
     */
    int getTorque(){
        return feedback[motorNumber][2];
    }

    /**
     * @brief Returns temperature of motor
     * 
     * @return int 
     */
    int getTemperature(){
        return feedback[motorNumber][3];
    }

    void setPositionPID(double Kp, double Ki, double Kd){
        PIDValuesPosition[motorNumber][0] = Kp;
        PIDValuesPosition[motorNumber][1] = Ki;
        PIDValuesPosition[motorNumber][2] = Kd;
    }

    void setSpeedPID(double Kp, double Ki, double Kd){
        PIDValuesSpeed[motorNumber][0] = Kp;
        PIDValuesSpeed[motorNumber][1] = Ki;
        PIDValuesSpeed[motorNumber][2] = Kd;
    }
    
    /**
     * @brief turns an int to four bytes
     * 
     * @param n an int
     * @return an array of four unsigned bytes in the form of int8_ts
     */
    uint8_t* intToBytes(int n){
        uint8_t out[4] = {(uint8_t)(n >> 24),(uint8_t)(n >> 16),(uint8_t)(n >> 8),(uint8_t)n};
        return out;
    }

    /**
     * @brief turns four bytes into an int
     * 
     * @param bytes array of four unsigned bytes
     * @return an int
     */
    int bytesToInt(uint8_t bytes[4]){
        return (int)((bytes[0] << 24) + (bytes[1] << 16) + (bytes[2] << 8) + (bytes[3]));
    } 

    /**
     * @brief turns an int to two bytes
     * 
     * @param n an int
     * @return an array of two unsigned bytes (int8_ts)
     */
    uint8_t* int16ToBytes(int n){
        uint8_t out[2] = {(uint8_t)(n >> 8),(uint8_t)n};
        return out;
    }

    /**
     * @brief turns two bytes into an int
     * 
     * @param bytes array of two unsigned bytes (int8_ts)
     * @return an int
     */
    int bytesToInt16(uint8_t bytes[2]){
        return (int)((bytes[2] << 8) + (bytes[3]));
    } 

    static int PIDPositionError(int desiredAngle, int motorID) {
        int error = multiTurnPositionAngle[motorID] - desiredAngle;
        static unsigned long lastTime[8] = {0};
        unsigned long Time = us_ticker_read() / 1000;
        unsigned long timeDifference = Time - lastTime[motorID];

        static int lastError[8] = {0};
        static int sumerror[8] = {0};
        sumerror[motorID] += error;
        double kP = PIDValuesPosition[motorID][0];
        double kI = PIDValuesPosition[motorID][1];
        double kD = PIDValuesPosition[motorID][2];


        if (abs(error) < 2500)
            sumerror[motorID] = 0;

        int maxsumerror = 2000;

        if (sumerror[motorID] > maxsumerror)
            sumerror[motorID] = maxsumerror;
        else if (sumerror[motorID] < -maxsumerror)
            sumerror[motorID] = -maxsumerror;   

        
    
        int PIDCalc = kP * error + kI * sumerror[motorID] + kD * ((double)(error - lastError[motorID])/timeDifference);
        
        //printf("PIDCALC: %d\n",PIDCalc);
        
        int maxcurrent = 20000;
        if (PIDCalc > maxcurrent)
            PIDCalc = maxcurrent;
        else if (PIDCalc < -maxcurrent)
            PIDCalc = -maxcurrent;

        lastTime[motorID] = Time;
        error = lastError[motorID];

        return PIDCalc;
    }

    static int PIDSpeedError(int desiredSpeed, int motorID) {
        int error = staticSpeed(motorID) - desiredSpeed;
        static unsigned long lastTime[8] = {0};
        unsigned long Time = us_ticker_read() / 1000;
        unsigned long timeDifference = Time - lastTime[motorID];

        static int lastError[8] = {0};
        static int sumerror[8] = {0};
        sumerror[motorID] += error;
        double kP = PIDValuesSpeed[motorID][0];
        double kI = PIDValuesSpeed[motorID][1];
        double kD = PIDValuesSpeed[motorID][2];


        if (abs(error) < 2500)
            sumerror[motorID] = 0;

        int maxsumerror = 2000;

        if (sumerror[motorID] > maxsumerror)
            sumerror[motorID] = maxsumerror;
        else if (sumerror[motorID] < -maxsumerror)
            sumerror[motorID] = -maxsumerror;   

        
    
        int PIDCalc = kP * error + kI * sumerror[motorID] + kD * ((double)(error - lastError[motorID])/timeDifference);
        
        printf("PIDCALC: %d\n",PIDCalc);
        
        int maxcurrent = 5000;
        if (PIDCalc > maxcurrent)
            PIDCalc = maxcurrent;
        else if (PIDCalc < -maxcurrent)
            PIDCalc = -maxcurrent;

        lastTime[motorID] = Time;
        error = lastError[motorID];

        return PIDCalc;
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
    static void getFeedback(){
        if (can1.read(rxMsg)) {
            if(motorDebug){
                printf("-------------------------------------\r\n");
                printf("CAN message received\r\n");
                printMsg(rxMsg);
            }
            int motorID = rxMsg.id-0x201;
            if(motorID >= 8){
                motorID -= 4;
            }
            uint8_t recievedBytes[8] = {0,0,0,0,0,0,0,0};
            for(int i = 0;  i < 8; i ++){
                rxMsg >> recievedBytes[i]; //2 bytes per motor
            }
            if(motorDebug){
                printf("ArrayData    =");
                for (int i = 0; i < sizeof(recievedBytes); i++)
                    printf(" 0x%.2X", recievedBytes[i]);
                printf("\r\n");
            }
   
            feedback[motorID][0] = 0 | (recievedBytes[0]<<8) | recievedBytes[1];
            feedback[motorID][1] = 0 | (recievedBytes[2]<<8) | recievedBytes[3];
            feedback[motorID][2] = 0 | (recievedBytes[4]<<8) | recievedBytes[5];
            feedback[motorID][3] = ((int16_t) recievedBytes[6]);

            //printf("Motor 0x%x:\tAngle (0,8191):%d\tSpeed  ( RPM ):%d\tTorque ( CUR ):%d\tTemperature(C):%d \n",rxMsg.id,feedback[motorID][0],feedback[motorID][1],feedback[motorID][2],feedback[motorID][3]);
        }
        //CAN Recieving from feedback IDs
    }

    static void multiTurnPositionControl() {
        int Threshold = 3000;

        static int lastMotorAngle[8] = {0,0,0,0,0,0,0,0};

        for (int i = 0; i < 7; i++) {
            if (abs(staticSpeed(i)) < 100) {
                if ( staticAngle(i) > (8191 - Threshold) && lastMotorAngle[i] < Threshold)
                    multiTurnPositionAngle[i] += -(staticAngle(i) - 8191) - lastMotorAngle[i];

                else if (staticAngle(i) < Threshold && lastMotorAngle[i] > (8191 - Threshold))
                    multiTurnPositionAngle[i] -= -(staticAngle(i) - 8191) - lastMotorAngle[i];
                else 
                    multiTurnPositionAngle[i] += staticAngle(i) - lastMotorAngle[i];
            }
            else {
                int delta = staticAngle(i) - lastMotorAngle[i]; // 0 to 199 POS// 8000 to 128 NEG
                if(staticSpeed(i) < 0 && delta > 0){ //neg skip
                    multiTurnPositionAngle[i] += (delta - 8191);
                }else if(staticSpeed(i) > 0 && delta < 0){ //pos skip
                    multiTurnPositionAngle[i] += (delta + 8191);
                }else { //pos no skip or neg no skip same case
                    multiTurnPositionAngle[i] += delta;
                }
            }
            lastMotorAngle[i] = staticAngle(i);

        }
      
    }

    /**
     * @brief Returns specified data of specified motor
     * canBuss is a field between 1 and 8, specifing the can bus of the motor
     * dataNumber is the element of data you want
     * Angle: 0
     * Speed: 1
     * Torque: 2
     * Temperature: 3
     * 
     * @return int 
     */
    static int getData(int canBus, int dataNumber){
        return feedback[canBus -1][dataNumber];
    }

    /**
     * @brief send all motor values after setting them in setDesiredValues
     * 
     */
    static void sendValues(){
        //CAN Sending to the two sending IDs
        if(motorExists[0] || motorExists[1] || motorExists[2] || motorExists[3]){
            int16_t outputArray[4] = {0, 0, 0, 0};
            for (int i = 0; i < 4; i++) {
                if (mode[i] == DISABLED)
                    outputArray[i] = 0;
                else if (mode[i] == POSITION)
                    outputArray[i] = -PIDPositionError(motorOut1[i], i);
                else if (mode[i] == SPEED)
                    outputArray[i] += -PIDSpeedError(motorOut1[i], i);
                else if (mode[i] == CURRENT) {
                    outputArray[i] = motorOut1[i];
                }
            }

            rawSend(sendIDs[0], outputArray[0], outputArray[1], outputArray[2], outputArray[3]);
        }
        if(motorExists[4] || motorExists[5] || motorExists[6] || motorExists[7]){
            int16_t outputArray[4] = {0, 0, 0, 0};
            int16_t outputArrayGM6020[4] = {0, 0, 0, 0};
            bool doSend[2] = {false,false};
            for (int i = 0; i < 4; i++) {
                if(types[i+4] == STANDARD){
                    if (mode[i+4] == DISABLED){
                        outputArray[i] = 0;
                    }else if (mode[i+4] == POSITION){
                        outputArray[i] = -PIDPositionError(motorOut2[i], i+4);
                        doSend[0] = true;
                    }else if (mode[i+4] == SPEED){
                        outputArray[i] += -PIDSpeedError(motorOut2[i], i+4);
                        doSend[0] = true;
                    }else if (mode[i+4] == CURRENT) {
                        outputArray[i] = motorOut2[i];
                        doSend[0] = true;
                    }
                }else if(types[i+4] == GM6020){
                    if (mode[i+4] == DISABLED){
                        outputArrayGM6020[i] = 0;
                    }else if (mode[i+4] == SPEED){
                        //printf("Poes:%d\n",motorOut2[i]);
                        outputArrayGM6020[i] += -PIDSpeedError(motorOut2[i], i+4);
                        printf("\t\t\t\tCurrent given:%d\n",outputArrayGM6020[i]);
                        doSend[1] = true;
                    }else if (mode[i+4] == POSITION){
                        //printf("Poes:%d\n",motorOut2[i]);
                        outputArrayGM6020[i] = -PIDPositionError(motorOut2[i], i+4);
                        doSend[1] = true;
                    }else if (mode[i+4] == CURRENT) {
                        outputArrayGM6020[i] = motorOut2[i];
                        doSend[1] = true;
                    }
                }
            }
            if(doSend[0])
                rawSend(sendIDs[1], outputArray[0], outputArray[1], outputArray[2], outputArray[3]);
            if(doSend[1])
                rawSend(sendIDs[2], outputArrayGM6020[0], outputArrayGM6020[1], outputArrayGM6020[2], outputArrayGM6020[3]);
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
    static void rawSend(int id, int data1, int data2, int data3, int data4){
        txMsg.clear(); // clear Tx message storage
        //txMsg.format = CANStandard;
        //txMsg.type = CANData; 
        txMsg.id = id; 

        int motorSending[4] = {data1,data2,data3,data4};

        int8_t sentBytes1[8] = {0,0,0,0,0,0,0,0};
        for(int i = 0; i < 4;  i ++){
            sentBytes1[(2*i)+1] = motorSending[i] & (0xFF);
            sentBytes1[2*i] = (motorSending[i] >> 8) & (0xFF);
        }
        for(int i = 0;  i < 8; i ++){
            txMsg << sentBytes1[i]; //2 bytes per motor
        }
        //printf(".");
        bool isWrite = 1;
        isWrite = can1.write(txMsg);
        //printf("Bool:%d\n",isWrite);
        //isWrite = 1;
        //printf("Erpr:%d\n", can1.tderror());
        if (isWrite) {
            // transmit message
            if(motorDebug){
                printf("-------------------------------------\r\n");
                printf("-------------------------------------\r\n");
                printf("CAN message sent\r\n");
                printMsg(txMsg);
            }
        }
        else if(isWrite == 0){
            //printf("Transmission error\n");
            //break; //TODO AT SOME POINT REMOVE THIs WHEN A TRANSMISSION ERROR ISNT CATASTROPHIC
        }else{
            printf("???????????????????\n");
        }
    }

    /**
     * @brief Function that should be called by the user every tick, runs necessary elements for CAN motors to work.
     * 
     */
    static void tick(){
        getFeedback();
        multiTurnPositionControl();
        sendValues();
    }

    /**
     * @brief equivalent to tick
     * 
     */
    static void update(){
        tick();
    }


};

#endif //motor_h