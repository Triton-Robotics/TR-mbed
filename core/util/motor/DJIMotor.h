#pragma clang diagnostic push
#pragma ide diagnostic ignored "misc-unconventional-assign-operator"
//
// Created by ankit on 1/31/23.
//

#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
#ifndef TR_EMBEDDED_DJIMOTOR_H
#define TR_EMBEDDED_DJIMOTOR_H

#define CAN_HANDLER_NUMBER 2            //Number of can handlers
#define TIMEOUT_MS 400                  //timeout for motor feedback

#include "mbed.h"
#include "algorithms/PID.h"
#include "communications/CANHandler.h"
#include "helperFunctions.hpp"
#include "algorithms/speedtocurrent.hpp"
#include <cmath>

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
    //be changed to STANDARD, because that's what the code uses.

    GIMBLY = 2, //Gimblyyyyyyyyyy
    GM6020 = 2
};


class DJIMotor {

private:
    float defaultGimblyPosSettings[5] = {10.88,1.2,18.9,8000,500};
    float defautlGimblySpeedSettings[5] = {0.13, 8.8, 0, 25000, 1000};
    float defautM3508PosSettings[5] = {.48, 0.0137, 4.2, 3000, 300};
    float defautM3508SpeedSettings[5] = {1.79, 0.27, 10.57, 15000, 500};

    enum motorMoveMode{
        OFF = 0,
        POS = 1,
        SPD = 2,
        POW = 3,
        ERR = 4
    };

    static DJIMotor* allMotors  [CAN_HANDLER_NUMBER][3][4];
    static bool motorsExist     [CAN_HANDLER_NUMBER][3][4];
    static long int lastCalled  [CAN_HANDLER_NUMBER][3][4];



    static CANHandler* canHandlers[CAN_HANDLER_NUMBER];
    short motorNumber;                                          // canID - 1, because canID is 1-8, arrays are 0-7
    int gearRatio = 1;                                          //the gear ratio of the motor to encoder

    CANHandler::CANBus canBus = CANHandler::NOBUS;              //the CANBus this motor is on
    motorType type = NONE;                                      //mode of the motor
    motorMoveMode mode = OFF;                                   //mode of the motor

    unsigned long timeOfLastFeedback = 0;
    unsigned long timeOfLastPID = 0;

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

    bool conflict{};                                            //check for a conflict when running motors
    unsigned long lastTime = 0;
    int outCap = 16000;

    bool useAbsEncoder = false;
    bool justPosError = false;
    static bool sendDebug;
    static bool feedbackDebug;

    // user methods
    explicit DJIMotor(bool isErroneousMotor = false);
    DJIMotor(short canID, CANHandler::CANBus bus, motorType mType = STANDARD);
    ~DJIMotor();

    // static void printChunk(CANHandler::CANBus bus, short sendID, motorDataType data = POWEROUT);
    static void setCANHandlers(CANHandler* bus_1, CANHandler* bus_2, bool threadSend = true, bool threadFeedback = true);
    static void updateMultiTurnPosition();
    static void sendOneID(CANHandler::CANBus bus, short sendIDindex, bool debug = false);
    static void getFeedback();

    __attribute__((unused)) static bool checkConnection(bool debug = false);

    static void tickThread();
    static void feedbackThread();
    static void sendThread();
    static void sendValues();
    static void tick();

    //int getValue();

    __attribute__((unused)) int getPowerOut() const;
    int getData(motorDataType data);

    void setValue(int val);
    void setOutput();
    void setPower(int power);
    void setSpeed(int speed);
    void setPosition(int position);

    __attribute__((unused)) void zeroPos();

    void printAllMotorData();

    void operator=(int value);
    int operator>>(motorDataType data);

    inline void setPositionPID(float kP, float kI, float kD)    { pidPosition.setPID(kP, kI, kD); }
    inline void setPositionIntegralCap(double cap)              { pidPosition.setIntegralCap((float)cap); }
    inline void setPositionOutputCap(double cap)                { pidPosition.setOutputCap((float)cap); }

    inline void setSpeedPID(float kP, float kI, float kD)       { pidSpeed.setPID(kP, kI, kD); }
    inline void setSpeedIntegralCap(double cap)                 { pidSpeed.setIntegralCap((float)cap); }
    inline void setSpeedOutputCap(double cap)                   { pidSpeed.setOutputCap((float)cap); }

};


#endif //TR_EMBEDDED_DJIMOTOR_H

#pragma clang diagnostic pop
#pragma clang diagnostic pop