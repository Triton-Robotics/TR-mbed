#pragma clang diagnostic push
#pragma ide diagnostic ignored "misc-unconventional-assign-operator"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
#ifndef TR_EMBEDDED_DJIMOTOR_H
#define TR_EMBEDDED_DJIMOTOR_H

#include "mbed.h"
#include "algorithms/PID.h"
#include "communications/CANHandler.h"
#include <cmath>
#include <string>

#define CAN_HANDLER_NUMBER 2            //Number of can handlers
#define TIMEOUT_MS 400                  //timeout for motor feedback

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
    POWEROUT = 5,
};

//keep in mind that in the constructor, this is only used to
//set the default pid values and gear ratio. The motorType will
//be changed to STANDARD, because that's what the code uses.

enum motorType {
    NONE = 0,
    STANDARD = 1,       //identifier for all motors that use the standard can protocol, used by the C610 and C620

    GIMBLY = 2,
    GM6020 = 2,

    C620 = 3,
    M3508 = 3,

    C610 = 4,
    M2006 = 4,

    M3508_FLYWHEEL = 5,
};

class DJIMotor {

private:

    struct PidSettings{
        float p;
        float i;
        float d;
        float outputCap;
        float integralCap;
    };

    struct MotorSettings{
        PidSettings pos;
        PidSettings speed;
    };

    // Default motor PID settings

    MotorSettings gimbly = {
            {10.88,1.2,18.9,8000,500},
            {0.13, 8.8, 0, 25000, 1000}
    };

    MotorSettings m3508 = {
            {.48, 0.0137, 4.2, 3000, 300},
            {1.79, 0.27, 10.57, 15000, 500}
    };

    MotorSettings m3508Flywheel = {
            {.48, 0.0137, 4.2, 3000, 300},
            {2.013, 0.319, 20.084, 15000, 500}
    };

    enum motorMoveMode{
        OFF = 0,
        POS = 1,
        SPD = 2,
        POW = 3,
        ERR = 4
    };

    static DJIMotor* s_allMotors  [CAN_HANDLER_NUMBER][3][4];
    static bool s_motorsExist     [CAN_HANDLER_NUMBER][3][4];

    static CANHandler* s_canHandlers[CAN_HANDLER_NUMBER];
    CANHandler::CANBus canBus = CANHandler::NOBUS;                  //the CANBus this motor is on

    short canID_0;                                                  // canID - 1, because canID is 1-8, arrays are 0-7
    short motorID_0;
    int gearRatio = 1;                                              //the gear ratio of the motor to encoder
    motorType type = NONE;                                          //mode of the motor
    motorMoveMode mode = OFF;                                       //mode of the motor

    unsigned long timeSinceLastFeedback = 0;
    unsigned long timeOfLastFeedback = 0;
    unsigned long timeOfLastPID = 0;

public:

    std::string name = "NO_NAME";

    int maxSpeed = 8723;
    int bounds[2] = {0,0};

    //  angle | velocity | torque | temperature
    int16_t motorData[4] = {0,0,0,0};
    int multiTurn = 0;
    int lastMotorAngle = 0;

    PID pidSpeed;
    PID pidPosition;

    int value = 0;
    int16_t powerOut = 0;

    bool conflict{};                                            //check for a conflict when running motors
    unsigned long lastTime = 0;
    int outCap = 16000;

    bool useAbsEncoder = false;
    bool justPosError = false;
    bool useKalmanForPID = false;
    bool useIntegrationForPID = false;
    bool printAngle = false;
    int integratedAngle = 0;
    long lastIntegrationTime = -1;
    static bool feedbackDebug;

    explicit DJIMotor(bool isErroneousMotor = false);
    //DJIMotor(short motorID, CANHandler::CANBus canBus, motorType type, std::string name);
    DJIMotor(short motorID, CANHandler::CANBus canBus, motorType type = STANDARD, const std::string& name = "NO_NAME");
    ~DJIMotor();

    // static void printChunk(CANHandler::CANBus bus, short sendID, motorDataType data = POWEROUT);
    static void setCANHandlers(CANHandler* bus_1, CANHandler* bus_2, bool threadSend = true, bool threadFeedback = true);
    static void updateMultiTurnPosition();
    static void sendOneID(CANHandler::CANBus canBus, short sendIDindex, bool debug = false);
    static void getFeedback();

    bool isConnected() const;
    __attribute__((unused)) static bool s_allMotorsConnected(bool debug);
    static bool s_isMotorConnected(CANHandler::CANBus bus, motorType type, short canID);

    static void feedbackThread();
    static void sendThread();
    static void sendValues(bool debug = false);
    static void tick();

    //int getValue();

    int getData(motorDataType data);

    void setValue(int val);
    void setOutput();
    void setPower(int power);
    void setSpeed(int speed);
    void setPosition(int position);

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