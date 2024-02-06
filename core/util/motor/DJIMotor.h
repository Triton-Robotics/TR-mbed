#ifndef TR_EMBEDDED_DJIMOTOR_H
#define TR_EMBEDDED_DJIMOTOR_H

#include "mbed.h"
#include "algorithms/PID.h"
#include "communications/CANHandler.h"
#include <cmath>
#include <string>

constexpr double M3508_GEAR_RATIO = 3591.0 / 187.0;
constexpr double M2006_GEAR_RATIO = 36.0;

constexpr int TICKS_REVOLUTION = 8192;
constexpr int TIMEOUT_MS = 400;
constexpr int CAN_HANDLER_NUMBER = 2;

constexpr int INT16_T_MAX = 32767;
constexpr int INT15_T_MAX = 16384;

static int s_sendIDs[3] = {0x200, 0x1FF, 0x2FF};           //IDs to send data
static Thread s_motorFeedbackThread(osPriorityAboveNormal);     //threading for Motor::tick()
static Thread s_motorSendThread(osPriorityNormal);              //threading for Motor::tick()

enum motorDataType {
    ANGLE,
    VELOCITY,
    TORQUE,
    TEMPERATURE,
    MULTITURNANGLE,
    POWEROUT
};


//keep in mind that in the constructor, this is only used to
//set the default pid values and gear ratio. The motorType will
//be changed to STANDARD, because that's what the code uses.

enum motorType {
    NONE = 0,
    STANDARD = 1,       //identifier for all motors that use the standard can protocol, used by the C610 and C620

    GIMBLY = 2,
    GM6020 = 2,

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
        float outCap;
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
        OFF,
        POS,
        SPD,
        POW,
        ERR
    };

    static DJIMotor* s_allMotors  [CAN_HANDLER_NUMBER][3][4];
    static bool s_motorsExist     [CAN_HANDLER_NUMBER][3][4];

    static CANHandler* s_canHandlers[CAN_HANDLER_NUMBER];
    CANHandler::CANBus canBus = CANHandler::NOBUS;

    short canID_0;                                                  // canID - 1, because canID is 1-8, arrays are 0-7
    short motorID_0;                                                // physical motorID - 1
    motorType type = NONE;                                          // type of the motor
    motorMoveMode mode = OFF;                                       // mode of the motor

    //  angle | velocity | torque | temperature
    int16_t motorData[4] = {};
    int value = 0;

    int lastMotorAngle = 0;
    int integratedAngle = 0;
    long lastIntegrationTime = -1;

    uint32_t timeOfLastFeedback = 0;
    uint32_t timeOfLastPID = 0;

    static void s_updateMultiTurnPosition();
    static void s_sendOneID(CANHandler::CANBus canBus, short sendIDindex, bool debug = false);

    void setOutput();

public:

    static int motorCount;
    static bool initializedWarning;
    std::string name = "NO_NAME";
    int16_t powerOut = 0;

    PID pidSpeed;
    PID pidPosition;

    int multiTurn = 0;
    int outputCap = INT16_T_MAX;
    bool useAbsEncoder = true;
    bool printAngle = false;

    explicit DJIMotor(bool isErroneousMotor = false);
    DJIMotor(short motorID, CANHandler::CANBus canBus, motorType type = STANDARD, const std::string& name = "NO_NAME");
    ~DJIMotor();

    static void s_setCANHandlers(CANHandler* bus_1, CANHandler* bus_2, bool threadSend = true, bool threadFeedback = true);
    static void s_getFeedback(bool debug = false);
    static void s_sendValues(bool debug = false);
    static void s_feedbackThread();
    static void s_sendThread();

    int getData(motorDataType data);
    void printAllMotorData();

    inline void setPower(int power){
        value = power;
        mode = POW;
    }

    inline void setSpeed(int speed){
        value = speed;
        mode = SPD;
    }

    inline void setPosition(int position){
        value = position;
        mode = POS;
    }

    __attribute__((unused)) inline bool isConnected() const {
        return us_ticker_read() / 1000 - timeOfLastFeedback <= TIMEOUT_MS;
    }
    static bool s_theseConnected(DJIMotor* motors[], int size, bool debug = false);
    static bool s_allConnected(bool debug = false);
    static int s_calculateDeltaPhase(int target, int current, int max = TICKS_REVOLUTION);



    inline int operator>>(motorDataType data)                                   { return getData(data); }

    inline void setPositionPID(float kP, float kI, float kD)                    { pidPosition.setPID(kP, kI, kD); }
    inline void setPositionIntegralCap(double cap)                              { pidPosition.setIntegralCap((float)cap); }
    inline void setPositionOutputCap(double cap)                                { pidPosition.setOutputCap((float)cap); }

    inline void setSpeedPID(float kP, float kI, float kD)                       { pidSpeed.setPID(kP, kI, kD); }
    inline void setSpeedIntegralCap(double cap)                                 { pidSpeed.setIntegralCap((float)cap); }
    inline void setSpeedOutputCap(double cap)                                   { pidSpeed.setOutputCap((float)cap); }

    inline int calculateSpeedPID(int desired, int current, double dt)           { return pidSpeed.calculate(desired, current, dt); }
    inline int calculatePositionPID(int desired, int current, double dt)        { return pidPosition.calculate(desired, current, dt); }
    inline int calculatePeriodicPosition(float dE, double dt)                   { return pidPosition.calculatePeriodic(dE, dt); }

};

#endif //TR_EMBEDDED_DJIMOTOR_H