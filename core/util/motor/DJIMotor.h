#ifndef TR_EMBEDDED_DJIMOTOR_H
#define TR_EMBEDDED_DJIMOTOR_H

#pragma once
#include "mbed.h"
#include "util/algorithms/PID.h"
#include "util/communications/CANHandler.h"
#include <cmath>
#include <string>

constexpr double M3508_GEAR_RATIO = 3591.0 / 187.0;
constexpr double M2006_GEAR_RATIO = 36.0;

constexpr int TICKS_REVOLUTION = 8192;
constexpr int TIMEOUT_MS = 400;
constexpr int CAN_HANDLER_NUMBER = 2;

constexpr int INT16_T_MAX = 32767;
constexpr int INT15_T_MAX = 16383;

static int s_sendIDs[3] = {0x200, 0x1FF, 0x2FF};                //IDs to send data

enum motorDataType {
    ANGLE,
    VELOCITY,
    TORQUE,
    TEMPERATURE,
    MULTITURNANGLE,
    POWEROUT,
    VALUE
};


//keep in mind that in the constructor, this is only used to
//set the default pid values and gear ratio. The motorType will
//be changed to STANDARD, because that's what the code uses.

enum motorType {
    NONE = 0,
    STANDARD = 1,       //identifier for all motors that use the standard can protocol, used by the C610 and C620
    GM6020 = 2,
    M3508 = 3,
    M2006 = 4,
    M3508_FLYWHEEL = 5,
};

class DJIMotor {
public:
    enum motorMoveMode{
        OFF,
        POS,
        SPD,
        POW,
        ERR
    };

    struct config
    {
        short motorID;
        CANHandler::CANBus canBus;
        motorType type = STANDARD;
        std::string name = "NO_NAME";
        PID::config vel_cfg;
        PID::config pos_cfg;
    };


    static int motorCount;
    static bool initializedWarning;
    std::string name = "NO_NAME";
    int16_t powerOut = 0;

    PID pidSpeed;
    PID pidPosition;

    int multiTurn = 0;
    int outputCap = INT16_T_MAX;
    bool useAbsEncoder = false; // false for everything except gm6020s
    bool printAngle = false;

    motorType type = NONE;


    explicit DJIMotor(bool isErroneousMotor = false);
    DJIMotor(short motorID, CANHandler::CANBus canBus, motorType type = STANDARD, const std::string& name = "NO_NAME");
    DJIMotor(config cfg);
    ~DJIMotor();

    static int calculateDeltaPhase(int target, int current, int max = TICKS_REVOLUTION);
    static float calculateDeltaPhase(float target, float current, float max);

    static void getCan1Feedback(const CANMsg * msg);
    static void getCan2Feedback(const CANMsg * msg);
    static void sendValues(bool debug = false);

    int getData(motorDataType data);

    inline void setMotorOutput(int val, motorMoveMode mod)
    {
        value = val;
        mode = mod;
    }

    inline void setPower(int power)
    {
        value = power;
        mode = POW;
        powerOut = power;
    }

    inline void setSpeed(int speed){
        value = speed;
        mode = SPD;
    }

    inline void setPosition(int position){
        value = position;
        mode = POS;
    }

    inline motorMoveMode getMode(){
        return mode;
    }

    __attribute__((unused)) inline bool isConnected() const {
        return us_ticker_read() / 1000 - timeOfLastFeedback <= TIMEOUT_MS;
    }

    inline int operator>>(motorDataType data)                                                       { return getData(data); }

    inline void setPositionPID(float kP, float kI, float kD)                                        { pidPosition.setPID(kP, kI, kD); }
    inline void setPositionIntegralCap(double cap)                                                  { pidPosition.setIntegralCap((float)cap); }
    inline void setPositionOutputCap(double cap)                                                    { pidPosition.setOutputCap((float)cap); }

    inline void setSpeedPID(float kP, float kI, float kD)                                           { pidSpeed.setPID(kP, kI, kD); }
    inline void setSpeedIntegralCap(double cap)                                                     { pidSpeed.setIntegralCap((float)cap); }
    inline void setSpeedDerivativeCap(double cap)                                                   { pidSpeed.setDerivativeCap((float)cap); }
    inline void setSpeedOutputCap(double cap)                                                       { pidSpeed.setOutputCap((float)cap); }

    inline int calculateSpeedPID(int desired, int current, double dt)                               { return pidSpeed.calculate(desired, current, dt); }
    inline int calculatePositionPID(int desired, int current, double dt, int chassis_rpm = 0)       { return pidSpeed.calculate(pidPosition.calculate(desired, current, dt) - chassis_rpm, getData(VELOCITY), dt); }
    inline int calculatePeriodicPosition(float dE, double dt, int chassis_rpm = 0)                  { return pidSpeed.calculate(pidPosition.calculatePeriodic(dE, dt)       - chassis_rpm, getData(VELOCITY), dt); }

private:
    static DJIMotor* s_allMotors  [CAN_HANDLER_NUMBER][3][4];
    static bool s_motorsExist     [CAN_HANDLER_NUMBER][3][4];

    static CANHandler* s_canHandlers[CAN_HANDLER_NUMBER];
    CANHandler::CANBus canBus = CANHandler::NOBUS;

    short canID_0;              // canID - 1, because canID is 1-8, arrays are 0-7
    short motorID_0;            // physical motorID - 1
    motorMoveMode mode = OFF;   // mode of the motor

    //  angle | velocity | torque | temperature
    int16_t motorData[4] = {};
    int value = 0;

    int lastMotorAngle = 0;
    int integratedAngle = 0;
    long lastIntegrationTime = -1;

    uint32_t timeOfLastFeedback = 0;
    uint32_t timeOfLastPID = 0;

    static void updateOneMultiTurn(int canBus, int can_line, int motor_id);
    static void sendOneID(CANHandler::CANBus canBus, short sendIDindex, bool debug = false);
    void setOutput();
};

#endif //TR_EMBEDDED_DJIMOTOR_H