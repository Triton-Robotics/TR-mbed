#include "CANMotor.hpp"
#ifndef canmotor_statics
#define canmotor_statics
CANMotor* CANMotor::allMotors[2][3][4];
NewCANHandler* CANMotor::canHandlers[2];
bool CANMotor::motorsExist[2][3][4];
bool CANMotor::sendDebug = false;
bool CANMotor::feedbackDebug = false;
#endif