#include "momsSpaghetti.hpp"

CANMotor* CANMotor::allMotors[2][3][4];
NewCANHandler* CANMotor::canHandlers[2];
bool CANMotor::motorsExist[2][3][4];