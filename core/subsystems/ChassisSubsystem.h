#ifndef TR_EMBEDDED_CHASSIS_SUBSYSTEM_H
#define TR_EMBEDDED_CHASSIS_SUBSYSTEM_H

#include "mbed.h"
#include "../util/peripherals/imu/BNO055.h"

class ChassisSusbsystem
{
public:
    ChassisSusbsystem(short lfId, short rfId, short lbId, short rbId, BNO055 &imu);
};

#endif // TR_EMBEDDED_CHASSIS_SUBSYSTEM_H
