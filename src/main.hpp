#include "mbed.h"
#include "../util/motor/motor.hpp"
#include "../util/communications/include/DJIRemote.hpp"
#include "../util/communications/canHandler.hpp"
#include "../util/algorithms/pid.hpp"
#include "subsystems/ChassisSubsystem.hpp"
#include "subsystems/TurretSubsystem.hpp"
#include "../util/communications/djiremoteuart.hpp"
#include "../util/helperFunctions.hpp"

enum robotType{TEST_BENCH, INFANTRY, HERO, ENGINEER, SENTRY};
