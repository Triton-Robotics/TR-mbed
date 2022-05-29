#include "mbed.h"
#include "../../util/motor/momsSpaghetti.hpp"
#include "../../util/motor/pwmmotor.cpp"
#include <cstdlib>

//INCOMPLETE

#ifndef turret_subsystem_hpp
#define turret_subsystem_hpp

/**
 * Chassis Subsystem, handles movement of chassis.
 */
class TurretSubsystem{
    public:
        CANMotor turretX; 
        CANMotor turretY; 
        CANMotor indexer;
        PWMMotor flywheelL;
        PWMMotor flywheelR;
       
        /**
         * @brief Constructor for a chassis
         *
         * @param m1_pin pin for left front motor
         * @param m2_pin pin for right front motor
         * @param m3_pin pin for left back motor
         * @param m4_pin pin for right back motor
         * 
         * @param bus can bus
         * @param TYPE the type of motor
         */ 
        TurretSubsystem(int turretXID,int turretYID,int indexerID,int m4_can_id, CANHandler::CANBus bus,motorType TYPE):
            turretX(turretXID,bus,TYPE),
            turretY(turretYID,bus,TYPE),
            flywheelL(PA_5),
            flywheelR(PA_6) //INCOMPLETE

        {
            // MotorRightFront.setInverted(true);
            // MotorRightBack.setInverted(true);
        }
};

#endif  
