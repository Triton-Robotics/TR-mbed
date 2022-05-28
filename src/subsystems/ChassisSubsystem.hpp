#include "mbed.h"
#include "../../util/motor/momsSpaghetti.hpp"
#include <cstdlib>
#ifndef chassis_subsystem_cpp
#define chassis_subsystem_cpp

enum chassisMode{TANK, MECANUM, BEYBLADE}; //what movement mode the chassis is in

enum motorLoc{LEFT_FRONT,RIGHT_FRONT,LEFT_BACK,RIGHT_BACK}; //the four potential locations of a chassis motor

/**
 * Chassis Subsystem, handles movement of chassis.
 */
class ChassisSubsystem{
    public:
        CANMotor MotorRightFront; 
        CANMotor MotorLeftFront; 
        CANMotor MotorRightBack;
        CANMotor MotorLeftBack;
       
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
        ChassisSubsystem(int m1_can_id,int m2_can_id,int m3_can_id,int m4_can_id, CANHandler::CANBus bus,motorType TYPE):
            MotorLeftFront(m1_can_id,bus,TYPE),
            MotorRightFront(m2_can_id,bus,TYPE),
            MotorLeftBack(m3_can_id,bus,TYPE),
            MotorRightBack(m4_can_id,bus,TYPE)

        {
            // MotorRightFront.setInverted(true);
            // MotorRightBack.setInverted(true);
        }
       
        /**
         * @brief Sets the individual speed of all four motors with given values.
         * 
         * @param LF speed for left front motor
         * @param RF speed for right front motor
         * @param LB speed for left back motor
         * @param RB speed for right back motor
         */ 
        void setSpeeds(int LF,int RF, int LB, int RB){
            MotorRightFront.setSpeed(RF);
            MotorLeftFront.setSpeed(LF);
            MotorRightBack.setSpeed(RB);
            MotorLeftBack.setSpeed(LB);
            
        }

        /**
         * @brief moves the chassis
         * 
         * @param y the forward movement of the chassis
         * @param x the sideways movement of the chassis
         * @param rx the sideways movement of the chassis
         */
        void move(int y, int x, int rx){
            // y=-y/660;
            // x=x/660;
            // rx = rx/660;
            //printf("%d,%d,%d\n",y,x,rx);
            int RF = y + x + rx, LF = y - x - rx, RB = y - x + rx, LB = y + x - rx;
            setSpeeds(RF,LF,RB,LB);
            //printf("%d,%d,%d,%d\n",RF,LF,RB,LB);
        }

        // /**
        //  * @brief Returns a motor given a motorLoc enum value
        //  * 
        //  * @param location, which motor to return
        //  * @return the motor
        //  */
        // CANMotor getMotor(motorLoc location){
        //     if(location == LEFT_FRONT) return MotorLeftFront;
        //     else if(location == RIGHT_FRONT) return MotorRightFront;
        //     else if(location == LEFT_BACK) return MotorLeftBack;
        //     else return MotorRightBack;
        // }

};

#endif  
