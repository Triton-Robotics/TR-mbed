#include "mbed.h"
#include "../../util/motor/CANMotor.hpp"
#include <cstdlib>
#include <math.h>
///////////////////////////////////////////////////////////////////////
//                  ___      __  __          __             __  __                      _________ __  __              __  ___          _      ____               __       __  __  _ 
//  __ _____  __ __/ _ )___ / /_/ /____ ____/ /  ___  ___ __\ \/ /__  __ _________ ___ / / _/  _/_\ \/ /__  __ ______/  |/  /__  __ _ ( )___ / __/__  ___ ____ _/ /  ___ / /_/ /_(_)
// / // / _ \/ // / _  / -_) __/ __/ -_) __/ /__/ _ \(_-</ -_)  / _ \/ // / __(_-</ -_) / _// // _ \  / _ \/ // / __/ /|_/ / _ \/  ' \|/(_-<_\ \/ _ \/ _ `/ _ `/ _ \/ -_) __/ __/ / 
// \_, /\___/\_,_/____/\__/\__/\__/\__/_/ /____/\___/___/\__//_/\___/\_,_/_/ /___/\__/_/_//___/_//_/_/\___/\_,_/_/ /_/  /_/\___/_/_/_/ /___/___/ .__/\_,_/\_, /_//_/\__/\__/\__/_/  
///___/                                                                                                                                       /_/        /___/                      
//
///////////////////////////////////////////////////////////////////////
#ifndef new_chassis_subsystem_cpp
#define new_chassis_subsystem_cpp

enum chassisMode{TANK, MECANUM, BEYBLADE}; //what movement mode the chassis is in

enum motorLoc{LEFT_FRONT,RIGHT_FRONT,LEFT_BACK,RIGHT_BACK}; //the four potential locations of a chassis motor

/**
 * Chassis Subsystem, handles movement of chassis.
 */
class NewChassisSubsystem{
    public:
        CANMotor MotorRightFront; 
        CANMotor MotorLeftFront; 
        CANMotor MotorRightBack;
        CANMotor MotorLeftBack;

        int theta;
       
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
        NewChassisSubsystem(int m1_can_id,int m2_can_id,int m3_can_id,int m4_can_id, NewCANHandler::CANBus bus,motorType TYPE):
            MotorLeftFront(m1_can_id,bus,TYPE),
            MotorRightFront(m2_can_id,bus,TYPE),
            MotorLeftBack(m3_can_id,bus,TYPE),
            MotorRightBack(m4_can_id,bus,TYPE)
            
        {
            theta = 0;
            //MotorRightFront.setInverted(true);
            //MotorRightBack.setInverted(true);
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

            // MotorRightFront.setPower(RF);
            // MotorLeftFront.setPower(LF);
            // MotorRightBack.setPower(RB);
            // MotorLeftBack.setPower(LB);
            
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
            int LF = y + x + rx, RF = y - x - rx, LB = y - x + rx, RB = y + x - rx;
            setSpeeds(LF,-RF,LB,-RB);
            //printf("%d,%d,%d,%d\n",RF,LF,RB,LB);
        }

        /**
         * @brief moves the chassis
         * 
         * @param y the forward movement of the chassis
         * @param x the sideways movement of the chassis
         * @param rx the sideways movement of the chassis
         */
        void movePow(int y, int x, int rx){
            // y=-y/660;
            // x=x/660;
            // rx = rx/660;
            //printf("%d,%d,%d\n",y,x,rx);
            int LF = y + x + rx, RF = y - x - rx, LB = y - x + rx, RB = y + x - rx;
            //setSpeeds(LF,-RF,LB,-RB);
            MotorLeftFront.setPower(LF);
            MotorRightFront.setPower(-RF);
            MotorLeftBack.setPower(LB);
            MotorRightBack.setPower(-RB);
            //printf("MOVEPOW- %d,%d,%d,%d\n",RF,LF,RB,LB);
        }

        void beyblade(int y, int x, int dTheta, int theta){
            int angle = theta - atan(y/x);
            
        }

        /**
         * @brief Returns a motor given a motorLoc enum value
         * 
         * @param location, which motor to return
         * @return the motor
         */
        CANMotor getMotor(motorLoc location){
            if(location == LEFT_FRONT) return MotorLeftFront;
            else if(location == RIGHT_FRONT) return MotorRightFront;
            else if(location == LEFT_BACK) return MotorLeftBack;
            else return MotorRightBack;
        }

};

#endif  
