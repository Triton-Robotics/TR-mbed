#include "mbed.h"
#include "../../util/motor/CANMotor.hpp"
#include "../../util/motor/pwmmotor.cpp"
#include <cstdlib>
#include "../../util/helperFunctions.hpp"

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

        PWMMotor flywheelL1;
        PWMMotor flywheelR1;
        PWMMotor flywheelL2;
        PWMMotor flywheelR2;

        int turretXBounds[2] = {0,0};
        int turretYBounds[2] = {0,0};
        int indexerSpeed =  1500;
        int flywheelPower = 60; //0-180

       
        /**
         * @brief Constructor for a chassis
         *
         * @param turretXID pin for left front motor
         * @param turretXType pin for right front motor
         * @param turretYID pin for left front motor
         * @param turretYType pin for right front motor
         * @param indexerID pin for left front motor
         * @param indexerType pin for right front motor
         * 
         * @param bus can bus
         * @param flywheelL1Pin pin for L1 flywheel
         * @param flywheelR1Pin pin for R1 flywheel
         * @param flywheelL2Pin pin for L2 flywheel
         * @param flywheelR2Pin pin for R2 flywheel
         */ 
        TurretSubsystem(int turretXID, motorType turretXType, int turretYID, motorType turretYType,int indexerID, motorType indexerType, CANHandler::CANBus bus, PinName flywheelL1Pin, PinName flywheelR1Pin, PinName flywheelL2Pin, PinName flywheelR2Pin):
            turretX(turretXID,bus,turretXType),
            turretY(turretYID,bus,turretYType),
            indexer(indexerID,bus,indexerType),
            flywheelL1(flywheelL1Pin),
            flywheelR1(flywheelR1Pin),
            flywheelL2(flywheelL2Pin),
            flywheelR2(flywheelR2Pin) //INCOMPLETE
        {}

                /**
         * @brief Constructor for a chassis
         *
         * @param turretXID pin for left front motor
         * @param turretXType pin for right front motor
         * @param turretYID pin for left front motor
         * @param turretYType pin for right front motor
         * @param indexerID pin for left front motor
         * @param indexerType pin for right front motor
         * 
         * @param bus can bus
         * @param flywheelL1Pin pin for L1 flywheel
         * @param flywheelR1Pin pin for R1 flywheel
         * @param flywheelL2Pin pin for L2 flywheel
         * @param flywheelR2Pin pin for R2 flywheel
         */ 
        TurretSubsystem(int turretXID, motorType turretXType, int turretYID, motorType turretYType,int indexerID, motorType indexerType, CANHandler::CANBus bus, PinName flywheelL1Pin, PinName flywheelR1Pin):
            turretX(turretXID,bus,turretXType),
            turretY(turretYID,bus,turretYType),
            indexer(indexerID,bus,indexerType),
            flywheelL1(flywheelL1Pin),
            flywheelR1(flywheelR1Pin),
            flywheelL2(NC),
            flywheelR2(NC) //INCOMPLETE
        {}

        void serializer(int mode, int manualPower = 0){ //mode [2,off] [3,normal] [1, manual]
            int indexJamTime = 0;
            if(mode == 2){
                indexer.setPower(0);
            }else if(mode == 3){
                if(abs(indexer.getData(TORQUE)) > 1000 & abs(indexer.getData(VELOCITY)) < 20){ //jam
                    indexJamTime = us_ticker_read() /1000;
                }
                if(us_ticker_read() / 1000 - indexJamTime < 500){
                    indexer.setPower(-7000); //jam
                    printf("JAMMMMM- ");
                }else if(us_ticker_read() / 1000 - indexJamTime < 750){
                    indexer.setPower(7000); //jam
                    printf("POWER FORWARD- ");
                }else{
                    indexer.setPower(1700);
                }
                printf("AUTO-PWR:%d Jam-Free:%dms TORQ:%d, VELO:%d\n",indexer.powerOut,us_ticker_read() / 1000 - indexJamTime, indexer.getData(TORQUE), indexer.getData(VELOCITY));
            }else if(mode == 1){
                indexer.setPower(manualPower*4);
                printf("MANUAL-PWR:%d VELO:%d\n", indexer.powerOut, indexer.getData(VELOCITY));
            }
        }

        void movePower(int turX, int turY){
            turretX.setPower(turX);
            turretY.setPower(turY);
        }

        void moveRawPos(int turX, int turY){
            turretX.setPosition(std::max(std::min(turX,turretXBounds[0]),-turretXBounds[1]));
            turretY.setPosition(std::max(std::min(turY,turretYBounds[0]),-turretYBounds[1]));
        }

        void movePos(float x, float y){
            int angleX = map(x, -1, 1, turretXBounds[0], turretXBounds[1]);
            int angleY = map(y, -1, 1, turretYBounds[0], turretYBounds[1]);
            moveRawPos(angleX, angleY);
        }
};

#endif  
