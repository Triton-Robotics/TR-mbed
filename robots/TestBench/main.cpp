#include "main.h"
#include <cstdlib>

#include <iostream>
#include "subsystems/ChassisSubsystem.h"

DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);
DigitalOut ledbuiltin(LED1);

// DJIMotor testMot(3, CANHandler::CANBUS_1, STANDARD, "testMotor");

I2C i2c(I2C_SDA, I2C_SCL);
BNO055 imu(i2c, IMU_RESET, MODE_IMU);
ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2286);

#define IMPULSE_DT 100
#define IMPULSE_STRENGTH 16383
#define POWER 1000




int main(){

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    int refLoop = 0;

    // testMot.setSpeedPID(1.5,0,200);

    bool prevL = false;
    bool switL = false;

    int motorSpeed = 0;

    while(true){
        timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > 25){
            loopTimer = timeStart;
            led = !led;
            ledbuiltin = !ledbuiltin;

            refLoop++;
            if (refLoop >= 5){
                refereeThread(&referee);
                refLoop = 0;
                led2 = !led2;
                // if (remote.leftSwitch() == Remote::SwitchState::UP) {
                    
                //     testMot.setPower(remote.leftX()*20);
                //     // testMot1.setPower(POWER);


                //     float current = 20*testMot.getData(POWEROUT)/IMPULSE_STRENGTH;
                //     float chassis_power = power_heat_data.chassis_power;  
                //     float It = current;
                //     float Kt = 0.01562;
                //     float torque = Kt * It;
                //     int rotorSpeed = testMot.getData(VELOCITY);
                //     float mechPower = (torque*rotorSpeed) / 9.55;
                //     // P_in = P_m + k1 * ω² + k2 * τ² + a
                //     float theory_in = mechPower + ((0.000000162)*(rotorSpeed*rotorSpeed)) + (6448.8)*(torque*torque) + 2.77;    
                //     printff("P_in:%.4f\tP_theory:%.4f\n",chassis_power,theory_in);
                //     // printff("omega:%d,tau:%.5f,P_in:%.3f,P_m:%.3f\n", abs(rotorSpeed), abs(torque), abs(chassis_power), abs(mechPower));
                //     // printff("%.5f,%d,%.5f\n", chassis_power, rotorSpeed, torque);
                //     // printff("%d,%.5f,%.3f,%.3f\n", rotorSpeed, torque, chassis_power, mechPower);
                    
                // }

                // else if (remote.leftSwitch() == Remote::SwitchState::MID) {
                //     int power = remote.leftX()*6;
                //     testMot.setPower(power);
                //     // printff("power:%d\tomega:%d\n", power, testMot.getData(VELOCITY));

                //     float current = 20*testMot.getData(POWEROUT)/IMPULSE_STRENGTH;
                //     float chassis_power = power_heat_data.chassis_power;  
                //     float It = current;
                //     float Kt = 0.01562;
                //     float torque = Kt * It;
                //     int rotorSpeed = testMot.getData(VELOCITY);
                //     float mechPower = (torque*rotorSpeed) / 9.55;
                //     printff("%.5f,%d,%.5f\n", chassis_power, rotorSpeed, torque);

                // }
                if (remote.leftSwitch() == Remote::SwitchState::MID) {

                    double jx = remote.leftX() / 660.0;
                    double jy = remote.leftY() / 660.0;
                    double jr = remote.rightX() / 660.0;

                    Chassis.setSpeedFF_Ks(0.065);
                    Chassis.setChassisSpeeds({jx * Chassis.m_OmniKinematicsLimits.max_Vel,
                                              jy * Chassis.m_OmniKinematicsLimits.max_Vel,
                                              0  * Chassis.m_OmniKinematicsLimits.max_vOmega},
                                              ChassisSubsystem::YAW_ORIENTED);

                    float chassis_power = power_heat_data.chassis_power; 
                    float It = abs((Chassis.getMotor(ChassisSubsystem::LEFT_FRONT).getData(POWEROUT) + 
                                Chassis.getMotor(ChassisSubsystem::RIGHT_FRONT).getData(POWEROUT) + 
                                Chassis.getMotor(ChassisSubsystem::LEFT_BACK).getData(POWEROUT) + 
                                Chassis.getMotor(ChassisSubsystem::RIGHT_BACK).getData(POWEROUT))*(20/16384));
                    float Kt = 0.01562;
                    float torque = It + Kt;
                    float rotorSpeed = abs(Chassis.getMotor(ChassisSubsystem::LEFT_FRONT).getData(VELOCITY) + 
                                        Chassis.getMotor(ChassisSubsystem::RIGHT_FRONT).getData(VELOCITY) + 
                                        Chassis.getMotor(ChassisSubsystem::LEFT_BACK).getData(VELOCITY) + 
                                        Chassis.getMotor(ChassisSubsystem::RIGHT_BACK).getData(VELOCITY));
                    float mechPower = (torque*rotorSpeed)/9.55;
                    float theory_in = mechPower + ((0.000000162)*(rotorSpeed*rotorSpeed)) + (6448.8)*(torque*torque) + 2.77;    
                    printff("P_in:%.4f\tP_theory:%.4f\n",chassis_power,theory_in);


                }

                else {
                    Chassis.getMotor(ChassisSubsystem::LEFT_FRONT).setPower(0); 
                    Chassis.getMotor(ChassisSubsystem::RIGHT_FRONT).setPower(0);
                    Chassis.getMotor(ChassisSubsystem::LEFT_BACK).setPower(0);
                    Chassis.getMotor(ChassisSubsystem::RIGHT_BACK).setPower(0);
                    printff("off\n");
                }
            }
            remoteRead();

            

            
            

            
            // prevL = switL;
            // remoteRead();
            // switL = (remote.leftSwitch() == Remote::SwitchState::UP);

            // // testMot.setPower(remote.leftX() * 3);
            // // if(!prevL && switL){
            // //     motorSpeed += 10;
            // // }

            // motorSpeed = remote.leftX() / 6;

            // testMot.setSpeed(motorSpeed);
            // int dir = 0;
            // if(motorSpeed > 0){
            //     dir = 1;
            // }else if(motorSpeed < 0){
            //     dir = -1;
            // }
            // testMot.pidSpeed.feedForward = dir * (874 + (73.7 * abs(motorSpeed)) + (0.0948 * motorSpeed * motorSpeed));
            // printff("%d\t%d\t%d\n", testMot>>POWEROUT, motorSpeed, testMot>>VELOCITY);

            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}