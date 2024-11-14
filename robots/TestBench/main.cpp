#include "main.h"
#include <cstdlib>

#include <iostream>
#include "subsystems/ChassisSubsystem.h"

DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);
DigitalOut ledbuiltin(LED1);

I2C i2c(I2C_SDA, I2C_SCL);
BNO055 imu(i2c, IMU_RESET, MODE_IMU);
ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2286);

#define IMPULSE_DT 100
#define IMPULSE_STRENGTH 16383
#define POWER 1000


float power_theory(float give_current, float rotor_speed) {

    float It = (20/16384) * give_current;
    float Kt = 0.01562;
    float torquee = It + Kt;

    float mechPower = (torquee * rotor_speed)/9.55;
    float theory_in = mechPower + ((0.000000162) * (rotor_speed * rotor_speed)) + (6448.8) * (torquee * torquee) + 2.77;

    return theory_in;

}

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

                    float Pmax = robot_status.chassis_power_limit;

                    double jx = remote.leftX() / 660.0;
                    double jy = remote.leftY() / 660.0;
                    double jr = remote.rightX() / 660.0;

                    
                    double tolerance = 0.05;
                    jx = (abs(jx) < tolerance) ? 0 : jx;
                    jy = (abs(jy) < tolerance) ? 0 : jy;
                    jr = (abs(jr) < tolerance) ? 0 : jr;

                    Chassis.setSpeedFF_Ks(0.065);
                    Chassis.setChassisSpeedsPWR(Pmax, {jx * Chassis.m_OmniKinematicsLimits.max_Vel,
                                              jy * Chassis.m_OmniKinematicsLimits.max_Vel,
                                              0  * Chassis.m_OmniKinematicsLimits.max_vOmega},
                                              ChassisSubsystem::YAW_ORIENTED);

                    // float chassis_power = power_heat_data.chassis_power; 
                    // float It = abs((Chassis.getMotor(ChassisSubsystem::LEFT_FRONT).getData(POWEROUT) + 
                    //             Chassis.getMotor(ChassisSubsystem::RIGHT_FRONT).getData(POWEROUT) + 
                    //             Chassis.getMotor(ChassisSubsystem::LEFT_BACK).getData(POWEROUT) + 
                    //             Chassis.getMotor(ChassisSubsystem::RIGHT_BACK).getData(POWEROUT))*(20/16384));
                    // float Kt = 0.01562;
                    // float torque = It + Kt;
                    // float rotorSpeed = abs(Chassis.getMotor(ChassisSubsystem::LEFT_FRONT).getData(VELOCITY) + 
                    //                     Chassis.getMotor(ChassisSubsystem::RIGHT_FRONT).getData(VELOCITY) + 
                    //                     Chassis.getMotor(ChassisSubsystem::LEFT_BACK).getData(VELOCITY) + 
                    //                     Chassis.getMotor(ChassisSubsystem::RIGHT_BACK).getData(VELOCITY));
                    // float mechPower = (torque*rotorSpeed)/9.55;
                    // float theory_in = mechPower + ((0.000000162)*(rotorSpeed*rotorSpeed)) + (6448.8)*(torque*torque) + 2.77;    
                    // printff("P_in:%.4f\tP_theory:%.4f\n",chassis_power,theory_in);




                    // float give_current_M1 = 20*Chassis.getMotor(ChassisSubsystem::LEFT_FRONT).getData(POWEROUT)/IMPULSE_STRENGTH;
                    // float give_current_M2 = 20*Chassis.getMotor(ChassisSubsystem::RIGHT_FRONT).getData(POWEROUT)/IMPULSE_STRENGTH;
                    // float give_current_M3 = 20*Chassis.getMotor(ChassisSubsystem::LEFT_BACK).getData(POWEROUT)/IMPULSE_STRENGTH;
                    // float give_current_M4 = 20*Chassis.getMotor(ChassisSubsystem::RIGHT_BACK).getData(POWEROUT)/IMPULSE_STRENGTH;

                    // float rotor_speed_Motor1 = Chassis.getMotor(ChassisSubsystem::LEFT_FRONT).getData(VELOCITY);
                    // float rotor_speed_Motor2 = Chassis.getMotor(ChassisSubsystem::RIGHT_FRONT).getData(VELOCITY);
                    // float rotor_speed_Motor3 = Chassis.getMotor(ChassisSubsystem::LEFT_BACK).getData(VELOCITY);
                    // float rotor_speed_Motor4 = Chassis.getMotor(ChassisSubsystem::RIGHT_BACK).getData(VELOCITY);


                    // float P_cmd_1 = power_theory(give_current_M1,rotor_speed_Motor1);
                    // float P_cmd_2 = power_theory(give_current_M2,rotor_speed_Motor2);
                    // float P_cmd_3 = power_theory(give_current_M3,rotor_speed_Motor3);
                    // float P_cmd_4 = power_theory(give_current_M4,rotor_speed_Motor4);

                    // float sum_Pcmd = P_cmd_1 + P_cmd_2 + P_cmd_3 + P_cmd_1;
                    // float k = Pmax/sum_Pcmd;

                    // printff("%.5f\n", k);

                    // float Pout_motor1 = k * Chassis.getMotor(ChassisSubsystem::LEFT_FRONT).getData(POWEROUT); 
                    // float Pout_motor2 = k * Chassis.getMotor(ChassisSubsystem::RIGHT_FRONT).getData(POWEROUT); 
                    // float Pout_motor3 = k * Chassis.getMotor(ChassisSubsystem::LEFT_BACK).getData(POWEROUT); 
                    // float Pout_motor4 = k * Chassis.getMotor(ChassisSubsystem::RIGHT_BACK).getData(POWEROUT); 

                    // printff("%d\t%d\t%d\t%d\n", Chassis.getMotor(ChassisSubsystem::LEFT_FRONT).getData(POWEROUT),
                    //                             Chassis.getMotor(ChassisSubsystem::RIGHT_FRONT).getData(POWEROUT),
                    //                             Chassis.getMotor(ChassisSubsystem::LEFT_BACK).getData(POWEROUT),
                    //                             Chassis.getMotor(ChassisSubsystem::RIGHT_BACK).getData(POWEROUT));

                    // Chassis.getMotor(ChassisSubsystem::LEFT_FRONT).setPower(Pout_motor1); 
                    // Chassis.getMotor(ChassisSubsystem::RIGHT_FRONT).setPower(Pout_motor2);
                    // Chassis.getMotor(ChassisSubsystem::LEFT_BACK).setPower(Pout_motor3);
                    // Chassis.getMotor(ChassisSubsystem::RIGHT_BACK).setPower(Pout_motor4);

                }

                else {
                    Chassis.getMotor(ChassisSubsystem::LEFT_FRONT).setSpeed(0); 
                    Chassis.getMotor(ChassisSubsystem::RIGHT_FRONT).setSpeed(0);
                    Chassis.getMotor(ChassisSubsystem::LEFT_BACK).setSpeed(0);
                    Chassis.getMotor(ChassisSubsystem::RIGHT_BACK).setSpeed(0);
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


// float power_theory(float give_current, float rotor_speed) {

//     It = (20/16384) * give_current;
//     Kt = 0.01562;
//     torquee = It + Kt;

//     float mechPower = (torqueerotor_speed)/9.55;
//     float theory_in = mechPower + ((0.000000162)(rotor_speedrotor_speed)) + (6448.8)(torqueetorquee) + 2.77;

//     return theory_in;

// }


// P_cmd_1 = power_theory(give_current_M1,rotor_speed_Motor1);
// P_cmd_2 = power_theory(give_current_M2,rotor_speed_Motor2);
// P_cmd_3 = power_theory(give_current_M3,rotor_speed_Motor3);
// P_cmd_4 = power_theory(give_current_M4,rotor_speed_Motor4);

// sum_Pcmd = P_cmd_1 + P_cmd_2 + P_cmd_3 + P_cmd_1;
// k = Pmax/sum_Pcmd;


// float Pout_motor1 = k * P_cmd_1;
// float Pout_motor2 = k * P_cmd_2;
// float Pout_motor3 = k * P_cmd_3;
// float Pout_motor4 = k * P_cmd_4;

// motor1.setmotorpower(Pout_motor1);
// motor2.setmotorpower(Pout_motor2);
// motor3.setmotorpower(Pout_motor3);
// motor4.setmotorpower(Pout_motor4);




































































































































