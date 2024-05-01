// increase speed --> p
// avoid swing --> d

// choose a p (< 50) and then tune d

#include "main.h"
#include "subsystems/ChassisSubsystem.h"
#include <cmath>

DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);

DJIMotor pitch(7, CANHandler::CANBUS_2, GIMBLY, "PITCH");

I2C i2c(I2C_SDA, I2C_SCL);
BNO055 imu(i2c, IMU_RESET, MODE_IMU);

DJIMotor yaw(5, CANHandler::CANBUS_1, GM6020, "YAW");
ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2286); // radius is 9 in

int main(){

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();

    Chassis.setYawReference(&yaw, 2050); // "5604" is the number of ticks of yawOne considered to be robot-front
    Chassis.setSpeedFF_Ks(0.065);

    // yaw.useAbsEncoder = true;
    yaw.setSpeedPID(10, 0, 0);
    yaw.setSpeedOutputCap(32000);

    pitch.useAbsEncoder = true;
    pitch.setPositionPID(20, 0, 4000);
    pitch.setPositionOutputCap(32000);
    float currentPitch = 0;
    float desiredPitch = 0;
    float pitch_phase = -23 / 180.0 * PI;
    float InitialOffset_Ticks = 2500;

    // FeedForward Modeling: FF = K * sin(theta) , output: [-1,1]
    float K = 0.85;

    unsigned long start = us_ticker_read();
    unsigned long current = us_ticker_read();
    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    unsigned long power = 0;
    int refLoop = 0;

    while (true)
    {
        timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > 25)
        {
            led2 = !led2;

            refLoop++;
            if (refLoop >= 5){              // prints in here, prints less often
                refereeThread(&referee);
                refLoop = 0;
                led2 = !led2;

                // float pComponent = pitch.pidPosition.pComp;
                // float iComponent = pitch.pidPosition.iComp;
                // float dComponent = pitch.pidPosition.dComp;
                // printff("P = %f, I = %f, D = %f\n", pComponent, iComponent, dComponent);

                printff("Yaw_POS: %d\n", yaw.getData(ANGLE));
            }

            remoteRead(); 

            double scalar = 1;
            double jx = remote.leftX() / 660.0 * scalar;
            double jy = remote.leftY() / 660.0 * scalar;
            double jr = remote.rightX() / 660.0 * scalar;

            double tolerance = 0.05;
            jx = (abs(jx) < tolerance) ? 0 : jx;
            jy = (abs(jy) < tolerance) ? 0 : jy;
            jr = (abs(jr) < tolerance) ? 0 : jr;

            if (remote.rightSwitch() == Remote::SwitchState::UP)
            {           
                Chassis.setChassisSpeeds({jx * Chassis.m_OmniKinematicsLimits.max_Vel, 
                                          jy * Chassis.m_OmniKinematicsLimits.max_Vel, 
                                          -jr * Chassis.m_OmniKinematicsLimits.max_vOmega}, 
                                          ChassisSubsystem::YAW_ORIENTED);
                yaw.setPower(0);
            }
            else if (remote.rightSwitch() == Remote::SwitchState::DOWN)
            {
                Chassis.setChassisSpeeds({jx * Chassis.m_OmniKinematicsLimits.max_Vel, 
                                          jy * Chassis.m_OmniKinematicsLimits.max_Vel, 
                                          0}, 
                                          ChassisSubsystem::YAW_ORIENTED);
                yaw.setSpeed(int(-jr * 300));  
            }
            else
            {
                Chassis.setChassisSpeeds({0, 0, 0});
                yaw.setPower(0);
            }
            Chassis.periodic();

            // current = us_ticker_read();
            // if((current - start) / 1000 > 12000) {
            //     power += 1000;
            //     start = us_ticker_read();
            // }
            
            
            currentPitch = (double(pitch.getData(ANGLE) - InitialOffset_Ticks) / TICKS_REVOLUTION) * 360; // degrees
            
            // left switch state
            Remote::SwitchState leftSwitchState = remote.leftSwitch();

            // left stick value
            int stick = remote.rightY();

            // check switch mode
            if ( leftSwitchState == Remote::SwitchState::UP ) {
                //pitch.setPower(leftStickValue / 20);
                desiredPitch = stick / 20;
            }
            else if ( leftSwitchState == Remote::SwitchState::DOWN ) {
                pitch.setPower(0);
            }
            /*
            else if ( leftSwitchState == Remote::SwitchState::MID ) {
                pitch.setSpeed(leftStickValue * 10);
            }
            else if ( leftSwitchState == Remote::SwitchState::MID ) {
                pitch.setPosition(leftStickValue * 10);
            }
            */

            // want to have the desired pitch work based on the controller rather than manually setting it
            // desiredPitch = 45; // degrees
            float FF = K * sin((desiredPitch / 180 * PI) - pitch_phase); // output: [-1,1]
            pitch.pidPosition.feedForward = int((INT16_T_MAX) * FF);
            pitch.setPosition(int((desiredPitch / 360) * TICKS_REVOLUTION + InitialOffset_Ticks));

            //printff("%d %f %f\n", pitch.getData(POWEROUT), currentPitch, FF);

            
            loopTimer = timeStart;
            DJIMotor::s_sendValues();

        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}
