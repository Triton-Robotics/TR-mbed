#include "main.h"
#include "subsystems/ChassisSubsystem.h"

DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);

I2C i2c(I2C_SDA, I2C_SCL);
BNO055 imu(i2c, IMU_RESET, MODE_IMU);
// DJIMotor ChassisOne(1, CANHandler::CANBUS_1, STANDARD, "testMotor");
// DJIMotor ChassisTwo(2, CANHandler::CANBUS_1, STANDARD, "testMotor");
// DJIMotor ChassisThree(3, CANHandler::CANBUS_1, STANDARD, "testMotor");
// DJIMotor ChassisFour(4, CANHandler::CANBUS_1, STANDARD, "testMotor");
DJIMotor yawOne(5, CANHandler::CANBUS_1, GM6020, "testMotor");
ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2286); // radius is 9 in

int main()
{

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();
    DJIMotor::initializedWarning = false;
    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    int refLoop = 0;

    while (true)
    {
        timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > 25)
        {
            led2 = !led2;

            refLoop++;
            if (refLoop >= 10)
            {
                refereeThread(&referee);
                refLoop = 0;
                led = !led;
    
                // printff("%f\n", Chassis.getMotorSpeed(ChassisSubsystem::LEFT_FRONT, ChassisSubsystem::METER_PER_SECOND));
                // printff("%d\n", Chassis.getMotor(ChassisSubsystem::LEFT_FRONT)>>VELOCITY);
                // printff("%s\n", Chassis.desiredWheelSpeeds.to_string());
                WheelSpeeds ws = Chassis.desiredWheelSpeeds;

                // printff("%d ", Chassis.getMotor(ChassisSubsystem::LEFT_BACK).getData(POWEROUT));
                // printff("%f\n", Chassis.getMotorSpeed(ChassisSubsystem::LEFT_BACK, ChassisSubsystem::RPM) / M3508_GEAR_RATIO);
                // printff("%f ", Chassis.desiredWheelSpeeds.LB / M3508_GEAR_RATIO * 2 * PI / 60 * WHEEL_DIAMETER_METERS / 2);
                // printff("%f\n",Chassis.getMotorSpeed(ChassisSubsystem::LEFT_BACK, ChassisSubsystem::METER_PER_SECOND));
                // printff("LF: %4.2f RF: %4.2f LB: %4.2f RB: %4.2f\n", ws.LF, ws.RF, ws.LB, ws.RB);

                printff("%f\n", sin(PI));

            }

            remoteRead();

            // gets the angle off the chassis
            BNO055_ANGULAR_POSITION_typedef imuAngle;
            imu.get_angular_position_quat(&imuAngle);

            // printff("%f\n", imuAngle.yaw);

            // gets the angle of the motor

            // printff("%d\n", yawOne.getData(ANGLE));
            if (remote.rightSwitch() == Remote::SwitchState::UP)
            {
                double scalar = 1;
                double jx = remote.leftX() / 660.0 * scalar;
                double jy = remote.leftY() / 660.0 * scalar;
                double jr = remote.rightX() / 660.0 * scalar;

                double tolerance = 0.05;
                jx = (abs(jx) < tolerance) ? 0 : jx;
                jy = (abs(jy) < tolerance) ? 0 : jy;
                jr = (abs(jr) < tolerance) ? 0 : jr;

                Chassis.setSpeedFF_Ks(0.065);
                Chassis.setChassisSpeeds({jx * Chassis.m_OmniKinematicsLimits.max_Vel, jy * Chassis.m_OmniKinematicsLimits.max_Vel, jr * Chassis.m_OmniKinematicsLimits.max_vOmega});
                // Chassis.setChassisPower({jx, jy, jr});
                
                // double LFa = (1 / sqrt(2)) * (jx + jy + jr * ((-0.228) - (0.228)));
                // ChassisOne.setSpeed(LFa / (0.0254 * 2) / (2 * PI / 60) * M3508_GEAR_RATIO);
                // // ChassisOne.setPower(LFa);
                // double RFa = (1 / sqrt(2)) * (jx - jy - jr * ((0.228) + (0.228)));
                // ChassisTwo.setSpeed(RFa / (0.0254 * 2) / (2 * PI / 60) * M3508_GEAR_RATIO);
                // // ChassisTwo.setPower(RFa);
                // double LBa = (1 / sqrt(2)) * (-jx + jy + jr * ((-0.228) + (-0.228)));
                // ChassisThree.setSpeed(LBa / (0.0254 * 2) / (2 * PI / 60) * M3508_GEAR_RATIO);
                // // ChassisThree.setPower(LBa);
                // double RBa = (1 / sqrt(2)) * (-jx - jy - jr * ((0.228) - (-0.228)));
                // ChassisFour.setSpeed(RBa / (0.0254 * 2) / (2 * PI / 60) * M3508_GEAR_RATIO);
                // // ChassisFour.setPower(RBa);

                // double a[4] = {LFa, RFa, LBa, RBa};
                // for (int i = 0; i < 4; i++)
                // {
                //     printff("%f ", a[i]);
                // }
                // printff("\n");
            }
            else
            {
                Chassis.setChassisSpeeds({0,0,0});
            }
            Chassis.periodic();

            unsigned long time = us_ticker_read();

            loopTimer = timeStart;
            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}
