#include "main.h"

DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);

I2C i2c(I2C_SDA, I2C_SCL);
BNO055 imu(i2c, IMU_RESET, MODE_IMU);
DJIMotor ChassisOne(1, CANHandler::CANBUS_1, STANDARD, "testMotor");
DJIMotor ChassisTwo(2, CANHandler::CANBUS_1, STANDARD, "testMotor");
DJIMotor ChassisThree(3, CANHandler::CANBUS_1, STANDARD, "testMotor");
DJIMotor ChassisFour(4, CANHandler::CANBUS_1, STANDARD, "testMotor");
DJIMotor yawOne(5, CANHandler::CANBUS_1, GM6020, "testMotor");
// Chassis chassis(1, 2, 3, 4, &i2c);


int main()
{

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();
    DJIMotor::initializedWarning=false;
    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    int refLoop = 0;

    ChassisOne.setSpeedPID(2, 0, 0);
    ChassisTwo.setSpeedPID(2, 0, 0);
    ChassisThree.setSpeedPID(2, 0, 0);
    ChassisFour.setSpeedPID(2, 0, 0);

    

    while (true)
    {
        timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > 25)
        {// 

            led2 = !led2;

            refLoop++;
            if (refLoop >= 10)
            {
                refereeThread(&referee);
                refLoop = 0;
                led = !led;
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
                double max_vel = 0.0254*2* (8000/M3508_GEAR_RATIO) * (2*PI/60);
                double jx = remote.leftX() / 660.0;
                double jy = remote.leftY() / 660.0;
                double jr = remote.rightX() / 660.0;

                double LFa = (1/sqrt(2))*(jx + jy + jr *((-0.228) - (0.228)));
                ChassisOne.setSpeed(LFa / (0.0254*2) / (2*PI/60) * M3508_GEAR_RATIO );
                // ChassisOne.setPower(LFa);
                double RFa = (1/sqrt(2))*(jx - jy - jr *((0.228) + (0.228)));
                ChassisTwo.setSpeed(RFa / (0.0254*2) / (2*PI/60) * M3508_GEAR_RATIO );
                // ChassisTwo.setPower(RFa);
                double LBa = (1/sqrt(2))*(-jx  + jy + jr *((-0.228) + (-0.228)));
                ChassisThree.setSpeed(LBa / (0.0254*2) / (2*PI/60) * M3508_GEAR_RATIO );
                // ChassisThree.setPower(LBa);
                double RBa = (1/sqrt(2))*(-jx  - jy - jr *((0.228) - (-0.228)));
                ChassisFour.setSpeed(RBa / (0.0254*2) / (2*PI/60) * M3508_GEAR_RATIO );
                // ChassisFour.setPower(RBa);

                double a[4] = {LFa, RFa, LBa, RBa};
                for (int i = 0; i < 4; i++)
                {
                    printff("%f ", a[i]);
                }
                printff("\n");
            }
            unsigned long time = us_ticker_read();

            loopTimer = timeStart;
            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}
