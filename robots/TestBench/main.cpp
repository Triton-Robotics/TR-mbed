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
    DJIMotor::initializedWarning = false;

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
        {

            unsigned long time = us_ticker_read();

            led2 = !led2;

            refLoop++;
            if (refLoop >= 10)
            {
                refereeThread(&referee);
                refLoop = 0;
                led = !led;
                //  printff("angle:%f\n", euler_angles.h);
            }

            remoteRead();
            // gets the angle off the chassis
            BNO055_ANGULAR_POSITION_typedef angle_quat;
            imu.get_angular_position_quat(&angle_quat);

            // printff("%f\n", angle_quat.yaw);

            // gets the angle of the motor

            // printff("%d\n", yawOne.getData(ANGLE));
            if (remote.rightSwitch() == Remote::SwitchState::UP)
            {
                int LFa = (-1 * remote.leftY()) + remote.leftX() + remote.rightX();
                ChassisOne.setSpeed(LFa);
                // ChassisOne.setPower(LFa);
                int RFa = (1 * remote.leftY()) - remote.leftX() - remote.rightX();
                ChassisTwo.setSpeed(-RFa);
                // ChassisTwo.setPower(RFa);
                int LBa = remote.leftY() - remote.leftX() + remote.rightX();
                ChassisThree.setSpeed(LBa);
                // ChassisThree.setPower(LBa);
                int RBa = remote.leftY() + remote.leftX() - remote.rightX();
                ChassisFour.setSpeed(-RBa);
                // ChassisFour.setPower(RBa);
                int a[4] = {LFa, -RFa, LBa, -RBa};
                for (int i = 0; i < 4; i++)
                {
                    printff("%d ", a[i]);
                }
                printff("\n");
            }
            loopTimer = timeStart;
            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}