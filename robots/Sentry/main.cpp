#include <mbed.h>
#include <main.h>
#include "peripherals/oled/Adafruit_SSD1306.h"
#include "communications/Jetson.h"

Jetson jetson(PC_12, PD_2);
Thread cv;

DJIMotor yaw1(5, CANHandler::CANBUS_1, GIMBLY);
DJIMotor yaw2(7, CANHandler::CANBUS_1, GIMBLY);
DJIMotor pitch(6, CANHandler::CANBUS_2, GIMBLY);
DJIMotor indexerL(7, CANHandler::CANBUS_2, C610);
DJIMotor indexerR(8, CANHandler::CANBUS_2, C610);

// add radius measurement here
#define RADIUS 0.5

I2C i2c(I2C_SDA, I2C_SCL);
BNO055 imu(i2c, IMU_RESET, MODE_IMU);
ChassisSubsystem chassis(2, 1, 3, 4, imu, RADIUS);
Adafruit_SSD1306_I2c oled(i2c, LED1);

DJIMotor RTOPFLYWHEEL(1, CANHandler::CANBUS_2, M3508_FLYWHEEL);
DJIMotor LTOPFLYWHEEL(2, CANHandler::CANBUS_2, M3508_FLYWHEEL);
DJIMotor RBOTTOMFLYWHEEL(4, CANHandler::CANBUS_2, M3508_FLYWHEEL);
DJIMotor LBOTTOMFLYWHEEL(3, CANHandler::CANBUS_2, M3508_FLYWHEEL);

void setFlyWheelSpeed(int speed)
{
    LTOPFLYWHEEL.setSpeed(-speed);
    LBOTTOMFLYWHEEL.setSpeed(speed);
    RTOPFLYWHEEL.setSpeed(speed);
    RBOTTOMFLYWHEEL.setSpeed(-speed);
}
void setFlyWheelPower(int speed)
{
    LTOPFLYWHEEL.setPower(-speed);
    LBOTTOMFLYWHEEL.setPower(speed);
    RTOPFLYWHEEL.setPower(speed);
    RBOTTOMFLYWHEEL.setPower(-speed);
}

bool xyzToSpherical(double x, double y, double z, double &phi, double &theta)
{
    double mag = sqrt(x * x + y * y + z * z);

    if (mag > 28 && mag < 32)
    {

        x /= mag;
        y /= mag;
        z /= mag;

        // z
        phi = acos(z);
        phi *= 180 / PI;
        phi -= 90;

        // xy
        theta = atan2(y, x);
        theta *= 180 / PI;
        theta -= 90;

        return true;
    }
    return false;
}

void setMotorSettings()
{
    pitch.pidPosition.feedForward = 1900;

    pitch.setPositionPID(24.3, 0.3, 35.5);
    pitch.setPositionIntegralCap(10000);
    pitch.setPositionOutputCap(20000);
    pitch.useAbsEncoder = true;

    yaw1.setPositionPID(10.5, 0.2, 4.4);
    yaw1.setPositionIntegralCap(10000);
    yaw1.useAbsEncoder = false;
    yaw1.pidPosition.setOutputCap(100000);
    yaw1.outputCap = 32000;

    indexerL.setSpeedPID(1.94, 0.002, 0.166);
    indexerL.setSpeedIntegralCap(500000);
}

int main()
{

    int yawSetPoint = 0;
    setMotorSettings();

    cv.start([]()
             {
        while (true) {
            //jetson.read();
            jetson.write();
        } });

    DigitalOut led(L26);
    DigitalOut led2(L27);

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_getFeedback();

    int refLoop = 0;
    uint16_t max_power;
    float ref_chassis_power;
    int ref_yaw;
    double rotationalPower;

    double phi = 0;
    double theta = 0;

    double x, y, z;
    int p;

    unsigned long loopTimer = us_ticker_read();
    unsigned long timeStart;
    unsigned long timeEnd;

    while (true)
    {

        timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > 25)
        {
            loopTimer = timeStart;

            // led = !led;
            remoteRead();

            refLoop++;
            if (refLoop >= 5)
            {
                refereeThread(&referee);
                refLoop = 0;
                led2 = !led2;
            }

            ref_chassis_power = ext_power_heat_data.data.chassis_power;
            max_power = ext_game_robot_state.data.chassis_power_limit;
            ref_yaw = ext_game_robot_pos.data.yaw;

            jetson.set(Jetson::cv::X, ref_yaw);
            jetson.set(Jetson::cv::Y, (pitch.getData(ANGLE) - 6890) * 360.0 / 8191.0);

            // printf("%d %d\n", ref_yaw, int(-(pitch.getData(ANGLE) - 6890) * 360.0 / 8191));

            // printff("Ref power: %i\n", (int) (ref_chassis_power * 100));

            // printf("RS: %i\n", rS);
            // printff("Pitch:%d PWR: %d\n",pitch.getData(ANGLE),pitch.getData(POWEROUT));
            // printff("M2:%d %d\n",chassis.getMotor(1)>>VELOCITY,chassis.getMotor(1)>>POWEROUT);
            // chassis.printMotorAngle();

            // printff("%d\n",Wh);

            double beyblade = 0;

            // MOVEMENT CODE BEGIN
            double scalar = 1;
            double jx = remote.leftX() / 660.0 * scalar;
            double jy = remote.leftY() / 660.0 * scalar;
            double jr = remote.rightX() / 660.0 * scalar;

            double tolerance = 0.05;
            jx = (abs(jx) < tolerance) ? 0 : jx;
            jy = (abs(jy) < tolerance) ? 0 : jy;
            jr = (abs(jr) < tolerance) ? 0 : jr;

            if (remote.rightSwitch() == Remote::SwitchState::MID || remote.rightSwitch() == Remote::SwitchState::UNKNOWN)
            {
                chassis.setWheelPower({0, 0, 0, 0});
                yaw1.setPower(0);
                yaw2.setPower(0);
                pitch.setPower(0);
                // printf("%d %d", ref_yaw, yaw1.getData(ANGLE));

                beyblade = 0;
            }
            else if (remote.rightSwitch() == Remote::SwitchState::DOWN)
            {
                // yawSetPoint -= remote.rightX() / 4.5;
                // yaw1.setPosition(-yawSetPoint);
                // pitch.setPosition((2 * remote.rightY() / 3) + 6700);
                // yaw2.setPower(yaw1.powerOut);
                // beyblade = 0;

                // printf("%d, %d\n", -yawSetPoint, yaw1.getData(MULTITURNANGLE));
                chassis.setChassisSpeeds({jx * chassis.m_OmniKinematicsLimits.max_Vel,
                                          jy * chassis.m_OmniKinematicsLimits.max_Vel,
                                          0},
                                         ChassisSubsystem::ROBOT_ORIENTED);
                yaw1.setSpeed(int(-jr * 300));
                yaw2.setSpeed(int(-jr * 300));
            }
            else if (remote.rightSwitch() == Remote::SwitchState::UP)
            {

                // x = jetson.get(Jetson::cv::Z);
                // y = jetson.get(Jetson::cv::X);
                // z = jetson.get(Jetson::cv::Y);

                // if (xyzToSpherical(x, y, z, phi, theta))
                //     printf("%f %f %f %f %f\n", x, y, z, phi, theta);

                // printf("%d\n", p);

                // // 6890
                // //                printf("%d\n", pitch.getData(ANGLE));
                chassis.setChassisSpeeds({jx * chassis.m_OmniKinematicsLimits.max_Vel,
                                          jy * chassis.m_OmniKinematicsLimits.max_Vel,
                                          -jr * chassis.m_OmniKinematicsLimits.max_vOmega},
                                         ChassisSubsystem::ROBOT_ORIENTED);
                yaw1.setPower(0);
                yaw2.setPower(0);
            }
            int pitchUpperBound = 2000;
            int pitchLowerBound = 7000;
            if (remote.rightSwitch() == Remote::SwitchState::UP || remote.rightSwitch() == Remote::SwitchState::DOWN)
            {
                int mid = (pitchLowerBound + pitchUpperBound) / 2;
                int range = pitchLowerBound - pitchUpperBound;
                pitch.setPosition(mid + (remote.rightY() / -660.0 * range));
            }
            else
            {
                pitch.setPower(0);
            }

            if (remote.leftSwitch() == Remote::SwitchState::UP)
            {
                //              indexerL.setSpeed(-2000);
                setFlyWheelPower(8000);
                indexerL.setSpeed(2000);
                indexerR.setSpeed(-2000);
                // printff("indexer.s%d\n")
            }
            else if (remote.leftSwitch() == Remote::SwitchState::MID)
            {
                //              indexerL.setSpeed(-2000);
                setFlyWheelPower(8000);
                indexerL.setPower(0);
                indexerR.setPower(0);
                // printff("indexer.s%d\n")
            }
            else
            { // disable serializer
                indexerL.setPower(0);
                indexerR.setPower(0);
                setFlyWheelPower(0);
                // LTOPFLYWHEEL.setPower(0);
            }

            chassis.periodic();
            timeEnd = us_ticker_read();
            // chassis.driveTurretRelativePower(ref_chassis_power, max_power, {-remote.leftX() * 5.0, -remote.leftY() * 5.0, beyblade}, -yaw1.getData(MULTITURNANGLE) * 360.0 / 8192 + 90, int(timeEnd - timeStart), rotationalPower);
            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}