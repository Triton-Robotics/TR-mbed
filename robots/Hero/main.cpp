// increase speed --> p
// avoid swing --> d

// choose a p (< 50) and then tune d

#include "main.h"
#include "subsystems/ChassisSubsystem.h"
#include <cmath>

DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);

I2C i2c(I2C_SDA, I2C_SCL);
BNO055 imu(i2c, IMU_RESET, MODE_IMU);

DJIMotor yaw(6, CANHandler::CANBUS_1, GM6020, "YAW");
DJIMotor pitch(5, CANHandler::CANBUS_2, GIMBLY, "PITCH");

DJIMotor indexer(2, CANHandler::CANBUS_2, M3508, "INDEXER");
DJIMotor feeder(5,CANHandler::CANBUS_2, C610, "FEEDER");

DJIMotor top_flywheel(4,CANHandler::CANBUS_2, M3508_FLYWHEEL, "Top Flywheel");
DJIMotor bottom_flywheel(1,CANHandler::CANBUS_2, M3508_FLYWHEEL, "Bottom Flywheel");

ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.29845); // radius is 9 in



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
    pitch.setPositionPID(10,0,5);
    pitch.setPositionOutputCap(16000);
    pitch.useAbsEncoder = true;
    int pitchUpperBound = 2000;
    int pitchLowerBound = 7000;

    long burstTimestamp = 0;

    top_flywheel.setSpeedPID(1.5,0,0);
    bottom_flywheel.setSpeedPID(1.5,0,0);

    indexer.setSpeedPID(3,0,0);

    unsigned long start = us_ticker_read();
    unsigned long current = us_ticker_read();

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();

    unsigned long power = 0;
    int refLoop = 0;
    int burstTimer = 0;

    while (true)
    {
        timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > 10)
        {
            led2 = !led2;

            refLoop++;
            if (refLoop >= 10){              // prints in here, prints less often
                refereeThread(&referee);
                refLoop = 0;
                led2 = !led2;

                // float pComponent = pitch.pidPosition.pComp;
                // float iComponent = pitch.pidPosition.iComp;
                // float dComponent = pitch.pidPosition.dComp;
                // printff("P = %f, I = %f, D = %f\n", pComponent, iComponent, dComponent);

                // printff("Yaw_POS: %d\n", yaw>>ANGLE);
                // printff("burst %d\n", us_ticker_read()-burstTimestamp);

                printff("%f %f %d \n", ext_power_heat_data.data.chassis_power, ext_power_heat_data.data.chassis_power_buffer, 
                ext_game_robot_state.data.chassis_power_limit);
            }

            remoteRead(); 


            //MOVEMENT CODE BEGIN
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
                                          ChassisSubsystem::ROBOT_ORIENTED);
                yaw.setPower(0);
            }
            else if (remote.rightSwitch() == Remote::SwitchState::DOWN)
            {
                Chassis.setChassisSpeeds({jx * Chassis.m_OmniKinematicsLimits.max_Vel, 
                                          jy * Chassis.m_OmniKinematicsLimits.max_Vel, 
                                          0}, 
                                          ChassisSubsystem::ROBOT_ORIENTED);
                yaw.setSpeed(int(-jr * 300));  
            }
            else
            {
                Chassis.setChassisSpeeds({0, 0, 0});
                yaw.setPower(0);
            }

            if(remote.rightSwitch() == Remote::SwitchState::UP || remote.rightSwitch() == Remote::SwitchState::DOWN){
                int mid = (pitchLowerBound + pitchUpperBound)/2;
                int range = pitchLowerBound - pitchUpperBound;
                pitch.setPosition(mid + (remote.rightY()/-660.0 * range));
            }else{
                pitch.setPower(0);
            }
            
            Chassis.periodic();
            //MOVEMENT CODE END
            
            #define feederburstlength 100
            #define burstthreshold 0
            #define indexerburstend 500
            //SHOOTING CODE BEGIN
            double mps = ext_game_robot_state.data.shooter_barrel_heat_limit;
            double rpm = 60 * (mps / 0.03) / (3.14159 * 2);

            if(mps == 0) rpm = 5400;
            // printff("rpm: %f\n",rpm);
            rpm = 4000;
            
            // shooting code
            if (remote.leftSwitch() == Remote::SwitchState::UP ) {
                // printff("shootin' ");
                //pitch.setPower(leftStickValue / 20);

                top_flywheel.setSpeed(rpm);
                bottom_flywheel.setSpeed(-rpm);


                burstTimer = (us_ticker_read() - burstTimestamp)/1000;
                if(burstTimer < feederburstlength){
                    feeder.setPower(8000);
                }else{
                    feeder.setPower(0);
                }
                if(burstTimer < indexerburstend){
                    indexer.setSpeed(20 * M3508_GEAR_RATIO);
                }else{
                    indexer.setPower(0);
                }
            }
            else if (remote.leftSwitch() == Remote::SwitchState::MID ) {
                // printff("revvin' up @%fRPM", rpm);
                // pitch.setPower(0);
                top_flywheel.setSpeed(rpm*0.9);
                bottom_flywheel.setSpeed(-rpm*0.9);
                burstTimestamp = us_ticker_read();
                indexer.setPower(0);
                feeder.setPower(0);
            }else{
                // printff("naaaaah ");
                top_flywheel.setSpeed(0);
                bottom_flywheel.setSpeed(0);
                if(abs(top_flywheel>>VELOCITY) < 500) top_flywheel.setPower(0);
                if(abs(bottom_flywheel>>VELOCITY) < 500) bottom_flywheel.setPower(0);
                burstTimestamp = us_ticker_read();
            }

            // printff("burst %d\n", (us_ticker_read()-burstTimestamp)/1000);
            // int burstTimer = (us_ticker_read() - burstTimestamp)/1000;
            // if(burstTimer > burstthreshold){
                
            // }
            // if(burstTimer >= feederburstlength){
            //     feeder.setPower(0);
            // }
            // if(burstTimer >= feederburstlength){
            //     indexer.setPower(0);
            // }
            //SHOOTING CODE END
            
            loopTimer = timeStart;
            DJIMotor::s_sendValues();

        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}
