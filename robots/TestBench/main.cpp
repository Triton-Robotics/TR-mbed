// #include "main.h"
// #include <cstdlib>


// #include <iostream>
// #include "subsystems/ChassisSubsystem.h"

// DigitalOut led(L27);
// DigitalOut led2(L26);
// DigitalOut led3(L25);
// DigitalOut ledbuiltin(LED1);
// // AnalogIn curr(PA_7);

// I2C i2c(I2C_SDA, I2C_SCL);
// BNO055 imu(i2c, IMU_RESET, MODE_IMU);
// ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2286);

// #define IMPULSE_DT 100
// #define IMPULSE_STRENGTH 16383
// #define POWER 1000

// int main(){

//     DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
//     DJIMotor::s_sendValues();
//     DJIMotor::s_getFeedback();

//     unsigned long timeStart;
//     unsigned long loopTimer = us_ticker_read();
//     int refLoop = 0;



//     bool prevL = false;
//     bool switL = false;
//     bool newData = true;

//     int motorSpeed = 0;

//     while(true){
//         timeStart = us_ticker_read();

//         if ((timeStart - loopTimer) / 1000 > 25){
//             loopTimer = timeStart;
//             led = !led;
//             ledbuiltin = !ledbuiltin;

//             refLoop++;
//             if (refLoop >= 5){
//                 if (referee.readable()){
//                     newData = true;
//                 }

//                 else {
//                     newData = false;
//                 }
//                 refereeThread(&referee);
//                 refLoop = 0;
//                 led2 = !led2;
  
//                 if (remote.leftSwitch() == Remote::SwitchState::MID) {

//                     float Pmax = robot_status.chassis_power_limit;

//                     double jx = remote.leftX() / 900.0; //was 660
//                     double jy = remote.leftY() / 900.0; //was 600
//                     double jr = remote.rightX() / 5000.0; //was 600

                    
//                     double tolerance = 0.05;
//                     jx = (abs(jx) < tolerance) ? 0 : jx;
//                     jy = (abs(jy) < tolerance) ? 0 : jy;
//                     jr = (abs(jr) < tolerance) ? 0 : jr;

//                     Chassis.setSpeedFF_Ks(0.065);
                    
//                     double spin = 0;

//                     double i = 0.17 * Pmax;
//                     spin  = ChassisSubsystem::rpmToRadPerSecond(i);

//                     // if (jx == 0 && jy == 0) {
//                     //     spin = ChassisSubsystem::rpmToRadPerSecond(i);
//                     // }

//                     // else {
//                     //     spin = ChassisSubsystem::rpmToRadPerSecond(i);
//                     // }



//                     float P_Theory = Chassis.setChassisSpeedsPWR(Pmax, {jx * Chassis.m_OmniKinematicsLimits.max_Vel,
//                                               jy * Chassis.m_OmniKinematicsLimits.max_Vel,
//                                               spin},
//                                               ChassisSubsystem::YAW_ORIENTED); //-jr * Chassis.m_OmniKinematicsLimits.max_vOmega
                    
//                     // printff("%d\t%.3f\n", power_heat_data.buffer_energy, P_Theory);
//                     // printff("%.2f\n", curr.read());

//                     // float chassis_power = power_heat_data.chassis_power; 
//                     // if (newData) {
//                     //     printff("P_actual: %.2f\tP_theory: %.2f\n", chassis_power, P_Theory);
//                     // }
//                 }

//                 else {
//                     Chassis.getMotor(ChassisSubsystem::LEFT_FRONT).setSpeed(0); 
//                     Chassis.getMotor(ChassisSubsystem::RIGHT_FRONT).setSpeed(0);
//                     Chassis.getMotor(ChassisSubsystem::LEFT_BACK).setSpeed(0);
//                     Chassis.getMotor(ChassisSubsystem::RIGHT_BACK).setSpeed(0);
//                     printff("off\n");
//                 }
//             }
//             remoteRead();





//             DJIMotor::s_sendValues();
//         }
//         DJIMotor::s_getFeedback();
//         ThisThread::sleep_for(1ms);
//     }
// }















#include "main.h"
// #include <jetson.h>
#include <cstdlib>

#include <iostream>
#include "subsystems/ChassisSubsystem.h"

#define PI 3.14159265

#define LOWERBOUND 35.0
#define UPPERBOUND -15.0

// add radius measurement here
#define RADIUS 0.5
#define RUNSPIN 1.0

#define JOYSTICK_SENSE_YAW 1.0/90
#define JOYSTICK_SENSE_PITCH 1.0/150
#define MOUSE_SENSE_YAW 1.0/5
#define MOUSE_SENSE_PITCH 1.0/5
#define MOUSE_KB_MULT 0.2

I2C i2c(I2C_SDA, I2C_SCL);

BNO055 imu(i2c, IMU_RESET, MODE_IMU);
ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2286); // radius is 9 in
DJIMotor yaw(7, CANHandler::CANBUS_1, GIMBLY,"Yeah");
DJIMotor pitch(7, CANHandler::CANBUS_2, GIMBLY,"Peach"); // right

DJIMotor indexer(5, CANHandler::CANBUS_2, C610,"Indexer");
DJIMotor RFLYWHEEL(2, CANHandler::CANBUS_2, M3508,"RightFly");
DJIMotor LFLYWHEEL(1, CANHandler::CANBUS_2, M3508,"LeftFly");

// DigitalOut led(L26);z
DigitalOut led2(L27);
DigitalOut led3(L25);
DigitalOut led4(LED1);
DigitalIn button(BUTTON1);
DigitalIn jumper(PC_9);

BNO055_ANGULAR_POSITION_typedef imuAngles;

int calculateDeltaYaw(int ref_yaw, int beforeBeybladeYaw)
{
    int deltaYaw = beforeBeybladeYaw - ref_yaw;

    if (abs(deltaYaw) > 180)
    {
        if (deltaYaw > 0)
            deltaYaw -= 360;
        else
            deltaYaw += 360;
    }
    return deltaYaw;
}

int main()
{

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);

    /*
    * MOTORS SETUP AND PIDS
    */

    pitch.setPositionPID(18, 0.01, 850); //12.3 0.03 2.5 //mid is 13.3, 0.03, 7.5
    pitch.setPositionIntegralCap(6000);
  //   merge difference:
//     pitch.setPositionPID(17.3, 0.03, 8.5); // 12.3 0.03 2.5 //mid is 13.3, 0.03, 7.5
//     pitch.setPositionIntegralCap(60000);
//    pitch.setPositionOutputCap(100000);
    pitch.pidPosition.feedForward = 0;
    pitch.outputCap = 32760;
    pitch.useAbsEncoder = true;


    pitch.useAbsEncoder = true;
    pitch.setPositionPID(15, 0, 1700); //15, 0 1700
    pitch.setPositionOutputCap(32000);
    float currentPitch = 0;
    float desiredPitch = 0;
    float pitch_phase = 33 / 180.0 * PI; // 5.69 theoretical
    float InitialOffset_Ticks = 4180;
    float K = 0.38; // 0.75 //0.85

    LFLYWHEEL.setSpeedPID(7.5, 0, 0.04);
    RFLYWHEEL.setSpeedPID(7.5, 0, 0.04);

    //     merge difference
    //     PID yawIMU(200.0, 0.1, 150, 20000, 8000); // 7.0,0.02,15.0,20000,8000
    Chassis.setYawReference(&yaw, 7315); // "5604" is the number of ticks of yawOne considered to be robot-front
    Chassis.setSpeedFF_Ks(0.065);
    
    yaw.setSpeedPID(225.5, 0, 200);
    PID yawBeyblade(0.32, 0, 550);
    PID yawNonBeyblade(0.15, 0, 550);

    yaw.setSpeedIntegralCap(1000);
    yaw.useAbsEncoder = false;

    indexer.setSpeedPID(1, 0, 1);
    indexer.setSpeedIntegralCap(8000);
    //PID for indexer angle position control. Surely there are better names then "sure"...
    PID sure(0.5,0,0.4);
    sure.setOutputCap(4000);
    unsigned long timeSure;
    unsigned long prevTimeSure;
    //  merge difference
    //   chassis.setBrakeMode(ChassisSubsystem::BrakeMode::COAST);

    // command.initialize();

    /*
    * OPERATIONAL VARIABLES
    */
    unsigned long loopTimer = us_ticker_read();

    int refLoop = 0;
    int ref_yaw;

    int yawSetPoint = imuAngles.yaw;
    double rotationalPower = 0;

    jumper.mode(PullDown);

    DJIMotor::s_getFeedback();
    double beybladeSpeed = 2;
    bool beybladeIncreasing = true;

    float chassis_power;
    uint16_t chassis_power_limit;
    uint16_t ref_chassis_temp1;
    uint16_t ref_chassis_temp2;
    uint16_t heatMax1;
    uint16_t heatMax2;

    int counter = 0;

    unsigned long lastTime = 0;
    unsigned long yawTime = us_ticker_read();

    //Variables & PID for burst fire
    bool shoot = false;
    //int shootPosition;
    int shootTargetPosition = 36*8190 ;
    bool shootReady = false;

    char drive = 'o';
    char shot = 'o';

    bool userButton;
    bool prev_userButton;
    char driveMode = 'j';

    int yawVelo = 0;

    while (true)
    {
        unsigned long timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > 25){
            led3 = !led3;

            loopTimer = timeStart;
            remoteRead();
            Chassis.periodic();

            if(jumper){
                driveMode = 'm';
                // led = 1;
            }else{
                driveMode = 'j';
                // led = 0;
            }
            if(remote.keyPressed(Remote::Key::R)){
                drive = 'm';
            }else if(remote.keyPressed(Remote::Key::E)){
                drive = 'u';
            }else if(remote.keyPressed(Remote::Key::Q)){
                drive = 'd';        
            }
            if(remote.keyPressed(Remote::Key::C)){
                shot = 'm';
            }else if(remote.keyPressed(Remote::Key::V)){
                shot = 'u';
            }else if(remote.keyPressed(Remote::Key::B)){
                shot = 'd';        
            }

            refLoop++;
            // imu.get_angular_position_quat(&imuAngles);
            if (refLoop >= 5){
                refereeThread(&referee);
                refLoop = 0;
                led2 = !led2;

                // printff("%d %d \n" , ext_power_heat_data.data.shooter_id1_17mm_cooling_heat, ext_game_robot_state.data.shooter_id1_17mm_cooling_limit);
            }




            float Pmax = robot_status.chassis_power_limit;

            double jx = remote.leftX() / 900.0; //was 660
            double jy = remote.leftY() / 900.0; //was 600
            double jr = remote.rightX() / 5000.0; //was 600

            
            double tolerance = 0.05;
            jx = (abs(jx) < tolerance) ? 0 : jx;
            jy = (abs(jy) < tolerance) ? 0 : jy;
            jr = (abs(jr) < tolerance) ? 0 : jr;

            Chassis.setSpeedFF_Ks(0.065);
            
            double spin = 0;

            double i = 0.80 * Pmax;
            spin  = ChassisSubsystem::rpmToRadPerSecond(i);


            currentPitch = (double(pitch.getData(ANGLE) - InitialOffset_Ticks) / TICKS_REVOLUTION) * 360; // degrees

            int leftStickValue = remote.rightY();



            /**
             * RightSwitch controls: Pitch, Yaw, Chassis
             * Up: Pitch enabled, yaw and chassis seperate
             * Mid or Unkown: Off. All power set to 0
             * Down: Pitch enabled, yaw and chassis Beyblade
             */
            int stick = remote.rightY();
            if (drive == 'u' || (drive =='o' && remote.leftSwitch() == Remote::SwitchState::UP)){          // All non-serializer motors activated
                unsigned long time = us_ticker_read();
                Chassis.setSpeedFF_Ks(0.065);
                float P_Theory = Chassis.setChassisSpeedsPWR(Pmax, {jx * Chassis.m_OmniKinematicsLimits.max_Vel,
                                          jy *
                                          Chassis.m_OmniKinematicsLimits.max_Vel,
                                          0 * Chassis.m_OmniKinematicsLimits.max_vOmega},
                                          ChassisSubsystem::YAW_ORIENTED);
                
                printff("%d\t%.3f\n", power_heat_data.buffer_energy, P_Theory);

                lastTime = time; 

                // if(driveMode == 'm'){
                //     yawSetPoint -= remote.getMouseX() * MOUSE_SENSE_YAW;
                // }else{
                //     yawSetPoint -= remote.rightX() * JOYSTICK_SENSE_YAW;
                // }
                yawVelo = -remote.rightX() * 0.1;

                int dir = 0;

                if(yawVelo > 0){
                    dir = 1;
                }else if(yawVelo < 0){
                    dir = -1;
                }
                yaw.pidSpeed.feedForward = dir * (874 + (73.7 * abs(yawVelo)) + (0.0948 * yawVelo * yawVelo));

                yaw.setSpeed(yawVelo);
                // imu.get_angular_position_quat(&imuAngles);





            } else if (drive == 'm' || (drive =='o' && remote.leftSwitch() == Remote::SwitchState::MID || remote.leftSwitch() == Remote::SwitchState::UNKNOWN)){ // disable all the non-serializer components
                Chassis.setSpeedFF_Ks(0.065);
                Chassis.setWheelPower({0,0,0,0});
                yaw.setPower(0);
                printff("off\n");



            } else if (drive == 'd' || (drive =='o' && remote.leftSwitch() == Remote::SwitchState::DOWN)){           // beyblade mode
                unsigned long time = us_ticker_read(); //time for pid
                pitch.setPower(0);
                Chassis.setSpeedFF_Ks(0.065);
                float P_Theory = Chassis.setChassisSpeedsPWR(Pmax, {jx * Chassis.m_OmniKinematicsLimits.max_Vel,
                            jy * Chassis.m_OmniKinematicsLimits.max_Vel,
                            spin},
                            ChassisSubsystem::YAW_ORIENTED);

                printff("%d\t%.3f\n", power_heat_data.buffer_energy, P_Theory);


                // yawVelo = -Chassis.getChassisSpeeds().vOmega / 3.14 * 30/* + yawBeyblade.calculatePeriodic(DJIMotor::s_calculateDeltaPhase(yawSetPoint,imuAngles.yaw+180, 360), timeSure - prevTimeSure)*/;

                yawVelo = -remote.rightX() * 0.1 - Chassis.getChassisSpeeds().vOmega / 3.14 * 30 * 3.25;

                int dir = 0;

                if(yawVelo > 0){
                    dir = 1;
                }else if(yawVelo < 0){
                    dir = -1;
                }
                yaw.pidSpeed.feedForward = dir * (874 + (73.7 * abs(yawVelo)) + (0.0948 * yawVelo * yawVelo));

                yaw.setSpeed(yawVelo);

                prevTimeSure = timeSure;
            }
            yawTime = us_ticker_read();

            DJIMotor::s_sendValues();
        }
        unsigned long timeEnd = us_ticker_read() / 1000;
        DJIMotor::s_getFeedback();
        prev_userButton = userButton;
        ThisThread::sleep_for(1ms);
        userButton = button;
    }
}











































































































