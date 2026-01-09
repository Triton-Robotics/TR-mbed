#include "main.h"
#include "subsystems/ChassisSubsystem.h"

DigitalOut led(L25);
DigitalOut led2(L26);
DigitalOut led3(L27);
DigitalOut ledbuiltin(LED1);

//CONSTANTS
constexpr float LOWERBOUND = 12.0;
constexpr float UPPERBOUND = -25.0;

constexpr float BEYBLADE_OMEGA = 2.0;

// constexpr float JOYSTICK_SENSITIVITY_YAW = 1.0/90;
// constexpr float JOYSTICK_SENSITIVITY_PITCH = 1.0/150;
// constexpr float MOUSE_SENSITIVITY_YAW = 1.0/5;
// constexpr float MOUSE_SENSITIVITY_PITCH = 1.0/5;

//DEGREES PER SECOND AT MAX
constexpr float JOYSTICK_SENSITIVITY_YAW_DPS = 180.0; 
constexpr float JOYSTICK_SENSITIVITY_PITCH_DPS = 180.0;
constexpr float MOUSE_SENSITIVITY_YAW_DPS = 1.0;
constexpr float MOUSE_SENSITIVITY_PITCH_DPS = 1.0;

constexpr int OUTER_LOOP_DT_MS = 15;

constexpr int PRINT_FREQUENCY = 20; //the higher the number, the less often

constexpr float CHASSIS_FF_KICK = 0.065;

#define USE_IMU

//CHASSIS DEFINING
I2C i2c(I2C_SDA, I2C_SCL);
BNO055 imu(i2c, IMU_RESET, MODE_IMU);

ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2794); // radius is 9 in

DJIMotor yaw(1, CANHandler::CANBUS_1, GIMBLY,"Yeah");
DJIMotor pitch(5, CANHandler::CANBUS_2, GIMBLY,"Peach"); // right

DJIMotor indexer(2, CANHandler::CANBUS_2, C610,"Indexer");
DJIMotor RFLYWHEEL(4, CANHandler::CANBUS_2, M3508,"RightFly");
DJIMotor LFLYWHEEL(1, CANHandler::CANBUS_2, M3508,"LeftFly");
DJIMotor feeder(5, CANHandler::CANBUS_2, C610);

#ifdef USE_IMU
BNO055_ANGULAR_POSITION_typedef imuAngles;
#endif

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

int main(){

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();
    usbSerial.set_blocking(false);

    /*
    * MOTORS SETUP AND PIDS
    */
    //YAW
    PID yawBeyblade(1,0.005,150);
    yawBeyblade.setIntegralCap(2);
    //PID yawBeyblade(1.5, 0, 550); //yaw PID is cascading, so there are external position PIDs for yaw control
    // PID yawNonBeyblade(0.15, 0, 550);
    yaw.setSpeedPID(380,0,0);
    yaw.setSpeedIntegralCap(8000);
    yaw.setSpeedOutputCap(32000);
    //yaw.setSpeedPID(50, 0.2, 300); // tried setting P to 37.5 same as infantry yaw PID
    yaw.outputCap = 16000;
    yaw.useAbsEncoder = false;

    int yawVelo = 0;
    #ifdef USE_IMU
    // imu.get_angular_position_quat(&imuAngles);
    while(imu.chip_ready()){

    }
    imu.get_euler_angles((BNO055_EULER_TypeDef*)&imuAngles);
   
    imuAngles.yaw = 180 - imuAngles.yaw;
    imuAngles.pitch = 180 - imuAngles.pitch;
    imuAngles.roll = 180 - imuAngles.roll;
    float yaw_desired_angle = imuAngles.yaw + 180;
    #else
    float yaw_desired_angle = (yaw>>ANGLE) * 360.0 / TICKS_REVOLUTION;
    float yaw_current_angle = (yaw>>ANGLE) * 360.0 / TICKS_REVOLUTION;
    #endif

    //PITCH
    pitch.setPositionPID(8, 0, 0); //15, 0, 1700
    pitch.setPositionOutputCap(32000);
    pitch.pidPosition.feedForward = 0;
    pitch.outputCap = 16000;
    pitch.useAbsEncoder = true;

    float pitch_current_angle = 0;
    float pitch_desired_angle = 0;
    float pitch_phase_angle = 33 / 180.0 * PI; // 5.69 theoretical //wtf is this?
    float pitch_zero_offset_ticks = 2000;
    float K = 0.38; // 0.75 //0.85

    //FLYWHEELS
    LFLYWHEEL.setSpeedPID(7.1849, 0.000042634, 0);
    RFLYWHEEL.setSpeedPID(7.1849, 0.000042634, 0);

    feeder.setSpeedPID(4, 0, 1);

    //INDEXER
    indexer.setSpeedPID(1.65, 0, 1);
    indexer.setSpeedIntegralCap(8000);
    //Cascading PID for indexer angle position control. Surely there are better names then "sure"...
    PID sure(0.5,0,0.4);
    sure.setOutputCap(4000);
    //Variables for burst fire
    unsigned long timeSure;
    unsigned long prevTimeSure;
    unsigned long shootTimer;
    bool shoot = false;
    int shootTargetPosition = 36*8190 ;
    bool shootReady = false;

    //CHASSIS
    Chassis.setYawReference(&yaw, 2500); //the number of ticks of yaw considered to be robot-front
    //Common values for reference are 6500 and 2500
    Chassis.setSpeedFF_Ks(CHASSIS_FF_KICK); //feed forward "kick" for wheels, a constant multiplier of max power in the direcion of movment

    //GENERAL VARIABLES

    //drive and shooting mode
    char drive = 'o'; //default o when using joystick
    char shot = 'o'; //default o when using joystick
    char driveMode = 'j'; //j for joystick, m for mouse/keyboard

    //user button (doesnt work?)
    bool userButton;
    bool prev_userButton;

    //ref variables
    uint16_t chassis_buffer;
    uint16_t chassis_power_limit;
    uint16_t barrel_heat1;
    uint16_t barrel_heat_max1;

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    int refLoop = 0;
    int printLoop = 0;

    ChassisSpeeds cs;

    while(true){
        timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > OUTER_LOOP_DT_MS){
            float elapsedms = (timeStart - loopTimer) / 1000;
            loopTimer = timeStart;
            led = !led;
            refLoop++;
            if (refLoop >= 5){
                refereeThread(&referee);
                refLoop = 0;
                led2 = !led2;

                //POWER LIMIT OVERRIDE INCASE
                if(robot_status.chassis_power_limit < 10){
                    chassis_power_limit = 50;
                }else{
                    chassis_power_limit = robot_status.chassis_power_limit;
                }
                
            }
            Chassis.periodic();
            cs = Chassis.getChassisSpeeds();

            Remote::SwitchState previous_mode = remote.leftSwitch();
            bool prevM = remote.getMouseL();
            remoteRead();

            #ifdef USE_IMU
            //imu.get_angular_position_quat(&imuAngles);
            imu.get_euler_angles((BNO055_EULER_TypeDef*)&imuAngles);
            imuAngles.yaw = 180 - imuAngles.yaw;
            imuAngles.pitch = 180 - imuAngles.pitch;
            imuAngles.roll = 180 - imuAngles.roll;
            #else
            yaw_current_angle = (yaw>>ANGLE) * 360.0 / TICKS_REVOLUTION;
            #endif

            //Keyboard-based drive and shoot mode
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

            //Driving input
            float scalar = 1;
            float jx = remote.leftX() / 660.0 * scalar; // -1 to 1
            float jy = remote.leftY() / 660.0 * scalar; // -1 to 1
            //Pitch, Yaw
            float jpitch = remote.rightY() / 660.0 * scalar; // -1 to 1
            float jyaw = remote.rightX() / 660.0 * scalar; // -1 to 1

            //joystick tolerance
            float tolerance = 0.05; 
            jx = (abs(jx) < tolerance) ? 0 : jx;
            jy = (abs(jy) < tolerance) ? 0 : jy;
            jpitch = (abs(jpitch) < tolerance) ? 0 : jpitch;
            jyaw = (abs(jyaw) < tolerance) ? 0 : jyaw;
            
            //Keyboard Driving
            float mult = 1;
            jx += mult * ((remote.keyPressed(Remote::Key::D) ? 1 : 0) + (remote.keyPressed(Remote::Key::A) ? -1 : 0));
            jy += mult * ((remote.keyPressed(Remote::Key::W) ? 1 : 0) + (remote.keyPressed(Remote::Key::S) ? -1 : 0));
            
            //Bounding the four j variables
            jx = max(-1.0F, min(1.0F, jx));
            jy = max(-1.0F, min(1.0F, jy));
            jpitch = max(-1.0F, min(1.0F, jpitch));
            jyaw = max(-1.0F, min(1.0F, jyaw));

            //Chassis Code
            if (drive == 'u' || (drive =='o' && remote.rightSwitch() == Remote::SwitchState::UP)){
                //REGULAR DRIVING CODE
                Chassis.setChassisSpeeds({jx * Chassis.m_OmniKinematicsLimits.max_Vel,
                                          jy * Chassis.m_OmniKinematicsLimits.max_Vel,
                                          0 * Chassis.m_OmniKinematicsLimits.max_vOmega},
                                          ChassisSubsystem::YAW_ORIENTED);
            }else if (drive == 'd' || (drive =='o' && remote.rightSwitch() == Remote::SwitchState::DOWN)){
                //BEYBLADE DRIVING CODE
                Chassis.setChassisSpeeds({jx * Chassis.m_OmniKinematicsLimits.max_Vel,
                                          jy * Chassis.m_OmniKinematicsLimits.max_Vel,
                                          -BEYBLADE_OMEGA},
                                          ChassisSubsystem::YAW_ORIENTED);
            }else{
                //OFF
                Chassis.setWheelPower({0,0,0,0});
            }

            //YAW CODE
            if (drive == 'u' || drive == 'd' || (drive =='o' && (remote.rightSwitch() == Remote::SwitchState::UP || remote.rightSwitch() == Remote::SwitchState::DOWN))){
                float chassis_rotation_radps = cs.vOmega;
                int chassis_rotation_rpm = chassis_rotation_radps * 60 / (2*M_PI) * 1.5; //I added this 4 but I don't know why.

                //Regular Yaw Code
                yaw_desired_angle -= jyaw * MOUSE_SENSITIVITY_YAW_DPS * elapsedms / 1000;
                yaw_desired_angle -= jyaw * JOYSTICK_SENSITIVITY_YAW_DPS * elapsedms / 1000;
                //yaw_desired_angle = (yaw_desired_angle + 360) % 360;
                yaw_desired_angle = floatmod(yaw_desired_angle, 360);

                prevTimeSure = timeSure;
                timeSure = us_ticker_read();
                #ifdef USE_IMU
                yawVelo = yawBeyblade.calculatePeriodic(DJIMotor::s_calculateDeltaPhase(yaw_desired_angle, imuAngles.yaw + 180, 360), timeSure - prevTimeSure);
                #else
                yawVelo = -jyaw * JOYSTICK_SENSITIVITY_YAW_DPS / 360.0 * 60;
                //yawVelo = yawBeyblade.calculatePeriodic(DJIMotor::s_calculateDeltaPhase(yaw_desired_angle, yaw_current_angle, 360), timeSure - prevTimeSure);
                #endif
                yawVelo -= chassis_rotation_rpm;
                //yawVelo *= 6; // scaled up arbitrarily 

                int dir = 0;
                if(yawVelo > 1){
                    dir = 1;
                }else if(yawVelo < -1){
                    dir = -1;
                }
                yaw.pidSpeed.feedForward = dir * (1855 + abs(yawVelo) * 120.48);
                yaw.setSpeed(yawVelo);
            }else{
                //Off
                yaw.setPower(0);
                #ifdef USE_IMU
                yaw_desired_angle = imuAngles.yaw + 180;
                #endif
            }

            //PITCH
            if (drive == 'u' || drive == 'd' || (drive =='o' && (remote.rightSwitch() == Remote::SwitchState::UP || remote.rightSwitch() == Remote::SwitchState::DOWN))){
                //Regular Pitch Code
                pitch_desired_angle += -jpitch * MOUSE_SENSITIVITY_PITCH_DPS * elapsedms / 1000;
                pitch_desired_angle -= -jpitch * JOYSTICK_SENSITIVITY_PITCH_DPS * elapsedms / 1000;

                if (pitch_desired_angle >= LOWERBOUND) {
                    pitch_desired_angle = LOWERBOUND;
                }
                else if (pitch_desired_angle <= UPPERBOUND) {
                    pitch_desired_angle = UPPERBOUND;
                }

                //float FF = K * sin((desiredPitch / 180 * PI) - pitch_phase); // output: [-1,1]
                //float FF = K * cos(pitch_desired_angle / 180 * PI);
                //pitch.pidPosition.feedForward = int((INT16_T_MAX) * FF);
                pitch.setPosition(int((pitch_desired_angle / 60) * TICKS_REVOLUTION + pitch_zero_offset_ticks));
            }else{
                //Off
                pitch.setPower(0);
            }

            //INDEXER CODE
            if((previous_mode == Remote::SwitchState::MID && remote.leftSwitch() == Remote::SwitchState::UP) || (!prevM && remote.getMouseL())){
                if(robot_status.shooter_barrel_heat_limit < 10 || power_heat_data.shooter_42mm_barrel_heat < robot_status.shooter_barrel_heat_limit - 110) {
                    shootTimer = us_ticker_read()/1000;
                }
            }

            if (us_ticker_read()/1000 - shootTimer < 200){
                feeder.setSpeed(7000);
            } else {
                feeder.setSpeed(0);
            }
            if (us_ticker_read()/1000 - shootTimer < 500){
                // indexer.setSpeed(8000);
                indexer.setSpeed(6000);
            }else if (us_ticker_read()/1000 - shootTimer > 500 && us_ticker_read()/1000 - shootTimer < 650){
                // indexer.setSpeed(8000);
                indexer.setPower(-16000);
            } else {
                indexer.setPower(0);
                // indexerOn = true;
            }
            // burst fire, turn the indexer to shoot 3-5 balls a time and stop indexer
            // only shoot when left switch changes from down/unknown/mid to up
            // if left switch remains at up state, indexer stops after 3-5 balls
//             if (shoot){
//                     //                 if (indexer>>MULTITURNANGLE >= shootTargetPosition){
//                     //                     // indexer.setSpeed(0);
//                     //                     shoot = false;
//                     //                 } else {
//                     //                     timeSure = us_ticker_read();
//                     //                     // indexer.setSpeed(0); //
//                     //                     // prevTimeSure = timeSure;
//                     //                 }
//                     //feeder
//                     bool feederOn = false;
//                     bool indexerOn = false;

//                     if (us_ticker_read()/1000 - shootTimer < 200){
//                         feeder.setSpeed(7000);
//                     } else {
//                         feeder.setSpeed(0);
//                         feederOn = true;
//                     }
//                     if (us_ticker_read()/1000 - shootTimer > 500 && us_ticker_read()/1000 - shootTimer < 650){
//                         // indexer.setSpeed(8000);
//                         indexer.setPower(-16000);
//                     } else {
//                         indexer.setSpeed(6000);
//                         // indexerOn = true;
//                     }
//                     if (feederOn){
//                         shoot = false;
//                     }

                    
//                     // comment out once the pitch data is settled

//                     // if (pitch is elevated) {
//                     //     //indexer
//                     //     if (us_ticker_read()/1000 - shootTimer < 300){
//                     //         indexer.setSpeed(8000);
//                     //     } else {
//                     //         indexer.setSpeed(350);
//                     //         indexerOn = true;
//                     //     }
//                     //     if (indexerOn && feederOn){
//                     //         shoot = false;
//                     //     }
//                     // } else { // standard values that work when the pitch is level
//                     //     //indexer
//                     //     if (us_ticker_read()/1000 - shootTimer < 300){
//                     //         indexer.setSpeed(8000);
//                     //     } else {
//                     //         indexer.setSpeed(350);
//                     //         indexerOn = true;
//                     //     }
//                     //     if (indexerOn && feederOn){
//                     //         shoot = false;
//                     //     }
//                     // }

//                 } else {
//                 // indexer.setSpeed(200);
// //                 feeder.setSpeed(0);
//                 feeder.setPower(0);
//                 }

            //FLYWHEELS
            if (remote.leftSwitch() != Remote::SwitchState::DOWN &&
                remote.leftSwitch() != Remote::SwitchState::UNKNOWN &&
                remote.rightSwitch() != Remote::SwitchState::MID){
                RFLYWHEEL.setSpeed(-7475);
                LFLYWHEEL.setSpeed(7475);
            } else{
                // left SwitchState set to up/mid/unknown
                RFLYWHEEL.setSpeed(0);
                LFLYWHEEL.setSpeed(0);
            }

            printLoop ++;
            if (printLoop >= PRINT_FREQUENCY){
                printLoop = 0;
                //printff("%.3f Pitch\n", pitch_desired_angle);
                //printff("Prints:\n");
                //printff("lX:%.1f lY:%.1f rX:%.1f rY:%.1f lS:%d rS:%d\n", remote.leftX(), remote.leftY(), remote.rightX(), remote.rightY(), remote.leftSwitch(), remote.rightSwitch());
                //printff("jx:%.3f jy:%.3f jpitch:%.3f jyaw:%.3f\n", jx, jy, jpitch, jyaw);

                //printff("%.3f  %d\n", pitch_desired_angle, pitch.getData(ANGLE));
                //printff("%d\n", indexer.getData(POWEROUT));

                #ifdef USE_IMU
                //printff("yaw_des_v:%d yaw_act_v:%d", yawVelo, yaw>>VELOCITY);
                // printff("yaw_des:%.3f yaw_act:%.3f\n", yaw_desired_angle, imuAngles.yaw + 180);
                #else
                // printff("yaw_des_v:%d yaw_act_v:%d\n", yawVelo, yaw>>VELOCITY);
                //printff("yaw_des:%.3f yaw_act:%.3f [%d]\n", yaw_desired_angle, yaw_current_angle, yaw>>ANGLE);
                #endif
                // printff("elap:%.5fms\n", elapsedms);
                // printff("Chassis: LF:%c RF:%c LB:%c RB:%c\n", 
                //     Chassis.getMotor(ChassisSubsystem::LEFT_FRONT).isConnected() ? 'y' : 'n', 
                //     Chassis.getMotor(ChassisSubsystem::RIGHT_FRONT).isConnected() ? 'y' : 'n', 
                //     Chassis.getMotor(ChassisSubsystem::LEFT_BACK).isConnected() ? 'y' : 'n', 
                //     Chassis.getMotor(ChassisSubsystem::RIGHT_BACK).isConnected() ? 'y' : 'n');
                // printff("Y:%c P:%c F_L:%c F_R:%c I:%c F:%c\n",
                //     yaw.isConnected() ? 'y' : 'n', 
                //     pitch.isConnected() ? 'y' : 'n', 
                //     LFLYWHEEL.isConnected() ? 'y' : 'n', 
                //     RFLYWHEEL.isConnected() ? 'y' : 'n',
                //     indexer.isConnected() ? 'y' : 'n',
                //     feeder.isConnected() ? 'y' : 'n');
                #ifdef USE_IMU
                //printff("IMU %.3f %.3f %.3f\n",imuAngles.yaw, imuAngles.pitch, imuAngles.roll);
                #endif

                WheelSpeeds ac = Chassis.getWheelSpeeds();
                ChassisSpeeds test = {jx * Chassis.m_OmniKinematicsLimits.max_Vel,
                                    jy * Chassis.m_OmniKinematicsLimits.max_Vel,
                                    -BEYBLADE_OMEGA};
                WheelSpeeds ws = Chassis.chassisSpeedsToWheelSpeeds(test);
                
                // printff("CS: %.1f %.1f %.1f ", cs.vX, cs.vY, cs.vOmega);
                // printff("DS: %.1f %.1f %.1f\n", test.vX, test.vY, test.vOmega);

                // printff("CH: %.2f %.2f %.2f %.2f ", ws.LF,ws.RF,ws.LB,ws.RB);
                // printff("A: %.2f %.2f %.2f %.2f\n", ac.LF,ac.RF,ac.LB,ac.RB);
                // printff("A_RAW: %d %d %d %d\n", 
                //     Chassis.getMotor(ChassisSubsystem::LEFT_FRONT).getData(VELOCITY), 
                //     Chassis.getMotor(ChassisSubsystem::RIGHT_FRONT).getData(VELOCITY), 
                //     Chassis.getMotor(ChassisSubsystem::LEFT_BACK).getData(VELOCITY), 
                //     Chassis.getMotor(ChassisSubsystem::RIGHT_BACK).getData(VELOCITY));
                // printff("A_MPS: %.2f %.2f %.2f %.2f\n", 
                //     Chassis.getMotorSpeed(ChassisSubsystem::LEFT_FRONT, ChassisSubsystem::METER_PER_SECOND), 
                //     Chassis.getMotorSpeed(ChassisSubsystem::RIGHT_FRONT, ChassisSubsystem::METER_PER_SECOND), 
                //     Chassis.getMotorSpeed(ChassisSubsystem::LEFT_BACK, ChassisSubsystem::METER_PER_SECOND), 
                //     Chassis.getMotorSpeed(ChassisSubsystem::RIGHT_BACK, ChassisSubsystem::METER_PER_SECOND));
            }

            DJIMotor::s_sendValues();

            // put PID TOOLS HERE


        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}