#include "main.h"
#include "subsystems/ChassisSubsystem.h"

DigitalOut led(L25);
DigitalOut led2(L26);
DigitalOut led3(L27);
DigitalOut ledbuiltin(LED1);

//CONSTANTS
constexpr float LOWERBOUND = 35.0;
constexpr float UPPERBOUND = -15.0;

constexpr float BEYBLADE_OMEGA = 4.0;

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

constexpr int FLYWHEEL_SPEED = 7000;

//#define USE_IMU

//CHASSIS DEFINING
I2C i2c(I2C_SDA, I2C_SCL);

BNO055 imu(i2c, IMU_RESET, MODE_IMU);
ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2286); // radius is 9 in
DJIMotor yaw(6, CANHandler::CANBUS_1, GIMBLY,"Yeah");
DJIMotor pitch(5, CANHandler::CANBUS_2, GIMBLY,"Peach"); // right
DJIMotor yaw2(7, CANHandler::CANBUS_1, GIMBLY,"Ye2"); // left, not functioning

DJIMotor indexerL(5, CANHandler::CANBUS_2, C610,"IndexerL");
DJIMotor indexerR(6, CANHandler::CANBUS_2, C610,"IndexerR");
DJIMotor RFLYWHEEL_U(1, CANHandler::CANBUS_2, M3508,"RightFlyU");
DJIMotor LFLYWHEEL_U(2, CANHandler::CANBUS_2, M3508,"LeftFlyU");
DJIMotor RFLYWHEEL_D(3, CANHandler::CANBUS_2, M3508,"RightFlyD");
DJIMotor LFLYWHEEL_D(4, CANHandler::CANBUS_2, M3508,"LeftFlyD");

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

    /*
    * MOTORS SETUP AND PIDS
    */
    //YAW
    PID yawBeyblade(5.0, 0, 150); //yaw PID is cascading, so there are external position PIDs for yaw control
    yaw.setSpeedPID(537.48, 0.48305, 0);
    yaw.setSpeedIntegralCap(5000);
    yaw.setSpeedOutputCap(32000);
    yaw.outputCap = 32000;
    yaw.useAbsEncoder = false;

    int yawVelo = 0;
    #ifdef USE_IMU
    imu.get_angular_position_quat(&imuAngles);
    float yaw_desired_angle = imuAngles.yaw + 180;
    #else
    float yaw_desired_angle = (yaw>>ANGLE) * 360.0 / TICKS_REVOLUTION;
    float yaw_current_angle = (yaw>>ANGLE) * 360.0 / TICKS_REVOLUTION;
    #endif

    //PITCH
    pitch.setPositionPID(23.0458, 0.022697, 756.5322);
    pitch.setPositionOutputCap(32000);
    pitch.pidPosition.feedForward = 0;
    pitch.outputCap = 16000;
    pitch.useAbsEncoder = true;

    float pitch_current_angle = 0;
    float pitch_desired_angle = 0;
    float pitch_phase_angle = 33 / 180.0 * PI; // 5.69 theoretical //wtf is this?
    float pitch_zero_offset_ticks = 6500;
    float K = 0.38; // 0.75 //0.85

    //FLYWHEELS
    LFLYWHEEL_U.setSpeedPID(7.1849, 0.000042634, 0);
    RFLYWHEEL_U.setSpeedPID(7.1849, 0.000042634, 0);
    LFLYWHEEL_D.setSpeedPID(7.1849, 0.000042634, 0);
    RFLYWHEEL_D.setSpeedPID(7.1849, 0.000042634, 0);

    //INDEXER
    indexerL.setSpeedPID(1, 0, 1);
    indexerL.setSpeedIntegralCap(8000);
    indexerR.setSpeedPID(1, 0, 1);
    indexerR.setSpeedIntegralCap(8000);
    //Cascading PID for indexer angle position control. Surely there are better names then "sure"...
    // PID sure(0.5,0,0.4);
    // sure.setOutputCap(4000);
    //Variables for burst fire
    unsigned long timeSure;
    unsigned long prevTimeSure;
    // bool shoot = false;
    // int shootTargetPosition = 36*8190 ;
    // bool shootReady = false;

    //CHASSIS
    Chassis.setYawReference(&yaw, 4096); //the number of ticks of yaw considered to be robot-front
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
    float chassis_power;
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
                led3 = referee.readable();
                refereeThread(&referee);
                refLoop = 0;
                led2 = !led2;

                //POWER LIMIT OVERRIDE INCASE
                if(robot_status.chassis_power_limit < 10){
                    chassis_power_limit = 50;
                }else{
                    chassis_power_limit = robot_status.chassis_power_limit;
                }
                
                Chassis.power_limit = (float)chassis_power_limit;
                chassis_power = power_heat_data.chassis_power;
            }
            Chassis.periodic();
            cs = Chassis.getChassisSpeeds();
            remoteRead();

            #ifdef USE_IMU
            imu.get_angular_position_quat(&imuAngles);
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
            float tolerance = 0.1; 
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
                                          ChassisSubsystem::REVERSE_YAW_ORIENTED);
            }else if (drive == 'd' || (drive =='o' && remote.rightSwitch() == Remote::SwitchState::DOWN)){
                //BEYBLADE DRIVING CODE
                Chassis.setChassisSpeeds({jx * Chassis.m_OmniKinematicsLimits.max_Vel,
                                          jy * Chassis.m_OmniKinematicsLimits.max_Vel,
                                          -BEYBLADE_OMEGA},
                                          ChassisSubsystem::REVERSE_YAW_ORIENTED);
            }else{
                //OFF
                Chassis.setWheelPower({0,0,0,0});
            }

            //YAW CODE
            if (drive == 'u' || drive == 'd' || (drive =='o' && (remote.rightSwitch() == Remote::SwitchState::UP || remote.rightSwitch() == Remote::SwitchState::DOWN))){
                float chassis_rotation_radps = cs.vOmega;
                int chassis_rotation_rpm = chassis_rotation_radps * 60 / (2*M_PI) * 4; //I added this 4 but I don't know why.
                
                //Regular Yaw Code
                yaw_desired_angle -= jyaw * MOUSE_SENSITIVITY_YAW_DPS * elapsedms / 1000;
                yaw_desired_angle -= jyaw * JOYSTICK_SENSITIVITY_YAW_DPS * elapsedms / 1000;
                //yaw_desired_angle = (yaw_desired_angle + 360) % 360;
                yaw_desired_angle = floatmod(yaw_desired_angle, 360);

                #ifdef USE_IMU
                yawVelo = yawBeyblade.calculatePeriodic(DJIMotor::s_calculateDeltaPhase(yaw_desired_angle, imuAngles.yaw + 180, 360), timeSure - prevTimeSure);
                #else
                yawVelo = -jyaw * JOYSTICK_SENSITIVITY_YAW_DPS / 360.0 * 60;
                #endif
                //yawVelo = 0;
                yawVelo -= chassis_rotation_rpm;

                int dir = 0;
                if(yawVelo > 0){
                    dir = 1;
                }else if(yawVelo < 0){
                    dir = -1;
                }
                //yaw.pidSpeed.feedForward = -5.30094881524873 * yawVelo * yawVelo * dir + 461.129143101395 * yawVelo + 2402.35249010233 * dir;
                yaw.pidSpeed.feedForward = 4 * dir * (874 + (73.7 * abs(yawVelo)) + (0.0948 * yawVelo * yawVelo));
                yaw.setSpeed(-yawVelo);

                prevTimeSure = timeSure;
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
                pitch_desired_angle += jpitch * MOUSE_SENSITIVITY_PITCH_DPS * elapsedms / 1000;
                pitch_desired_angle -= jpitch * JOYSTICK_SENSITIVITY_PITCH_DPS * elapsedms / 1000;

                if (pitch_desired_angle >= LOWERBOUND) {
                    pitch_desired_angle = LOWERBOUND;
                }
                else if (pitch_desired_angle <= UPPERBOUND) {
                    pitch_desired_angle = UPPERBOUND;
                }

                //float FF = K * sin((desiredPitch / 180 * PI) - pitch_phase); // output: [-1,1]
                //float FF = K * cos(pitch_desired_angle / 180 * PI);
                //pitch.pidPosition.feedForward = int((INT16_T_MAX) * FF);
                pitch.setPosition(int((pitch_desired_angle / 360) * TICKS_REVOLUTION + pitch_zero_offset_ticks));
            }else{
                //Off
                pitch.setPower(0);
            }
            pitch_current_angle = ((pitch>>ANGLE) - pitch_zero_offset_ticks) / TICKS_REVOLUTION * 360;

            //INDEXER CODE
            if (remote.leftSwitch() == Remote::SwitchState::UP || remote.getMouseL()){
                // if (shootReady){
                //     shootReady = false;
                //     shootTargetPosition = 8192 * 12 + (indexer>>MULTITURNANGLE);

                //     //shoot limit
                //     if(robot_status.shooter_barrel_heat_limit < 10 || power_heat_data.shooter_17mm_1_barrel_heat < robot_status.shooter_barrel_heat_limit - 40) {
                //         shoot = true;
                //     }
                    
                // }

                indexerL.setSpeed(-5 * 8 * M2006_GEAR_RATIO);
                indexerR.setSpeed(5 * 8 * M2006_GEAR_RATIO);
            } else {
                indexerL.setSpeed(0);
                indexerR.setSpeed(0);
                //SwitchState state set to mid/down/unknown
                //shootReady = true;
            }

            //FLYWHEELS
            if (remote.leftSwitch() != Remote::SwitchState::DOWN &&
                remote.leftSwitch() != Remote::SwitchState::UNKNOWN){
                RFLYWHEEL_U.setSpeed(-FLYWHEEL_SPEED);
                LFLYWHEEL_U.setSpeed(FLYWHEEL_SPEED);
                RFLYWHEEL_D.setSpeed(-FLYWHEEL_SPEED);
                LFLYWHEEL_D.setSpeed(FLYWHEEL_SPEED);
            } else{
                // left SwitchState set to up/mid/unknown
                RFLYWHEEL_U.setSpeed(0);
                LFLYWHEEL_U.setSpeed(0);
                RFLYWHEEL_D.setSpeed(0);
                LFLYWHEEL_D.setSpeed(0);
            }

            printLoop ++;
            if (printLoop >= PRINT_FREQUENCY){
                printLoop = 0;
                //printff("Prints:\n");
                //printff("lX:%.1f lY:%.1f rX:%.1f rY:%.1f lS:%d rS:%d\n", remote.leftX(), remote.leftY(), remote.rightX(), remote.rightY(), remote.leftSwitch(), remote.rightSwitch());
                //printff("jx:%.3f jy:%.3f jpitch:%.3f jyaw:%.3f\n", jx, jy, jpitch, jyaw);
                #ifdef USE_IMU
                printff("yaw_des_v:%d yaw_act_v:%d", yawVelo, yaw>>VELOCITY);
                printff("yaw_des:%.3f yaw_act:%.3f\n", yaw_desired_angle, imuAngles.yaw + 180);
                #else
                printff("yaw_des_v:%d yaw_act_v:%d PWR:%d\n", yawVelo, yaw>>VELOCITY, yaw>>POWEROUT);
                //printff("yaw_des:%.3f yaw_act:%.3f [%d]\n", yaw_desired_angle, yaw_current_angle, yaw>>ANGLE);
                #endif
                //printff("pitch_des_v:%d yaw_act_v:%d", yawVelo, yaw>>VELOCITY);
                //printff("pitch_des:%.3f pitch_act:%.3f [%d]\n", pitch_desired_angle, pitch_current_angle, pitch>>ANGLE);
                //printff("cX%.1f cY%.1f cOmega%.3f cRPM%.1f\n", cs.vX, cs.vY, cs.vOmega, cs.vOmega * 60 / (2*M_PI) * 4);
                // printff("Chassis: LF:%c RF:%c LB:%c RB:%c Yaw:%c Pitch:%c Flywheel_L:%c Flywheel_R:%c Indexer:%c\n", 
                //     Chassis.getMotor(ChassisSubsystem::LEFT_FRONT).isConnected() ? 'y' : 'n', 
                //     Chassis.getMotor(ChassisSubsystem::RIGHT_FRONT).isConnected() ? 'y' : 'n', 
                //     Chassis.getMotor(ChassisSubsystem::LEFT_BACK).isConnected() ? 'y' : 'n', 
                //     Chassis.getMotor(ChassisSubsystem::RIGHT_BACK).isConnected() ? 'y' : 'n',
                //     yaw.isConnected() ? 'y' : 'n', 
                //     pitch.isConnected() ? 'y' : 'n');

                // printff("Flywheels: LU:%c LD:%c RU:%c RD:%c IndexerL:%c IndexerR:%c\n", 
                //     LFLYWHEEL_U.isConnected() ? 'y' : 'n', 
                //     LFLYWHEEL_D.isConnected() ? 'y' : 'n', 
                //     RFLYWHEEL_U.isConnected() ? 'y' : 'n', 
                //     RFLYWHEEL_D.isConnected() ? 'y' : 'n', 
                //     indexerL.isConnected() ? 'y' : 'n',
                //     indexerR.isConnected() ? 'y' : 'n');
                //printff("%d %d %d\n", Chassis.getMotor(ChassisSubsystem::LEFT_FRONT)>>VELOCITY,  Chassis.getMotor(ChassisSubsystem::LEFT_FRONT)>>VALUE, Chassis.getMotor(ChassisSubsystem::LEFT_FRONT).getMode());
                #ifdef USE_IMU
                //printff("IMU %.3f %.3f %.3f\n",imuAngles.yaw, imuAngles.pitch, imuAngles.roll);
                #endif
                //printff("pwr:%.2f max:%d\n", chassis_power, chassis_power_limit);
                //printff("ID:%d LVL:%d HP:%d MAX_HP:%d\n", robot_status.robot_id, robot_status.robot_level, robot_status.current_HP, robot_status.maximum_HP);
                //printff("elap:%.5fms\n", elapsedms);
            }

            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}