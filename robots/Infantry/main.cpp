#include "main.h"
#include "subsystems/ChassisSubsystem.h"

DigitalOut led(L25);
DigitalOut led2(L26);
DigitalOut led3(L27);
DigitalOut ledbuiltin(LED1);

//CONSTANTS
constexpr float LOWERBOUND = -35.0;
constexpr float UPPERBOUND = 40.0;

//DEGREES PER SECOND AT MAX
constexpr float JOYSTICK_SENSITIVITY_YAW_DPS = 180.0; 
constexpr float JOYSTICK_SENSITIVITY_PITCH_DPS = 180.0;

// Mouse sensitivity initialized
constexpr float MOUSE_SENSITIVITY_YAW_DPS = 10.0;
constexpr float MOUSE_SENSITIVITY_PITCH_DPS = 10.0;

constexpr int OUTER_LOOP_DT_MS = 15;

constexpr int PRINT_FREQUENCY = 8; //the higher the number, the less often

constexpr float CHASSIS_FF_KICK = 0.065;

constexpr float pitch_zero_offset_ticks = 1500;

constexpr int NUM_BALLS_SHOT = 3;
constexpr int FLYWHEEL_VELO = 5500;

#define READ_DEBUG 0
#define MAGICBYTE 0xEE
#define USE_IMU

//CHASSIS DEFINING
I2C i2c(I2C_SDA, I2C_SCL);
BNO055 imu(i2c, IMU_RESET, MODE_IMU);
ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.22617); // radius is 9 in
DJIMotor yaw(4, CANHandler::CANBUS_1, GIMBLY,"Yeah");
DJIMotor pitch(7, CANHandler::CANBUS_2, GIMBLY,"Peach"); // right

DJIMotor indexer(7, CANHandler::CANBUS_2, C610,"Indexer");
DJIMotor RFLYWHEEL(1, CANHandler::CANBUS_2, M3508,"RightFly");
DJIMotor LFLYWHEEL(2, CANHandler::CANBUS_2, M3508,"LeftFly");

//CV STUFF
static BufferedSerial bcJetson(PC_12, PD_2, 115200);  //JETSON PORT



#ifdef USE_IMU
BNO055_ANGULAR_POSITION_typedef imuAngles;
#endif


int main(){
    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();
    usbSerial.set_blocking(false);

    bcJetson.set_blocking(false);

    /*
    * MOTORS SETUP AND PIDS
    */
    //YAW
    yaw.setSpeedPID(708.1461, 4.721, 2.6555);
    yaw.setSpeedIntegralCap(8000);
    yaw.setSpeedOutputCap(32000);

    yaw.setPositionPID(1.18, 0, 0);
    yaw.pidPosition.dBuffer.lastY = 5;
    yaw.pidPosition.setIntegralCap(2);
    yaw.pidPosition.setOutputCap(90);

    yaw.outputCap = 16000;
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
    // pitch.setPositionPID(26.2644, 0.034926, 1200); //15, 0 1700
    // pitch.setPositionOutputCap(32000);
    // pitch.setPositionIntegralCap(3000);
    PID pitchCascade(1.5,0.0005,0.05);
    pitchCascade.setIntegralCap(2);
    pitchCascade.setOutputCap(30);
    pitch.setSpeedPID(500,0.8,0);
    pitch.setSpeedIntegralCap(2000);
    pitch.setSpeedOutputCap(32000);
    pitch.pidPosition.feedForward = 0;
    pitch.outputCap = 16000;
    pitch.useAbsEncoder = true;
    pitchCascade.dBuffer.lastY = 5;

    int pitchVelo = 0;

    float pitch_current_angle = 0;
    float pitch_desired_angle = 0;

    //FLYWHEELS
    LFLYWHEEL.setSpeedPID(7.1849, 0.000042634, 0);
    RFLYWHEEL.setSpeedPID(7.1849, 0.000042634, 0);

    //INDEXER
    indexer.setSpeedPID(2.7, 0.001, 0);
    indexer.setSpeedIntegralCap(100);
    //Cascading PID for indexer angle position control. Surely there are better names then "sure"...
    PID sure(0.1,0,0.001);
    sure.setOutputCap(133 * M2006_GEAR_RATIO);
    sure.dBuffer = 10;
    
    //Variables for burst fire
    unsigned long timeSure;
    unsigned long prevTimeSure;
    bool shoot = false;
    int shootTargetPosition = 36*8190 ;
    bool shootReady = false;

    //CHASSIS
    Chassis.setYawReference(&yaw, 6500); //the number of ticks of yaw considered to be robot-front
    //Common values for reference are 6500 and 2500
    Chassis.setSpeedFF_Ks(CHASSIS_FF_KICK); //feed forward "kick" for wheels, a constant multiplier of max power in the direcion of movment

    //GENERAL VARIABLES
    //drive and shooting mode
    char drive = 'o'; //default o when using joystick
    char shot = 'o'; //default o when using joystick


    //ref variables
    uint16_t chassis_buffer;
    uint16_t chassis_power_limit;

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    unsigned long loopTimerCV = loopTimer;
    int refLoop = 0;
    int printLoop = 0;

    bool cv_enabled = false;
    char cv_shoot_status = 0;

    ChassisSpeeds cs;

    while(true){
        timeStart = us_ticker_read();

        //CV loop runs every 2ms
        if((timeStart - loopTimerCV) / 1000 > 1) { //1 with sync or 2 without
            loopTimerCV = timeStart;
            
            Jetson_send_data jetson_send_data;
            jetson_send_data.chassis_x_velocity = 0.0;
            jetson_send_data.chassis_y_velocity = 0.0;

            jetson_send_data.pitch_angle_rads = ChassisSubsystem::ticksToRadians( (pitch_zero_offset_ticks - pitch.getData(ANGLE)) );
            jetson_send_data.pitch_velocity = pitch.getData(VELOCITY) / 60.0;

            jetson_send_data.yaw_angle_rads = (imuAngles.yaw + 180.0) * (M_PI / 180.0);
            jetson_send_data.yaw_velocity = yaw.getData(VELOCITY)/60.0;

            jetson_send_feedback(bcJetson, jetson_send_data);
        }

        if ((timeStart - loopTimer) / 1000 > OUTER_LOOP_DT_MS){
            float elapsedms = (timeStart - loopTimer) / 1000;
            loopTimer = timeStart;
            led = !led;
            refLoop++;
            if (refLoop >= 5){
                led2 = referee.readable();
                refereeThread(&referee);
                refLoop = 0;

                //POWER LIMIT OVERRIDE INCASE
                if(robot_status.chassis_power_limit < 10){
                    chassis_power_limit = 49;
                }else{
                    chassis_power_limit = robot_status.chassis_power_limit;
                }
                
                Chassis.power_limit = (float)chassis_power_limit;
                chassis_buffer = power_heat_data.buffer_energy;
            }

            Chassis.periodic();
            cs = Chassis.getChassisSpeeds();
            remoteRead();

            Jetson_read_data jetson_received_data;
            int readResult = jetson_read_values(bcJetson, jetson_received_data);

            if(cv_enabled){
                if(readResult > 0){
                    led3 = 1;
                    yaw_desired_angle = jetson_received_data.requested_yaw_rads / M_PI * 180;
                    pitch_desired_angle = jetson_received_data.requested_pitch_rads / M_PI * 180;
                    cv_shoot_status = jetson_received_data.shoot_status;
                }else{
                    led3 = 0;
                }
            } else {
              cv_shoot_status = 0;
              led3 = 0;
            }

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
            if(remote.keyPressed(Remote::Key::V)){
                shot = 'm';
            }else if(remote.keyPressed(Remote::Key::C)){
                shot = 'd';        
            }

            if(remote.getMouseR() || remote.leftSwitch() == Remote::SwitchState::MID){
                cv_enabled = true;
            }else if(!remote.getMouseR() ){
                cv_enabled = false;
            }


            //Driving input
            float scalar = 1;
            float jx = remote.leftX() / 660.0 * scalar; // -1 to 1
            float jy = remote.leftY() / 660.0 * scalar; // -1 to 1
            //Pitch, Yaw
            float jpitch = remote.rightY() / 660.0 * scalar; // -1 to 1
            float jyaw = remote.rightX() / 660.0 * scalar; // -1 to 1

            float myaw = remote.getMouseX();
            float mpitch = -remote.getMouseY();

            //joystick tolerance
            float tolerance = 0.05; 
            jx = (abs(jx) < tolerance) ? 0 : jx;
            jy = (abs(jy) < tolerance) ? 0 : jy;
            jpitch = (abs(jpitch) < tolerance) ? 0 : jpitch;
            jyaw = (abs(jyaw) < tolerance) ? 0 : jyaw;
            
            //Keyboard Driving
            float mult = 0.7;

            // Shift to make robot go slower
            if (remote.keyPressed(Remote::Key::SHIFT)) {
                mult = 0.5;
            }
            if(remote.keyPressed(Remote::Key::CTRL)){
              mult = 1;
            }

            jx += mult * ((remote.keyPressed(Remote::Key::D) ? 1 : 0) + (remote.keyPressed(Remote::Key::A) ? -1 : 0));
            jy += mult * ((remote.keyPressed(Remote::Key::W) ? 1 : 0) + (remote.keyPressed(Remote::Key::S) ? -1 : 0));

            float j_hypo = sqrt(jx * jx + jy * jy);
            if(j_hypo > 1.0){
              jx = jx / j_hypo;
              jy = jy / j_hypo;
            }
            //Bounding the four j variables
            jx = max(-1.0F, min(1.0F, jx));
            jy = max(-1.0F, min(1.0F, jy));
            jpitch = max(-1.0F, min(1.0F, jpitch));
            jyaw = max(-1.0F, min(1.0F, jyaw));

            float max_linear_vel = -1.24 + 0.0513 * chassis_power_limit + -0.000216 * (chassis_power_limit * chassis_power_limit);
            // float max_omega = 0.326 + 0.0857 * chassis_power_limit + -0.000183 * (chassis_power_limit * chassis_power_limit);
            float max_omega = 4.8;

            if(remote.keyPressed(Remote::Key::CTRL)){
              jx = 0.0;
              jy = 0.0;
              max_omega = 6.1;
            }

            float linear_hypo = sqrtf(jx * jx + jy * jy);
            if(linear_hypo > 1.0){
              linear_hypo = 1.0;
            }

            float available_beyblade = 1.0 - linear_hypo;
            float omega_speed = max_omega * available_beyblade;

            //Chassis Code
            ChassisSpeeds beybladeSpeeds = {jx * max_linear_vel,
                                          jy * max_linear_vel,
                                          -omega_speed};
            if (drive == 'u' || (drive =='o' && remote.rightSwitch() == Remote::SwitchState::UP)){
                //REGULAR DRIVING CODE
                Chassis.setChassisSpeeds({jx * max_linear_vel,
                                          jy * max_linear_vel,
                                          0},
                                          ChassisSubsystem::YAW_ORIENTED);
            }else if (drive == 'd' || (drive =='o' && remote.rightSwitch() == Remote::SwitchState::DOWN)){
                //BEYBLADE DRIVING CODE
                
                Chassis.setChassisSpeeds(beybladeSpeeds,
                                          ChassisSubsystem::YAW_ORIENTED);
            }else{
                //OFF
                Chassis.setWheelPower({0,0,0,0});
            }

            
            //YAW CODE
            float error = 0;
            if (drive == 'u' || drive == 'd' || (drive =='o' && (remote.rightSwitch() == Remote::SwitchState::UP || remote.rightSwitch() == Remote::SwitchState::DOWN))){
                float chassis_rotation_radps = cs.vOmega;
                int chassis_rotation_rpm = chassis_rotation_radps * 60 / (2*PI) * 1.5; //I added this 4 but I don't know why.
                
                //Regular Yaw Code
                yaw_desired_angle -= myaw * MOUSE_SENSITIVITY_YAW_DPS * elapsedms / 1000;
                yaw_desired_angle -= jyaw * JOYSTICK_SENSITIVITY_YAW_DPS * elapsedms / 1000;

                yaw_desired_angle = fmod((fmod(yaw_desired_angle, 360.0) + 360.0), 360.0); 

                #ifdef USE_IMU
                error = DJIMotor::s_calculateDeltaPhaseF(yaw_desired_angle, imuAngles.yaw + 180, 360);
                yawVelo = yaw.calculatePeriodicPosition(error, timeSure - prevTimeSure, chassis_rotation_rpm);
                #else
                yawVelo = -jyaw * JOYSTICK_SENSITIVITY_YAW_DPS / 360.0 * 60;
                #endif
                //yawVelo = 0;
                // yawVelo -= chassis_rotation_rpm;

                int dir = 0;
                if(yawVelo > 1){
                    dir = 1;
                }else if(yawVelo < -1){
                    dir = -1;
                }
                yaw.pidSpeed.feedForward = 1221 * dir + 97.4 * yawVelo;
                yaw.setSpeed(yawVelo);
            }else{
                //Off
                yaw.setPower(0);
                #ifdef USE_IMU
                yaw_desired_angle = imuAngles.yaw + 180;
                #endif
            }

            prevTimeSure = timeSure;
            timeSure = us_ticker_read();

            //PITCH
            pitch_current_angle = (pitch_zero_offset_ticks - (pitch>>ANGLE)) / TICKS_REVOLUTION * 360;
            if (drive == 'u' || drive == 'd' || (drive =='o' && (remote.rightSwitch() == Remote::SwitchState::UP || remote.rightSwitch() == Remote::SwitchState::DOWN))){
                //Regular Pitch Code
                pitch_desired_angle += mpitch * MOUSE_SENSITIVITY_PITCH_DPS * elapsedms / 1000;
                pitch_desired_angle += jpitch * JOYSTICK_SENSITIVITY_PITCH_DPS * elapsedms / 1000;

                // if(jpitch > -0.33 && jpitch < 0.33){
                //     pitch_desired_angle = 0;
                // }else if(jpitch > 0.33){
                //     pitch_desired_angle = 30;
                // }else if(jpitch < -0.33){
                //     pitch_desired_angle = -30;
                // }       

                if (pitch_desired_angle <= LOWERBOUND) {
                    pitch_desired_angle = LOWERBOUND;
                }
                else if (pitch_desired_angle >= UPPERBOUND) {
                    pitch_desired_angle = UPPERBOUND;
                }

                pitchVelo = -pitchCascade.calculatePeriodic(pitch_desired_angle - pitch_current_angle, timeSure - prevTimeSure);
                
                int dir = 0;
                if(pitchVelo > 1){
                    dir = 1;
                }else if(pitchVelo < -1){
                    dir = -1;
                }

                float pitch_current_radians = -(pitch_current_angle / 360) * 2 * M_PI;
                pitch.pidSpeed.feedForward = (cos(pitch_current_radians) * -2600) + (1221 * dir + 97.4 * yawVelo);
                //pitch.setPosition(-int((pitch_desired_angle / 360) * TICKS_REVOLUTION - pitch_zero_offset_ticks));
                pitch.setSpeed(pitchVelo);
            }else{
                //Off
                pitch.setPower(0);
            }
            
            //INDEXER CODE
            if ((remote.leftSwitch() == Remote::SwitchState::UP || remote.getMouseL()) && (abs(RFLYWHEEL>>VELOCITY) > (FLYWHEEL_VELO - 500) && abs(LFLYWHEEL>>VELOCITY) > (FLYWHEEL_VELO - 500)) 
                /*&& remote.rightSwitch() != Remote::SwitchState::MID*/){        
                if (shootReady){

                    //shoot limit
                    if(robot_status.shooter_barrel_heat_limit < 10 || power_heat_data.shooter_17mm_1_barrel_heat < robot_status.shooter_barrel_heat_limit - 40) {
                        shoot = true;
                        shootReady = false;
                        shootTargetPosition = (8192 * M2006_GEAR_RATIO / 9 * NUM_BALLS_SHOT) + (indexer>>MULTITURNANGLE);
                    }
                    
                }
            } else if(!(remote.leftSwitch() == Remote::SwitchState::UP || remote.getMouseL())) {
                //SwitchState state set to mid/down/unknown
                shootReady = true;
            }

            // burst fire, turn the indexer to shoot 3-5 balls a time and stop indexer
            // only shoot when left switch changes from down/unknown/mid to up
            // if left switch remains at up state, indexer stops after 3-5 balls
            int indexer_target_velocity = 0; 
            if (shoot){
                // 1 degree of error allowed
                if (abs((indexer>>MULTITURNANGLE) - shootTargetPosition) <= 819){
                    // indexer.setSpeed(indexer_target_velocity);
                    // indexer.pidSpeed.feedForward = 0;
                    shoot = false;
                } else {
                    indexer_target_velocity = sure.calculate(shootTargetPosition, indexer>>MULTITURNANGLE, timeSure - prevTimeSure);
                    indexer.setSpeed(indexer_target_velocity); //
                    indexer.pidSpeed.feedForward = (indexer>>VALUE) / 4788 * 630;
                }
            } else {
                indexer.setSpeed(0);
                indexer.pidSpeed.feedForward = 0;
            }

            //FLYWHEELS
            if (shot == 'm' || (shot == 'o' && remote.leftSwitch() != Remote::SwitchState::DOWN && remote.leftSwitch() != Remote::SwitchState::UNKNOWN)){
                LFLYWHEEL.setSpeed(-FLYWHEEL_VELO);
                RFLYWHEEL.setSpeed(FLYWHEEL_VELO);
                LFLYWHEEL.pidSpeed.feedForward = 52;
                RFLYWHEEL.pidSpeed.feedForward = 77;
            } else{
                // left SwitchState set to up/mid/unknown
                if(abs(LFLYWHEEL>>VELOCITY) < 50){
                    LFLYWHEEL.setPower(0);
                }else{
                    LFLYWHEEL.setSpeed(0);
                }
                if(abs(RFLYWHEEL>>VELOCITY) < 50){
                    RFLYWHEEL.setPower(0);
                }else{
                    RFLYWHEEL.setSpeed(0);
                }
                LFLYWHEEL.pidSpeed.feedForward = 0;
                RFLYWHEEL.pidSpeed.feedForward = 0;
            }

            printLoop ++;
            if (printLoop >= PRINT_FREQUENCY){
                printLoop = 0;
                // printff("cv_enable: %d | cv_shoot_status: %d\n", cv_enabled == true, cv_shoot_status);
                //printff("Prints:\n");
                //printff("lX:%.1f lY:%.1f rX:%.1f rY:%.1f lS:%d rS:%d\n", remote.leftX(), remote.leftY(), remote.rightX(), remote.rightY(), remote.leftSwitch(), remote.rightSwitch());
                //printff("jx:%.3f jy:%.3f jpitch:%.3f jyaw:%.3f\n", jx, jy, jpitch, jyaw);
                #ifdef USE_IMU
                // printff("%d\n", remote.getMouseL());
                //printff("ydv:%d yav:%d PWR:%d ", yawVelo, yaw>>VELOCITY, yaw>>POWEROUT);
                //printff("V[%.1f][%.1f][%.1f]E:%.3f ", yaw.pidSpeed.pC, yaw.pidSpeed.iC, yaw.pidSpeed.dC, yawVelo - (yaw>>VELOCITY));
                //printff("YD:%.3f YA:%.3f CVY:%.3f\n", yaw_desired_angle, imuAngles.yaw + 180, CV_yaw_angle_radians * 180 / M_PI);
                // printff("ERR:%.3f\n", error);
                //printff("YD:%.3f YA:%.3f CVY:%.3f\n", yaw_desired_angle, imuAngles.yaw + 180, CV_yaw_angle_radians * 180 / M_PI);
                //printff("ERR:%.3f\n", error);
                #else
                //printff("yaw_des_v:%d yaw_act_v:%d\n", yawVelo, yaw>>VELOCITY);
                //printff("yaw_des:%.3f yaw_act:%.3f [%d]\n", yaw_desired_angle, yaw_current_angle, yaw>>ANGLE);
                #endif
                //printff("yaw_des_v:%d yaw_act_v:%d", yawVelo, yaw>>VELOCITY);

                //printff("ydv:%d yav:%d PWR:%d ", pitchVelo, pitch>>VELOCITY, pitch>>POWEROUT);
                //printff("yd:%.3f ya:%.3f [%d] %.3f ", pitch_desired_angle, pitch_current_angle, yaw>>ANGLE, pitch_desired_angle- pitch_current_angle);
                //printff("[%d][%d][%d]\n", (int)pitch.pidSpeed.pC, (int)pitch.pidSpeed.iC, (int)pitch.pidSpeed.dC);
                // printff("[%.1f][%.1f][%.1f] %.2f\n", pitchCascade.pC, pitchCascade.iC, pitchCascade.dC, pitch_desired_angle- pitch_current_angle);
                // printff("%d | [%.1f][%.1f][%.1f] %d | %d\n", shoot, indexer.pidSpeed.pC, indexer.pidSpeed.iC, indexer.pidSpeed.dC,  indexer_target_velocity - (indexer>>VELOCITY), indexer_target_velocity);
                // printff("%d | %d\n", LFLYWHEEL>>VELOCITY, RFLYWHEEL>>VELOCITY);
                //printff("%lu %.2f %.2f\n", loopTimer, pitch_desired_angle, pitch_current_angle);
                
                //printff("pitch_des:%.3f pitch_act:%.3f [%d]\n", pitch_desired_angle, pitch_current_angle, pitch>>ANGLE);
                //printff("cX%.1f cY%.1f cOmega%.3f cRPM%.1f\n", cs.vX, cs.vY, cs.vOmega, cs.vOmega * 60 / (2*M_PI) * 4);
                printff("Chassis: LF:%c RF:%c LB:%c RB:%c Yaw:%c Pitch:%c Flywheel_L:%c Flywheel_R:%c Indexer:%c\n", 
                    Chassis.getMotor(ChassisSubsystem::LEFT_FRONT).isConnected() ? 'y' : 'n', 
                    Chassis.getMotor(ChassisSubsystem::RIGHT_FRONT).isConnected() ? 'y' : 'n', 
                    Chassis.getMotor(ChassisSubsystem::LEFT_BACK).isConnected() ? 'y' : 'n', 
                    Chassis.getMotor(ChassisSubsystem::RIGHT_BACK).isConnected() ? 'y' : 'n',
                    yaw.isConnected() ? 'y' : 'n', 
                    pitch.isConnected() ? 'y' : 'n', 
                    LFLYWHEEL.isConnected() ? 'y' : 'n', 
                    RFLYWHEEL.isConnected() ? 'y' : 'n',
                    indexer.isConnected() ? 'y' : 'n');
                #ifdef USE_IMU
                // printff("IMU %.3f %.3f %.3f\n",imuAngles.yaw, imuAngles.pitch, imuAngles.roll);
                #endif
                //printff("pwr:%u max:%d heat:%d\n", chassis_buffer, robot_status.chassis_power_limit, power_heat_data.shooter_17mm_1_barrel_heat);
                //printff("ID:%d LVL:%d HP:%d MAX_HP:%d\n", robot_status.robot_id, robot_status.robot_level, robot_status.current_HP, robot_status.maximum_HP);
                // printff("elap:%.5fms\n", elapsedms);
                //printff("[HEAT] lim:%d buf:%d b1:%d b2:%d sp:%.1f fr:%d\n", robot_status.shooter_barrel_heat_limit, power_heat_data.buffer_energy, power_heat_data.shooter_17mm_1_barrel_heat, power_heat_data.shooter_17mm_2_barrel_heat, shoot_data.initial_speed, shoot_data.launching_frequency);

                // if(nucleo_value[24] != 0 && remote.rightSwitch() == Remote::SwitchState::UP){
                //     printff("[");
                //     for(int i = 0; i < 30; i ++){
                //         printff("%x,", nucleo_value[i]);
                //     }
                //     printff("]");
                // }
                //printff("CS: %.1f %.1f %.1f ", cs.vX, cs.vY, cs.vOmega);
                //printff("DS: %.1f %.1f %.1f\n", beybladeSpeeds.vX, beybladeSpeeds.vY, beybladeSpeeds.vOmega);
                // WheelSpeeds ac = Chassis.getWheelSpeeds();
                // WheelSpeeds ws = Chassis.chassisSpeedsToWheelSpeeds(beybladeSpeeds);
                //printff("CH: %.2f %.2f %.2f %.2f ", ws.LF,ws.RF,ws.LB,ws.RB);
                //printff("A: %.2f %.2f %.2f %.2f\n", ac.LF,ac.RF,ac.LB,ac.RB);
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
                //printff("%d,%d,%d,%d\n", abs(LFLYWHEEL>>VELOCITY), abs(LFLYWHEEL>>POWEROUT), abs(RFLYWHEEL>>VELOCITY), abs(RFLYWHEEL>>POWEROUT));
                // if(remote.rightSwitch() == Remote::SwitchState::UP){
                    // printff("%d %d %u %u %u %f ", abs(LFLYWHEEL>>VELOCITY), abs(RFLYWHEEL>>VELOCITY), shoot_data.bullet_type, shoot_data.shooter_number, shoot_data.launching_frequency, *(float*)(((uint8_t*)&shoot_data)+4) /*((shoot_data.initial_speed / 0.06) * (30 / M_PI))*/);
                    // for(int i = 0; i < 7; i ++){
                    //    printff("(%2x)", (uint8_t)*((((uint8_t*)&shoot_data)+i)));
                    // }
                    // float c = 0;
                    // memcpy(&c, ((uint8_t*)&shoot_data)+3, 4);
                    // for(int i = 0; i < 4; i ++){
                    //    printff("[%2x]", (uint8_t)*((((uint8_t*)&c)+i)));
                    // }
                    // printff(" %f %f %8x\n", shoot_data.initial_speed, c, c);
                    // printff("%d %d %f %f\n", abs(LFLYWHEEL>>VELOCITY), abs(RFLYWHEEL>>VELOCITY), ((c / 0.06) * (30 / M_PI)), c);
                    //printff("%d %d\n", abs(LFLYWHEEL>>VELOCITY), abs(RFLYWHEEL>>VELOCITY));
                    
                // }
            }

            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}