#include "main.h"
#include "subsystems/ChassisSubsystem.h"

#define PI 3.14159265
#define TIME 15
#define RADIUS 0.2286

DigitalOut led(L25);
DigitalOut led2(L26);
DigitalOut led3(L27);
DigitalOut ledbuiltin(LED1);

//CONSTANTS
constexpr float GEAR_RATIO = 2.0;
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

constexpr int FLYWHEEL_SPEED = 6000;

constexpr float BUFFER_ANGLE = PI / 16;
// PI/2 radps = 1 in vOmega terms (idk why), so we're converting PI/4 radps to 0.5 vOmega term
constexpr float ROT_INIT = 1; 
constexpr float ACCEL_INIT = 0.2;
constexpr float VEL_INIT = 0.5;
constexpr float DECEL_DIST = 2 * VEL_INIT * VEL_INIT / (2 * ACCEL_INIT* TIME);
constexpr float BUFFER = 0.1;
float rotation = 0;

#define READ_DEBUG 0
#define MAGICBYTE 0xEE
#define USE_IMU

//CV STUFF
static BufferedSerial bcJetson(PC_12, PD_2, 115200);  //JETSON PORT
float xRotated, yRotated;

char yaw_angle_char[4];
char yaw_velocity_char[4];
char pitch_angle_char[4];
char pitch_velocity_char[4];

//CV
float CV_pitch_angle_radians = 0.0;
float CV_yaw_angle_radians = 0.0;
char CV_shoot = 0;

//CHASSIS DEFINING
I2C i2c(I2C_SDA, I2C_SCL);

BNO055 imu(i2c, IMU_RESET, MODE_IMU);
ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2286); // radius is 9 in
DJIMotor yaw(7, CANHandler::CANBUS_1, GIMBLY,"Yeah");
DJIMotor pitch(5, CANHandler::CANBUS_2, GIMBLY,"Peach"); // right
DJIMotor yaw2(6, CANHandler::CANBUS_1, GIMBLY,"Ye2"); // left, not functioning

DJIMotor indexerL(5, CANHandler::CANBUS_2, C610,"IndexerL");
DJIMotor indexerR(6, CANHandler::CANBUS_2, C610,"IndexerR");
DJIMotor RFLYWHEEL_U(1, CANHandler::CANBUS_2, M3508,"RightFlyU");
DJIMotor LFLYWHEEL_U(2, CANHandler::CANBUS_2, M3508,"LeftFlyU");
DJIMotor RFLYWHEEL_D(3, CANHandler::CANBUS_2, M3508,"RightFlyD");
DJIMotor LFLYWHEEL_D(4, CANHandler::CANBUS_2, M3508,"LeftFlyD");

#ifdef USE_IMU
BNO055_ANGULAR_POSITION_typedef imuAngles;
#endif

struct SetValues {
    float lx;
    float ly;
    float rx;
};

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

float calculateDistance(float posx, float posy, float final_x, float final_y) {
    return sqrt(pow((final_x - posx), 2) + pow((final_y - posy), 2));
}

SetValues calculate_chassis_speeds(float posx, float posy, float angle, float final_x, float final_y, float vX, float vY) {
    float distance = calculateDistance(posx, posy, final_x, final_y);
    SetValues values;
    values.lx = 0;
    values.ly = 0;
    values.rx = 0;

    // if the final position is too far away:
    if ((final_x - posx) > BUFFER) {
        // if velocity is less than "max vel"
        if (abs(vX) < VEL_INIT) {
            // increase the velocity 
            values.lx = vX + ACCEL_INIT;
        }
        // velocity is too high, cap at 1
        else {
            values.lx = VEL_INIT;
        }

        // if we're within decelerating distance:
        if (abs(final_x - posx) < DECEL_DIST) {
            // if velocity is positive:
            if (vX > 0) {
                // decrease it by our acceleration cap
                values.lx = vX - ACCEL_INIT;
            }
            else {
                values.lx = 0;
            }
        }
    }
    // same logic as above, but this time final_x is negative
    else if ((posx - final_x) > BUFFER) {
        if (abs(vX) < VEL_INIT) {
            values.lx = vX - ACCEL_INIT;
        }
        else {
            values.lx = -VEL_INIT;
        }

        if (abs(final_x - posx) < DECEL_DIST) {
            if (vX < 0) {
                values.lx = vX + ACCEL_INIT;
            }
            else {
                values.lx = 0;
            }
        }
    }
    else {
        values.lx = 0;
    }
    

    // same logic as x
    if ((final_y - posy) > BUFFER) {
        if (abs(vY) < VEL_INIT) {
            values.ly = vY + ACCEL_INIT;
        }
        else {
            values.ly = VEL_INIT;
        }

        if (abs(final_y - posy) < DECEL_DIST) {
            if (vY > 0) {
                values.ly = vY - ACCEL_INIT;
            }
            else {
                values.ly = 0;
            }
        }
    }
    else if ((posy - final_y) > BUFFER) {
        if (abs(vY) < VEL_INIT) {
            values.ly = vY - ACCEL_INIT;
        }
        else {
            values.ly = -VEL_INIT;
        }

        if (abs(final_y - posy) < DECEL_DIST) {
            if (vY < 0) {
                values.ly = vY + ACCEL_INIT;
            }
            else {
                values.ly = 0;
            }
        }
    }
    else {
        values.ly = 0;
    }

    if (angle > BUFFER_ANGLE) {
        values.rx = ROT_INIT;
        values.lx = 0;
        values.ly = 0;
    }
    else if (angle < -BUFFER_ANGLE) {
        values.rx = -ROT_INIT;
        values.lx = 0;
        values.ly = 0;
    }
    else {
        values.rx = 0;
    }

    return values;
}

int main(){

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();

    ChassisSpeeds velocity = {0.0, 0.0, 0.0};
    ChassisSpeeds prev_velocity = {0.0, 0.0, 0.0};
    ChassisSpeeds accel = {0.0, 0.0, 0.0};

    /*
    * MOTORS SETUP AND PIDS
    */
    //YAW

    // for CV
    // PID yawBeyblade(0.3, 0, 0); //yaw PID is cascading, so there are external position PIDs for yaw control
    // yawBeyblade.setOutputCap(30);
    // // yaw.setSpeedPID(924.48, 1.8563, 100);
    // yaw.setSpeedPID(1250.355, 1.0061, 0);
    // // yaw.setSpeedPID(3386.438, 7.4473, 17859.4174);
    // yaw.setSpeedIntegralCap(8000);
    // yaw.setSpeedDerivativeCap(4000);
    // yaw.setSpeedOutputCap(32000);
    // yaw.outputCap = 16000;
    // yaw.useAbsEncoder = false;

    // // for manual driving
    PID yawBeyblade(0.5, 0, 0); //yaw PID is cascading, so there are external position PIDs for yaw control
    yawBeyblade.setOutputCap(35);
    yaw.setSpeedPID(922.9095, 0.51424, 0);
    yaw.setSpeedIntegralCap(5000);
    yaw.setSpeedOutputCap(32000);
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
    PID pitchCascade(1,0.0005,0.05);
    pitchCascade.setIntegralCap(2);
    pitchCascade.setOutputCap(30);
    pitch.setSpeedPID(400,0.8,0);
    pitch.setSpeedIntegralCap(4000);
    pitch.setSpeedOutputCap(32000);
    pitch.pidPosition.feedForward = 0;
    pitch.outputCap = 16000;
    pitch.useAbsEncoder = true;
    pitchCascade.dBuffer.lastY = 5;

    int pitchVelo = 0;

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
    Chassis.setYawReference(&yaw, 5650); //the number of ticks of yaw considered to be robot-front
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
    unsigned long loopTimerCV = loopTimer;
    int refLoop = 0;
    int printLoop = 0;

    ChassisSpeeds cs;

    bool cv_enabled = true;

    // Auto code
    std::vector<std::vector<float>> final_pos = {{0.0, 0.0}, {0.0,5.0}};//, {-3.5,5.0}, {-3.5,2.0}};

    float angle = 0.0;
    float posx = final_pos[0][0]; // need to go to 1676 ish
    float posy = final_pos[0][1]; // we need to go to -6800 (ish)
    int counter = 0;
    int beyblade_counter = 0;
    int settle_counter = 0;
    
    // final positions
    int idx = 1;
    float final_y = final_pos[idx][1];
    float final_x = final_pos[idx][0];

    float rx_init = 0.4; // you basically divide the angle you want by pi, so this is PI/4 / PI

    // ensure robot is not moving at startup
    Chassis.setWheelPower({0,0,0,0});

    while(true){
        timeStart = us_ticker_read();

        //CV loop runs every 2ms
        if((timeStart - loopTimerCV) / 1000 > 1) { 
            loopTimerCV = timeStart;

            Jetson_send_data jetson_send_data;
            jetson_send_data.chassis_x_velocity = 0.0;
            jetson_send_data.chassis_y_velocity = 0.0;

            jetson_send_data.pitch_angle_rads = ChassisSubsystem::ticksToRadians( (pitch_zero_offset_ticks - pitch.getData(ANGLE)) / GEAR_RATIO);
            jetson_send_data.pitch_velocity = pitch.getData(VELOCITY) / 60.0;

            jetson_send_data.yaw_angle_rads = (imuAngles.yaw + 180.0) * (M_PI / 180.0);
            jetson_send_data.yaw_velocity = yaw.getData(VELOCITY)/60.0;

            jetson_send_feedback(bcJetson, jetson_send_data); 
            led2 = !led2;
        }

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
            }
            Chassis.periodic();
            cs = Chassis.getChassisSpeeds();
            remoteRead();

            Jetson_read_data jetson_received_data;
            int readResult = jetson_read_values(bcJetson, jetson_received_data);
            char shoot_status_cv = 0;

            if(remote.rightSwitch() == Remote::SwitchState::DOWN){
              cv_enabled = false;
            } else {
              cv_enabled = true;
            }

            if(cv_enabled){
                if(readResult > 0){
                    yaw_desired_angle = jetson_received_data.requested_yaw_rads / M_PI * 180;
                    pitch_desired_angle = jetson_received_data.requested_pitch_rads * GEAR_RATIO / M_PI * 180;
                    shoot_status_cv = jetson_received_data.shoot_status;
                }
            }

            //like idk why ig floats are fucked oop
            if ( abs(CV_yaw_angle_radians) > 0.001 ) {
                //printf("you are now zero\n");
                // CV_yaw_angle_radians = 0;
                yaw_desired_angle = CV_yaw_angle_radians / M_PI * 180;
            }
            
            if ( abs(CV_pitch_angle_radians) > 0.001 ) {
                // CV_pitch_angle_radians = 0;
                pitch_desired_angle = CV_pitch_angle_radians / M_PI * 180;
            }
            
            #ifdef USE_IMU
            imu.get_angular_position_quat(&imuAngles);
            #else
            yaw_current_angle = (yaw>>ANGLE) * 360.0 / TICKS_REVOLUTION;
            #endif

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
            jyaw = (abs(jyaw) < tolerance * 3) ? 0 : jyaw;
            
            //Keyboard Driving
            float mult = 1;
            jx += mult * ((remote.keyPressed(Remote::Key::D) ? 1 : 0) + (remote.keyPressed(Remote::Key::A) ? -1 : 0));
            jy += mult * ((remote.keyPressed(Remote::Key::W) ? 1 : 0) + (remote.keyPressed(Remote::Key::S) ? -1 : 0));
            
            //Bounding the four j variables
            jx = max(-1.0F, min(1.0F, jx));
            jy = max(-1.0F, min(1.0F, jy));
            jpitch = max(-1.0F, min(1.0F, jpitch));
            jyaw = max(-1.0F, min(1.0F, jyaw));

            // auto position
            velocity = Chassis.getChassisSpeeds();
            accel =  {(velocity.vX     - prev_velocity.vX)     / (TIME*0.001), 
                      (velocity.vY     - prev_velocity.vY)     / (TIME*0.001), 
                      (velocity.vOmega - prev_velocity.vOmega) / (TIME*0.001)}; 
            
            // angle is always 0 for robot oriented drive
            angle = angle - ((velocity.vOmega * TIME * 0.001) + (1/2 * accel.vOmega * TIME * 0.001 * TIME * 0.001)) * PI / 2;
            
            if (angle > PI) {
                angle -= 2 * PI;
            }
            else if (angle < -PI) {
                angle += 2 * PI;
            }

            //angle = 0.0;

            posx = posx + ((velocity.vX * cos(angle) + velocity.vY * sin(angle)) * TIME * 0.001
                + (0.5 * (accel.vX * cos(angle) + accel.vY * sin(angle)) * TIME * 0.001 * TIME * 0.001)) * 1.6;

            posy = posy + ((velocity.vY * cos(angle) - velocity.vX * sin(angle)) * TIME * 0.001
                + (0.5 * (accel.vY * cos(angle) - accel.vX * sin(angle)) * TIME * 0.001 * TIME * 0.001)) * 1.6;

            float lx = 0;
            float ly = 0;

            if (remote.rightSwitch() == Remote::SwitchState::DOWN) {
                led = 1;
                lx = (remote.leftX() / 660.0) * Chassis.m_OmniKinematicsLimits.max_Vel;
                ly = (remote.leftY() / 660.0) * Chassis.m_OmniKinematicsLimits.max_Vel;
              
                Chassis.setChassisSpeeds({lx, ly, 0}, ChassisSubsystem::ROBOT_ORIENTED);
            }

            if(remote.rightSwitch() == Remote::SwitchState::UP){
              // Chassis.setChassisSpeeds({0,0,3});
            }

            if(remote.rightSwitch() == Remote::SwitchState::MID){
              Chassis.setWheelPower({0,0,0,0});
            }

            //TODO FIX ME 
            // if (true) {
            //     if (remote.rightSwitch() == Remote::SwitchState::UP) {
            //         led = 0;
            //         float distance = calculateDistance(posx, posy, final_x, final_y);

            //         SetValues values = calculate_chassis_speeds(posx, posy, angle, final_x, final_y, velocity.vX, velocity.vY);

            //         // If robot has stopped moving or its close to its setpoint:
            //         if ((distance < M_SQRT2 * BUFFER)) {
            //             settle_counter+= TIME;
            //             // if hp is greater than 20%, go to next setpoint
            //             if (robot_status.current_HP > robot_status.maximum_HP * 0.2) {

            //                 // if we are not at the final setpoint and 300ms have passed, move to the next setpoint
            //                 if ((idx < final_pos.size() - 1) && (settle_counter > 300)) {
            //                         idx += 1;
            //                         final_y = final_pos[idx][1];
            //                         final_x = final_pos[idx][0];
            //                 }
            //             }
            //             // hp low, so go to previous setpoint
            //             else {
            //                 beyblade_counter = 0;
            //                 if (idx > 0) {
            //                     idx -= 1;
            //                     final_y = final_pos[idx][1];
            //                     final_x = final_pos[idx][0];
            //                 }
            //             }
            //             values.ly = 0;
            //             values.lx = 0;
            //         }
                    
            //         // we've reached the last setpoint, if we stopped moving, then start beyblading
            //         if (idx == final_pos.size() - 1 && velocity.vX == 0 && velocity.vY == 0) {
            //             beyblade_counter++;
            //         } 
            //         if (beyblade_counter > 10) {
            //             values.rx = 3;
            //         }

            //         // Chassis.setChassisSpeeds({values.lx, values.ly, values.rx}, ChassisSubsystem::ROBOT_ORIENTED);
            //         Chassis.setChassisSpeeds({0,0,3});
            //     }
            //     else {
            //         //OFF
            //         Chassis.setWheelPower({0,0,0,0});
            //     }
                
            //     prev_velocity = {velocity.vX, velocity.vY, velocity.vOmega};
            // }

            //YAW CODE
            if (remote.rightSwitch() == Remote::SwitchState::DOWN  || remote.rightSwitch() == Remote::SwitchState::UP){
                float chassis_rotation_radps = cs.vOmega;
                int chassis_rotation_rpm = chassis_rotation_radps * 60 / (2*M_PI) * 1.5; //I added this 4 but I don't know why.
                
                //Regular Yaw Code
                yaw_desired_angle -= jyaw * JOYSTICK_SENSITIVITY_YAW_DPS * elapsedms / 1000;
                yaw_desired_angle = fmod((fmod(yaw_desired_angle, 360.0) + 360.0), 360.0);

                #ifdef USE_IMU
                yawVelo = -yawBeyblade.calculatePeriodic(DJIMotor::s_calculateDeltaPhase(yaw_desired_angle, imuAngles.yaw + 180, 360), timeSure - prevTimeSure);
                #else
                yawVelo = jyaw * JOYSTICK_SENSITIVITY_YAW_DPS / 360.0 * 60;
                #endif
                //yawVelo = 0;
                yawVelo += chassis_rotation_rpm;

                int dir = 0;
                if(yawVelo > 1){
                    dir = 1;
                }else if(yawVelo < 1){
                    dir = -1;
                }

                // 2 degree
                yaw.pidSpeed.feedForward = -0.6840 * yawVelo * yawVelo * dir + 225.6726 * yawVelo + 1868 * dir;

                // 1 degree
                // yaw.pidSpeed.feedForward = 175.3608 * yawVelo + 2302.1 * dir;

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
            if (remote.rightSwitch() == Remote::SwitchState::DOWN || remote.rightSwitch() == Remote::SwitchState::UP){
                //Regular Pitch Code
                pitch_desired_angle -= jpitch * JOYSTICK_SENSITIVITY_PITCH_DPS * elapsedms / 1000;

                if (pitch_desired_angle >= LOWERBOUND) {
                    pitch_desired_angle = LOWERBOUND;
                }
                else if (pitch_desired_angle <= UPPERBOUND) {
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
                pitch.pidSpeed.feedForward = (1221 * dir + 97.4 * yawVelo);
                //pitch.setPosition(-int((pitch_desired_angle / 360) * TICKS_REVOLUTION - pitch_zero_offset_ticks));
                pitch.setSpeed(pitchVelo);
            }else{
                //Off
                pitch.setPower(0);
            }
            pitch_current_angle = (pitch_zero_offset_ticks - (pitch>>ANGLE)) / TICKS_REVOLUTION * 360 / 2;

            //INDEXER CODE
            if ((remote.rightSwitch() == Remote::SwitchState::DOWN && remote.leftSwitch() == Remote::SwitchState::UP) || 
                shoot_status_cv == 1) {
                if(robot_status.shooter_barrel_heat_limit < 10 || power_heat_data.shooter_17mm_1_barrel_heat < robot_status.shooter_barrel_heat_limit - 30) {
                    indexerL.setSpeed(-5 * 16 * M2006_GEAR_RATIO);
                }
                else {
                    indexerL.setSpeed(0);
                }
                
                if(robot_status.shooter_barrel_heat_limit < 10 || power_heat_data.shooter_17mm_2_barrel_heat < robot_status.shooter_barrel_heat_limit - 30) {
                    indexerR.setSpeed(5 * 16 * M2006_GEAR_RATIO);
                }
                else {
                    indexerR.setSpeed(0);
                }
            } 
            else {
                indexerL.setSpeed(0);
                indexerR.setSpeed(0);
            }

            //FLYWHEELS
            if ((remote.leftSwitch() != Remote::SwitchState::DOWN &&
                remote.leftSwitch() != Remote::SwitchState::UNKNOWN &&
                remote.rightSwitch() == Remote::SwitchState::DOWN) ||
                remote.rightSwitch() == Remote::SwitchState::UP){
                RFLYWHEEL_U.setSpeed(FLYWHEEL_SPEED);  // correct
                LFLYWHEEL_U.setSpeed(-FLYWHEEL_SPEED); // actually RIGHT DOWN (from the back of the robot)
                RFLYWHEEL_D.setSpeed(-FLYWHEEL_SPEED); // actually LEFT UP
                LFLYWHEEL_D.setSpeed(FLYWHEEL_SPEED);  // correct
            } else{
                // left SwitchState set to down/unknown
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
                // printff("yaw_des_v:%d yaw_act_v:%d ", yawVelo, yaw>>VELOCITY);
                // printff("yaw_des:%.3f yaw_act:%.3f\n", yaw_desired_angle, imuAngles.yaw + 180);
                // printff("%.3f, %.3f, %.3f\n", imuAngles.yaw + 180, imuAngles.roll + 180, imuAngles.pitch + 180 );
                #else
                // printff("yaw_des_v:%d yaw_act_v:%d PWR:%d ", yawVelo, yaw>>VELOCITY, yaw>>POWEROUT);
                // printff("yaw_des:%.3f yaw_act:%.3f [%d]\n", yaw_desired_angle, yaw_current_angle, yaw>>ANGLE);
                #endif
                // printff("[%.1f][%.1f][%.1f] %.1f %.1f\n", pitchCascade.pC, pitchCascade.iC, pitchCascade.dC,  pitch_current_angle, pitch_desired_angle);
                // printff("%d, %d\n", power_heat_data.shooter_17mm_1_barrel_heat, power_heat_data.shooter_17mm_2_barrel_heat);
                //printff("pitch_des_v:%d yaw_act_v:%d", yawVelo, yaw>>VELOCITY);
                //printff("pitch_des:%.3f pitch_act:%.3f [%d]\n", pitch_desired_angle, pitch_current_angle, pitch>>ANGLE);
                //printff("cX%.1f cY%.1f cOmega%.3f cRPM%.1f\n", cs.vX, cs.vY, cs.vOmega, cs.vOmega * 60 / (2*M_PI) * 4);
                // printff("LF:%c RF:%c LB:%c RB:%c Yaw:%c Pitch:%c\n", 
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
                // printff("pwr:%u max:%d heat:%d\n", chassis_buffer, robot_status.chassis_power_limit, power_heat_data.shooter_17mm_1_barrel_heat);
                //printff("ID:%d LVL:%d HP:%d MAX_HP:%d\n", robot_status.robot_id, robot_status.robot_level, robot_status.current_HP, robot_status.maximum_HP);
                //printff("elap:%.5fms\n", elapsedms);
                // printff("%d\n", game_status.game_progress);
            }

            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}