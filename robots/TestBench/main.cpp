// Yaw or Pitch response reading

// #include "main.h"
// #include <cmath>

// // Testing Parameters
// bool infantry = true, sentry = false, hero = false;
// bool pitch = false, yaw = true;
// bool ind = false;
// bool chassis = false;
// bool position = false, velocity = true;

// DigitalOut led(L27);
// DigitalOut led2(L26);
// DigitalOut led3(L25);
// DigitalOut ledbuiltin(LED1);

// DJIMotor sentryPitch(5, CANHandler::CANBUS_2, GM6020, "sentry_pitch_motor");
// DJIMotor sentryYaw1(6, CANHandler::CANBUS_1, GM6020, "sentry_yaw_motor_1");
// DJIMotor sentryYaw2(7, CANHandler::CANBUS_1, GM6020, "sentry_yaw_motor_2");
// DJIMotor infPitch(7, CANHandler::CANBUS_2, GM6020, "infantry_pitch_motor");
// DJIMotor infYaw(4, CANHandler::CANBUS_1, GM6020, "infantry_yaw_motor");
// DJIMotor indexer(2, CANHandler::CANBUS_1, M3508, "indexer");
// DJIMotor heroPitch(5, CANHandler::CANBUS_2, GM6020, "hero_pitch_motor");
// DJIMotor heroYaw(1, CANHandler::CANBUS_1, GM6020, "hero_yaw_motor");

// // DJIMotor motor1(1, CANHandler::CANBUS_1, M3508, "chassis motor 1");
// // DJIMotor motor2(2, CANHandler::CANBUS_1, M3508, "chassis motor 2");
// // DJIMotor motor3(3, CANHandler::CANBUS_1, M3508, "chassis motor 3");
// // DJIMotor motor4(4, CANHandler::CANBUS_1, M3508, "chassis motor 4");

// #define IMPULSE_STRENGTH 8191
// #define REMOTE_MAX 660

// int main(){

//     DJIMotor *testMot = nullptr;
//     DJIMotor *testMot2 = nullptr;
//     DJIMotor *testMot3 = nullptr;
//     DJIMotor *testMot4 = nullptr;

//     if (infantry && pitch) {
//         testMot = &infPitch;
//     }
//     // else if (chassis) {
//         // testMot = &motor1;
//         // testMot2 = &motor2;
//         // testMot3 = &motor3;
//         // testMot4 = &motor4;
//     // }
//     else if (infantry && yaw)
//          testMot = &infYaw;
//     else if (sentry && pitch)
//          testMot = &sentryPitch;
//     else if (sentry && yaw) {
//         testMot = &sentryYaw1;
//         testMot2 = &sentryYaw2;
//     }
//     else if (hero && pitch)
//          testMot = &heroPitch;
//     else if (hero && yaw)
//          testMot = &heroYaw;
//     else if (ind)
//         testMot = &indexer;

//     DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
//     DJIMotor::s_sendValues();
//     DJIMotor::s_getFeedback();

//     unsigned long timeStart;
//     unsigned long loopTimer = us_ticker_read();
//     int refLoop = 0;

//     bool prevL = false;
//     bool switL = false;
//     bool debug = false;

//     int motorSpeed = 0;
//     int powerValue = 0;
//     int ry = 0, desiredVelocity = 0;
//     int stepAmplitude = IMPULSE_STRENGTH;
//     int16_t powerBuffer = 0;
//     int16_t velocityBuffer = 0, angleBuffer = 0, torqueBuffer = 0;
//     bool impulse = true, measureStart = false;

//     while(true){
//         timeStart = us_ticker_read();

//         if ((timeStart - loopTimer) / 1000 > 15){
//            loopTimer = timeStart;
//             led = !led;
//             ledbuiltin = !ledbuiltin;

//             refLoop++;
//             if (refLoop >= 5){
//                 refereeThread(&referee);
//                 refLoop = 0;
//                 led2 = !led2;
//             }
//             prevL = switL;
//             remoteRead();

//             switL = (remote.leftSwitch() == Remote::SwitchState::UP);
//             bool switLDown = (remote.leftSwitch() == Remote::SwitchState::DOWN);
//             bool switR = (remote.rightSwitch() == Remote::SwitchState::UP);
//             bool switRDown = (remote.rightSwitch() == Remote::SwitchState::DOWN);

//             if (switL) { // step response
//                 powerBuffer = stepAmplitude;
//             }
//             else if (switLDown) { // inverted step
//                 powerBuffer = -stepAmplitude;
//             }
//             else if (switR) {  // arbitrary response
//                 ry = remote.rightY();
// //                powerBuffer = -IMPULSE_STRENGTH + ((float) ry+REMOTE_MAX)/(REMOTE_MAX*2) * (2*IMPULSE_STRENGTH);
//                 powerBuffer = IMPULSE_STRENGTH*(2/(1 + exp(.002*ry)) - 1);
//             }
//             else if (switRDown) { // constant power increase
//                 if (!measureStart) {
//                     measureStart = true;
//                     powerBuffer = 0;
//                 }
//                 else if (powerBuffer > IMPULSE_STRENGTH) {
//                     powerBuffer = 0;
//                 }
//                 else
//                     powerBuffer++;
//             }
//             else {
//                 measureStart = false;
//                 powerBuffer = 0;
//             }

//             testMot->setPower(powerBuffer);
//             testMot2 ? testMot2->setPower(powerBuffer) : (void)0; // for double motor on sentry yaw
//             testMot3 ? testMot3->setPower(powerBuffer) : (void)0; // for chassis motor
//             testMot4 ? testMot4->setPower(powerBuffer) : (void)0; // for chassis motor
//             angleBuffer = (*testMot)>>ANGLE;
//             velocityBuffer = (*testMot)>>VELOCITY;
//             torqueBuffer = (*testMot)>>TORQUE;

//             if (switL || switR || switLDown || switRDown)  // print only when test is active
//                 if (position)
//                     printff("%d\t%d\n", powerBuffer, angleBuffer);
//                 else if (velocity)
//                     printff("%d\t%d\n", powerBuffer, velocityBuffer);
//             DJIMotor::s_sendValues();
//         }
// //        printff("working\n");
//         DJIMotor::s_getFeedback();
//         ThisThread::sleep_for(1ms);
//     }
// }



//// Test drive motors
//#include "main.h"
//
//DigitalOut led(L26);
//DigitalOut led2(L27);
//DigitalOut led3(L25);
//
//I2C i2c(I2C_SDA, I2C_SCL);
//
//  //DEFINE MOTORS, ETC
//const int RPM_MAX = 9000;
//const int REMOTE_MAX = 660;
//const int RPM_REMOTE_RATIO = RPM_MAX / REMOTE_MAX;
//
//DJIMotor* frontleft = new DJIMotor(1, CANHandler::CANBus::CANBUS_1, motorType::M3508, "FL");
//
//DJIMotor* frontright = new DJIMotor(2, CANHandler::CANBus::CANBUS_1, motorType::M3508, "FR");
//
//DJIMotor* backleft = new DJIMotor(3, CANHandler::CANBus::CANBUS_1, motorType::M3508, "BL");
//
//DJIMotor* backright = new DJIMotor(4, CANHandler::CANBus::CANBUS_1, motorType::M3508, "BR");
//
//
//int main(){
//
//    //assigning can handler objects to motor class.
//    DJIMotor::s_setCANHandlers(&canHandler1,&canHandler2, false, false);
//
//    //getting initial feedback.
//    DJIMotor::s_getFeedback();
//
//    unsigned long loopTimer_u = us_ticker_read();
//    unsigned long timeEnd_u;
//    unsigned long timeStart_u;
//
//    frontleft->setSpeedPID(5.2908, .010269, 0);
//    frontright->setSpeedPID(5.2908, .010269, 0);
//    backleft->setSpeedPID(5.2908, .010269, 0);
//    backright->setSpeedPID(5.2908, .010269, 0);
//
//    int refLoop = 0;
//
//    //DEFINE PIDs AND OTHER CONSTANTS
//
//    int flspeed = 0;
//    int frspeed = 0;
//    int blspeed = 0;
//    int brspeed = 0;
//
//    while(true){ //main loop
//        timeStart_u = us_ticker_read();
//
//        //inner loop runs every 25ms
//        if((timeStart_u - loopTimer_u) / 1000 > 25) {
//            loopTimer_u = timeStart_u;
//            led = !led; //led blink tells us how fast the inner loop is running
//
//            if (refLoop >= 5) { //ref code runs 5 of every inner loop,
//                refLoop = 0;
//                refereeThread(&referee);
//
//                printff("%d %d %d %d", *frontleft>>VELOCITY, *frontright>>VELOCITY, *backleft>>VELOCITY, *backright>>VELOCITY);
//                printff(" %d %d %d %d\n", flspeed, frspeed, blspeed, brspeed);
//            }
//            refLoop ++;
//
//            remoteRead(); //reading data from remote
//
//            //MAIN CODE
//            flspeed = 0;
//            frspeed = 0;
//            blspeed = 0;
//            brspeed = 0;
//
//            int lx = 0;
//            int ly = 0;
//            int rx = 0;
//
//            if (abs(remote.leftY()) > 20) {
//                ly = remote.leftY();
//            }
//            if (abs(remote.leftX()) > 20) {
//                lx = remote.leftX();
//            }
//            if (abs(remote.rightX()) > 20) {
//                rx = remote.rightX();
//            }
//
//            flspeed = ((ly + lx + rx) * RPM_REMOTE_RATIO) % RPM_MAX;
//
//            frspeed = ((-ly + lx + rx) * RPM_REMOTE_RATIO) % RPM_MAX;
//
//            blspeed = ((ly - lx + rx) * RPM_REMOTE_RATIO) % RPM_MAX;
//
//            brspeed = ((-ly - lx + rx) * RPM_REMOTE_RATIO) % RPM_MAX;
//
//            frontleft->setSpeed(flspeed);
//            frontright->setSpeed(frspeed);
//            backleft->setSpeed(blspeed);
//            backright->setSpeed(brspeed);
//
//            //MOST CODE DOESNT NEED TO RUN FASTER THAN EVERY 25ms
//
//            timeEnd_u = us_ticker_read();
//
//            DJIMotor::s_sendValues();
//        }
//
//        //FEEDBACK CODE DOES NEED TO RUN FASTER THAN 1MS
//        //OTHER QUICK AND URGENT TASKS GO HERE
//
//        DJIMotor::s_getFeedback();
//        ThisThread::sleep_for(1ms);
//    }
//}















#include "main.h"
#include "subsystems/ChassisSubsystem.h"

DigitalOut led(L25);
DigitalOut led2(L26);
DigitalOut led3(L27);
DigitalOut ledbuiltin(LED1);

//CONSTANTS
constexpr float LOWERBOUND = 12.0;
constexpr float UPPERBOUND = -25.0;

constexpr float BEYBLADE_OMEGA = 3.0;

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
I2C i2c(I2C_SDA, I2C_SCL);
// BNO055 imu(i2c, IMU_RESET, MODE_IMU);
AS5600 yawEncoder(i2c, AS5600_DEFAULT_ADDRESS, AS5600_MODE_DEGREES);

//CHASSIS DEFINING
DJIMotor indexer(2, CANHandler::CANBUS_1, M3508, "Indexer");


#ifdef USE_IMU
// BNO055_ANGULAR_POSITION_typedef imuAngles;
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


    // init the encoder to 0 (i think this is the right way to do it)
    yawEncoder.setZPosition(0);

    /*
    * MOTORS SETUP AND PIDS
    */
    //INDEXER
    indexer.setSpeedPID(2.21, 0, 5.2);
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
            led3 = !led3;
            refLoop++;
            
            // imu.get_angular_position_quat(&imuAngles);


            if (refLoop >= 5){
                refereeThread(&referee);
                refLoop = 0; 
                led = referee.readable(); //referee.readable() is true if referee data is available

                //POWER LIMIT OVERRIDE INCASE
                if(robot_status.chassis_power_limit < 10){
                    chassis_power_limit = 50;
                }else{
                    chassis_power_limit = robot_status.chassis_power_limit;
                }
                
            }

            Remote::SwitchState previous_mode = remote.leftSwitch();
            bool prevM = remote.getMouseL();
            remoteRead();

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

            
            //INDEXER CODE
            if (remote.leftSwitch() == Remote::SwitchState::MID  || (remote.getMouseL() && remote.leftSwitch() == Remote::SwitchState::MID)){
                if (shootReady){
                    shootReady = false;
                   if(robot_status.shooter_barrel_heat_limit < 10 || power_heat_data.shooter_42mm_barrel_heat < robot_status.shooter_barrel_heat_limit - 110) {
                       shoot = true;
                   }else {
                       shoot = false;
                   }

                    shootTargetPosition = 8192 * 3 + (indexer>>MULTITURNANGLE);
                    shootTimer = us_ticker_read()/1000;
                }
            } else {
                //SwitchState state set to mid/down/unknown
                shootReady = true;
                indexer.setSpeed(0);
            }

            // burst fire, turn the indexer to shoot 3-5 balls a time and stop indexer
            // only shoot when left switch changes from down/unknown/mid to up
            // if left switch remains at up state, indexer stops after 3-5 balls
            if (shoot){
                    bool indexerOn = false;
                    //indexer
                    if (us_ticker_read()/1000 - shootTimer < 300){
                        indexer.setSpeed(3000);
                    } else {
                        indexer.setSpeed(0);
                        indexerOn = true;
                    }
                    if (indexerOn){
                        shoot = false;
                    }
            } 
            else {
                indexer.setSpeed(0);
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

                // printff("limit: %d heat: %d\n", robot_status.shooter_barrel_heat_limit, power_heat_data.shooter_42mm_barrel_heat);
                // printff("ID:%d LVL:%d HP:%d MAX_HP:%d\n", robot_status.robot_id, robot_status.robot_level, robot_status.current_HP, robot_status.maximum_HP);
               
                #ifdef USE_IMU
                //printff("yaw_des_v:%d yaw_act_v:%d", yawVelo, yaw>>VELOCITY);
                // printff("yaw_des:%.3f yaw_act:%.3f\n", yaw_desired_angle, imuAngles.yaw + 180);
                int val = 0;
                yawEncoder.getZPosition(&val);
                printff("encoder pos: %d\n", val);
                #else
                // printff("yaw_des_v:%d yaw_act_v:%d\n", yawVelo, yaw>>VELOCITY);
                //printff("yaw_des:%.3f yaw_act:%.3f [%d]\n", yaw_desired_angle, yaw_current_angle, yaw>>ANGLE);
                #endif
                // printff("ref: %d\n", robot_status.current_HP);
                // printff("Indexer: %c\n", indexer.isConnected() ? 'y' : 'n');
                // printff("P: %hd, V: %hd, T: %d, trq: %hd\n", indexer>>POWEROUT, indexer>>VELOCITY, indexer>>TEMPERATURE, indexer>>TORQUE);
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
                // printff("IMU %.3f %.3f %.3f\n",imuAngles.yaw, imuAngles.pitch, imuAngles.roll);
                #endif

                // WheelSpeeds ac = Chassis.getWheelSpeeds();
                // ChassisSpeeds test = {jx * Chassis.m_OmniKinematicsLimits.max_Vel,
                //                     jy * Chassis.m_OmniKinematicsLimits.max_Vel,
                //                     -BEYBLADE_OMEGA};
                // WheelSpeeds ws = Chassis.chassisSpeedsToWheelSpeeds(test);
                
                // printff("CS: %.1f %.1f %.1f ", cs.vX, cs.vY, cs.vOmega);
                // printff("DS: %.1f %.1f %.1f\n", test.vX, test.vY, test.vOmega);

                // printff("CH: %.2f %.2f %.2f %.2f ", ws.LF,ws.RF,ws.LB,ws.RB);
                // printff("A: %.2f %.2f %.2f %.2f\n", ac.LF,ac.RF,ac.LB,ac.RB);
                // printff("A_RAW: %d %d %d %d\n", 
                //     Chassis.getMotor(ChassisSubsystem::LEFT_FRONT).getData(VELOCITY), 
                //     Chassis.getMotor(ChassisSubsystem::RIGHT_FRONT).getData(VELOCITY), 
                //     Chassiss.getMotor(ChassisSubsystem::LEFT_BACK).getData(VELOCITY), 
                //     Chassis.getMotor(ChassisSubsystem::RIGHT_BACK).getData(VELOCITY));
                // printff("A_MPS: %.2f %.2f %.2f %.2f\n", 
                //     Chassis.getMotorSpeed(ChassisSubsystem::LEFT_FRONT, ChassisSubsystem::METER_PER_SECOND), 
                //     Chassis.getMotorSpeed(ChassisSubsystem::RIGHT_FRONT, ChassisSubsystem::METER_PER_SECOND), 
                //     Chassis.getMotorSpeed(ChassisSubsystem::LEFT_BACK, ChassisSubsystem::METER_PER_SECOND), 
                //     Chassis.getMotorSpeed(ChassisSubsystem::RIGHT_BACK, ChassisSubsystem::METER_PER_SECOND));
            }

            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}