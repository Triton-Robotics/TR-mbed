#include "main.h"
#include "subsystems/ChassisSubsystem.h"

I2C i2c(I2C_SDA, I2C_SCL);
BNO055 imu(i2c, IMU_RESET, MODE_IMU);
DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);
DigitalOut ledbuiltin(LED1);
// ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.22617); // radius is 9 in - DISABLED FOR RAW TESTING
DJIMotor feeder(5, CANHandler::CANBUS_1, M2006);
DJIMotor RFLYWHEEL(6, CANHandler::CANBUS_1, M3508,"RightFly");
DJIMotor LFLYWHEEL(7, CANHandler::CANBUS_1, M3508,"LeftFly");

// RAW MOTOR CONTROL - NO PID, NO CHASSIS SUBSYSTEM
DJIMotor motor1(1, CANHandler::CANBUS_1, M3508, "Motor1");
DJIMotor motor2(2, CANHandler::CANBUS_1, M3508, "Motor2");
DJIMotor motor3(3, CANHandler::CANBUS_1, M3508, "Motor3");
DJIMotor motor4(4, CANHandler::CANBUS_1, M3508, "Motor4");

#define IMPULSE_DT 100
#define IMPULSE_STRENGTH 16383

int main(){    

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback(); 
    
    bool send_power = false;

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    unsigned long loopTimer2 = us_ticker_read();
    int powerCnt = 0;
    int refLoop = 0;    
    
    // Set up feeder PID for C610/M2006
    feeder.setSpeedPID(4, 0, 1);
    feeder.setSpeedIntegralCap(8000);
    feeder.setSpeedOutputCap(16000);

    LFLYWHEEL.setSpeedPID(7.1849, 0.000042634, 0);
    RFLYWHEEL.setSpeedPID(7.1849, 0.000042634, 0);
    
    while(true){
        timeStart = us_ticker_read();       

        if ((timeStart - loopTimer) / 1000 > 1){
            
            float elapsedms = (timeStart - loopTimer) / 1000;
            loopTimer = timeStart;
            led = !led;
            ledbuiltin = !ledbuiltin;            
            
            refLoop++;
            if (refLoop >= 5){                
                refereeThread(&referee);
                refLoop = 0;
                led2 = !led2;            
            
            }    
            
            // Chassis.periodic();
            remoteRead(); 

             //Driving input
            float scalar = 1;
            float jx = remote.leftX() / 660.0 * scalar; // -1 to 1
            float jy = remote.leftY() / 660.0 * scalar; // -1 to 1
            //Pitch, Yaw
            float jpitch = remote.rightY() / 660.0 * scalar; // -1 to 1
            float jyaw = remote.rightX() / 660.0 * scalar; // -1 to 

            //joystick tolerance
            float tolerance = 0.05; 
            jx = (abs(jx) < tolerance) ? 0 : jx;
            jy = (abs(jy) < tolerance) ? 0 : jy;
            jpitch = (abs(jpitch) < tolerance) ? 0 : jpitch;
            jyaw = (abs(jyaw) < tolerance) ? 0 : jyaw;
                        
            //Bounding the four j variables
            jx = max(-1.0F, min(1.0F, jx));
            jy = max(-1.0F, min(1.0F, jy));
            jpitch = max(-1.0F, min(1.0F, jpitch));
            jyaw = max(-1.0F, min(1.0F, jyaw));           
            // Chassis.setChassisSpeeds({jx * Chassis.m_OmniKinematicsLimits.max_Vel,
            //                               jy * Chassis.m_OmniKinematicsLimits.max_Vel,
            //                               0 * Chassis.m_OmniKinematicsLimits.max_vOmega}, 100,
            //                               ChassisSubsystem::YAW_ORIENTED);
            // motor1.setPower(remote.leftX() * 8000 / 660);
            // motor2.setPower(remote.leftY() * 8000 / 660);
            // motor3.setPower(remote.rightX() * 8000 / 660);
            // motor4.setPower(remote.rightY() * 8000 / 660);
            static int statusCounter = 0;
            if(statusCounter++ % 50 == 0) {
                if (send_power == true) {
                    send_power = false;
                }
                else if (send_power == false) {
                    send_power = true;
                }
            }


            if (send_power) {
                motor1.setSpeed(130*19);
                motor2.setSpeed(130*19);
                motor3.setSpeed(130*19);
                motor4.setSpeed(130*19);
                feeder.setPower(2500);
                RFLYWHEEL.setSpeed(3000);
                LFLYWHEEL.setSpeed(3000);
            } else {
                motor1.setSpeed(-130*19);
                motor2.setSpeed(-130*19);
                motor3.setSpeed(-130*19);
                motor4.setSpeed(-130*19);
                feeder.setPower(2500);
                RFLYWHEEL.setSpeed(-3000);
                LFLYWHEEL.setSpeed(-3000);
            }


            // if (send_power) {
            //     motor1.setPower(1000);
            //     motor2.setPower(1000);
            //     motor3.setPower(1000);
            //     motor4.setPower(1000);
            //     feeder.setSpeed(130*19);
            //     RFLYWHEEL.setPower(300);
            //     LFLYWHEEL.setPower(300);
            // } else {
            //     motor1.setPower(-1000);
            //     motor2.setPower(-1000);
            //     motor3.setPower(-1000);
            //     motor4.setPower(-1000);
            //     feeder.setPower(0);
            //     RFLYWHEEL.setPower(-300);
            //     LFLYWHEEL.setPower(-300);
            // }
            
                      
  
            // testMot.setSpeed(remote.leftX() * 3000 / 660);  // Use speed control instead of raw power
            
            // Debug PID behavior - Status output every 100ms
            // static int statusCounte = 0;
            // if(statusCounte++ % 100 == 0) {
            //     printff("V=%d P=%d | CHILL?:%s\n",
            //     feeder.getData(VELOCITY), feeder.getData(POWEROUT), 
            //     feeder.isConnected() ? "Y" : "N");
            // }

            // feeder.setSpeed((remote.rightX()/660) * 3000);
            // RFLYWHEEL.setSpeed((remote.rightY()/660) * 3000);
            // LFLYWHEEL.setSpeed((remote.rightY()/660) * 3000);
            
            // Debug output - reduce frequency to save memory
            // static int debugCounter = 0;
            // if(debugCounter++ % 50 == 0) {  // Only print every 50 loops
            //     printff("%d %d %d %d\n", 
            //     remote.rightX(),
            //     remote.rightY(),
            //     remote.leftX(),
            //     remote.leftY());
            // }
            
            // printff("%d,%d\n", testMot>>VELOCITY, testMot>>POWEROUT);            
            
        //      static int statusCounter = 0;
        //      if(statusCounter++ % 50 == 0) {

            DJIMotor::s_sendValues();  // Enable debug output
        //             printff("%d %d %d %d\n",
        // motor1.getData(VELOCITY)/24,
        // motor2.getData(VELOCITY)/24,
        // motor3.getData(VELOCITY)/24,
        // motor4.getData(VELOCITY)/24);
        //             }
        printff("%d\n", feeder.getData(VELOCITY));
        }


        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
                    // Check if all 4 motors are connected
            // DJIMotor* motors[4] = {&motor1, &motor2, &motor3, &motor4};
            // bool allConnected = DJIMotor::s_theseConnected(motors, 4, true);
            // if (!allConnected) {
            //     printff("WARNING: Some motors disconnected!\n");
            // }
    }
}























// #include "main.h"
// #include "subsystems/ChassisSubsystem.h"

// DigitalOut led(L25);
// DigitalOut led2(L26);
// DigitalOut led3(L27);
// DigitalOut ledbuiltin(LED1);

// //CONSTANTS
// constexpr float LOWERBOUND = 12.0;
// constexpr float UPPERBOUND = -25.0;

// constexpr float BEYBLADE_OMEGA = 3.0;

// // constexpr float JOYSTICK_SENSITIVITY_YAW = 1.0/90;
// // constexpr float JOYSTICK_SENSITIVITY_PITCH = 1.0/150;
// // constexpr float MOUSE_SENSITIVITY_YAW = 1.0/5;
// // constexpr float MOUSE_SENSITIVITY_PITCH = 1.0/5;

// //DEGREES PER SECOND AT MAX
// constexpr float JOYSTICK_SENSITIVITY_YAW_DPS = 180.0; 
// constexpr float JOYSTICK_SENSITIVITY_PITCH_DPS = 180.0;
// constexpr float MOUSE_SENSITIVITY_YAW_DPS = 1.0;
// constexpr float MOUSE_SENSITIVITY_PITCH_DPS = 1.0;

// constexpr int OUTER_LOOP_DT_MS = 15;

// constexpr int PRINT_FREQUENCY = 20; //the higher the number, the less often

// constexpr float CHASSIS_FF_KICK = 0.065;

// #define USE_IMU
// I2C i2c(I2C_SDA, I2C_SCL);
// // BNO055 imu(i2c, IMU_RESET, MODE_IMU);
// // AS5600 yawEncoder(i2c, AS5600_DEFAULT_ADDRESS, AS5600_MODE_DEGREES);

// //CHASSIS DEFINING
// DJIMotor indexer(2, CANHandler::CANBUS_1, M3508, "Indexer");


// #ifdef USE_IMU
// // BNO055_ANGULAR_POSITION_typedef imuAngles;
// #endif

// int calculateDeltaYaw(int ref_yaw, int beforeBeybladeYaw)
// {
//     int deltaYaw = beforeBeybladeYaw - ref_yaw;
//     if (abs(deltaYaw) > 180)
//     {
//         if (deltaYaw > 0)
//             deltaYaw -= 360;
//         else
//             deltaYaw += 360;
//     }
//     return deltaYaw;
// }

// int main(){

//     DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
//     DJIMotor::s_sendValues();
//     DJIMotor::s_getFeedback();


//     // init the encoder to 0 (i think this is the right way to do it)
//     // yawEncoder.setZPosition(0);

//     /*
//     * MOTORS SETUP AND PIDS
//     */
//     //INDEXER
//     indexer.setSpeedPID(2.21, 0, 5.2);
//     indexer.setSpeedIntegralCap(8000);
//     //Cascading PID for indexer angle position control. Surely there are better names then "sure"...
//     PID sure(0.5,0,0.4);
//     sure.setOutputCap(4000);
//     //Variables for burst fire
//     unsigned long timeSure;
//     unsigned long prevTimeSure;
//     unsigned long shootTimer;
//     bool shoot = false;
//     int shootTargetPosition = 36*8190 ;
//     bool shootReady = false;

//     //GENERAL VARIABLES

//     //drive and shooting mode
//     char drive = 'o'; //default o when using joystick
//     char shot = 'o'; //default o when using joystick
//     char driveMode = 'j'; //j for joystick, m for mouse/keyboard

//     //user button (doesnt work?)
//     bool userButton;
//     bool prev_userButton;

//     //ref variables
//     uint16_t chassis_buffer;
//     uint16_t chassis_power_limit;
//     uint16_t barrel_heat1;
//     uint16_t barrel_heat_max1;

//     unsigned long timeStart;
//     unsigned long loopTimer = us_ticker_read();
//     int refLoop = 0;
//     int printLoop = 0;

//     ChassisSpeeds cs;

//     while(true){
//         timeStart = us_ticker_read();

//         if ((timeStart - loopTimer) / 1000 > OUTER_LOOP_DT_MS){
//             float elapsedms = (timeStart - loopTimer) / 1000;
//             loopTimer = timeStart;
//             led3 = !led3;
//             refLoop++;
            
//             // imu.get_angular_position_quat(&imuAngles);

//             if (refLoop >= 5){
//                 refereeThread(&referee);
//                 refLoop = 0; 
//                 led = referee.readable(); //referee.readable() is true if referee data is available

//                 //POWER LIMIT OVERRIDE INCASE
//                 if(robot_status.chassis_power_limit < 10){
//                     chassis_power_limit = 50;
//                 }else{
//                     chassis_power_limit = robot_status.chassis_power_limit;
//                 }
                
//             }

//             Remote::SwitchState previous_mode = remote.leftSwitch();
//             bool prevM = remote.getMouseL();
//             remoteRead();

//             //Keyboard-based drive and shoot mode
//             if(remote.keyPressed(Remote::Key::R)){
//                 drive = 'm';
//             }else if(remote.keyPressed(Remote::Key::E)){
//                 drive = 'u';
//             }else if(remote.keyPressed(Remote::Key::Q)){
//                 drive = 'd';        
//             }
//             if(remote.keyPressed(Remote::Key::C)){
//                 shot = 'm';
//             }else if(remote.keyPressed(Remote::Key::V)){
//                 shot = 'u';
//             }else if(remote.keyPressed(Remote::Key::B)){
//                 shot = 'd';        
//             }

//             //Driving input
//             float scalar = 1;
//             float jx = remote.leftX() / 660.0 * scalar; // -1 to 1
//             float jy = remote.leftY() / 660.0 * scalar; // -1 to 1
//             //Pitch, Yaw
//             float jpitch = remote.rightY() / 660.0 * scalar; // -1 to 1
//             float jyaw = remote.rightX() / 660.0 * scalar; // -1 to 1

//             //joystick tolerance
//             float tolerance = 0.05; 
//             jx = (abs(jx) < tolerance) ? 0 : jx;
//             jy = (abs(jy) < tolerance) ? 0 : jy;
//             jpitch = (abs(jpitch) < tolerance) ? 0 : jpitch;
//             jyaw = (abs(jyaw) < tolerance) ? 0 : jyaw;
            
//             //Keyboard Driving
//             float mult = 1;
//             jx += mult * ((remote.keyPressed(Remote::Key::D) ? 1 : 0) + (remote.keyPressed(Remote::Key::A) ? -1 : 0));
//             jy += mult * ((remote.keyPressed(Remote::Key::W) ? 1 : 0) + (remote.keyPressed(Remote::Key::S) ? -1 : 0));
            
//             //Bounding the four j variables
//             jx = max(-1.0F, min(1.0F, jx));
//             jy = max(-1.0F, min(1.0F, jy));
//             jpitch = max(-1.0F, min(1.0F, jpitch));
//             jyaw = max(-1.0F, min(1.0F, jyaw));

            
//             //INDEXER CODE
//             if (remote.leftSwitch() == Remote::SwitchState::MID  || (remote.getMouseL() && remote.leftSwitch() == Remote::SwitchState::MID)){
//                 if (shootReady){
//                     shootReady = false;
//                    if(robot_status.shooter_barrel_heat_limit < 10 || power_heat_data.shooter_42mm_barrel_heat < robot_status.shooter_barrel_heat_limit - 110) {
//                        shoot = true;
//                    }else {
//                        shoot = false;
//                    }

//                     shootTargetPosition = 8192 * 3 + (indexer>>MULTITURNANGLE);
//                     shootTimer = us_ticker_read()/1000;
//                 }
//             } else {
//                 //SwitchState state set to mid/down/unknown
//                 shootReady = true;
//                 indexer.setSpeed(0);
//             }

//             // burst fire, turn the indexer to shoot 3-5 balls a time and stop indexer
//             // only shoot when left switch changes from down/unknown/mid to up
//             // if left switch remains at up state, indexer stops after 3-5 balls
//             if (shoot){
//                     bool indexerOn = false;
//                     //indexer
//                     if (us_ticker_read()/1000 - shootTimer < 300){
//                         indexer.setSpeed(3000);
//                     } else {
//                         indexer.setSpeed(0);
//                         indexerOn = true;
//                     }
//                     if (indexerOn){
//                         shoot = false;
//                     }
//             } 
//             else {
//                 indexer.setSpeed(0);
//             }


//             printLoop ++;
//             if (printLoop >= PRINT_FREQUENCY){
//                 printLoop = 0;
//                 //printff("%.3f Pitch\n", pitch_desired_angle);
//                 //printff("Prints:\n");
//                 //printff("lX:%.1f lY:%.1f rX:%.1f rY:%.1f lS:%d rS:%d\n", remote.leftX(), remote.leftY(), remote.rightX(), remote.rightY(), remote.leftSwitch(), remote.rightSwitch());
//                 //printff("jx:%.3f jy:%.3f jpitch:%.3f jyaw:%.3f\n", jx, jy, jpitch, jyaw);

//                 //printff("%.3f  %d\n", pitch_desired_angle, pitch.getData(ANGLE));
//                 //printff("%d\n", indexer.getData(POWEROUT));

//                 // printff("limit: %d heat: %d\n", robot_status.shooter_barrel_heat_limit, power_heat_data.shooter_42mm_barrel_heat);
//                 // printff("ID:%d LVL:%d HP:%d MAX_HP:%d\n", robot_status.robot_id, robot_status.robot_level, robot_status.current_HP, robot_status.maximum_HP);
               
//                 #ifdef USE_IMU
//                 //printff("yaw_des_v:%d yaw_act_v:%d", yawVelo, yaw>>VELOCITY);
//                 // printff("yaw_des:%.3f yaw_act:%.3f\n", yaw_desired_angle, imuAngles.yaw + 180);
//                 // printff("encoder pos: %d\n", yawEncoder.getZPosition());
//                 #else
//                 // printff("yaw_des_v:%d yaw_act_v:%d\n", yawVelo, yaw>>VELOCITY);
//                 //printff("yaw_des:%.3f yaw_act:%.3f [%d]\n", yaw_desired_angle, yaw_current_angle, yaw>>ANGLE);
//                 #endif
//                 // printff("ref: %d\n", robot_status.current_HP);
//                 // printff("Indexer: %c\n", indexer.isConnected() ? 'y' : 'n');
//                 // printff("P: %hd, V: %hd, T: %d, trq: %hd\n", indexer>>POWEROUT, indexer>>VELOCITY, indexer>>TEMPERATURE, indexer>>TORQUE);
//                 // printff("elap:%.5fms\n", elapsedms);
//                 // printff("Chassis: LF:%c RF:%c LB:%c RB:%c\n", 
//                 //     Chassis.getMotor(ChassisSubsystem::LEFT_FRONT).isConnected() ? 'y' : 'n', 
//                 //     Chassis.getMotor(ChassisSubsystem::RIGHT_FRONT).isConnected() ? 'y' : 'n', 
//                 //     Chassis.getMotor(ChassisSubsystem::LEFT_BACK).isConnected() ? 'y' : 'n', 
//                 //     Chassis.getMotor(ChassisSubsystem::RIGHT_BACK).isConnected() ? 'y' : 'n');
//                 // printff("Y:%c P:%c F_L:%c F_R:%c I:%c F:%c\n",
//                 //     yaw.isConnected() ? 'y' : 'n', 
//                 //     pitch.isConnected() ? 'y' : 'n', 
//                 //     LFLYWHEEL.isConnected() ? 'y' : 'n', 
//                 //     RFLYWHEEL.isConnected() ? 'y' : 'n',
//                 //     indexer.isConnected() ? 'y' : 'n',
//                 //     feeder.isConnected() ? 'y' : 'n');
//                 #ifdef USE_IMU
//                 // printff("IMU %.3f %.3f %.3f\n",imuAngles.yaw, imuAngles.pitch, imuAngles.roll);
//                 #endif

//                 // WheelSpeeds ac = Chassis.getWheelSpeeds();
//                 // ChassisSpeeds test = {jx * Chassis.m_OmniKinematicsLimits.max_Vel,
//                 //                     jy * Chassis.m_OmniKinematicsLimits.max_Vel,
//                 //                     -BEYBLADE_OMEGA};
//                 // WheelSpeeds ws = Chassis.chassisSpeedsToWheelSpeeds(test);
                
//                 // printff("CS: %.1f %.1f %.1f ", cs.vX, cs.vY, cs.vOmega);
//                 // printff("DS: %.1f %.1f %.1f\n", test.vX, test.vY, test.vOmega);

//                 // printff("CH: %.2f %.2f %.2f %.2f ", ws.LF,ws.RF,ws.LB,ws.RB);
//                 // printff("A: %.2f %.2f %.2f %.2f\n", ac.LF,ac.RF,ac.LB,ac.RB);
//                 // printff("A_RAW: %d %d %d %d\n", 
//                 //     Chassis.getMotor(ChassisSubsystem::LEFT_FRONT).getData(VELOCITY), 
//                 //     Chassis.getMotor(ChassisSubsystem::RIGHT_FRONT).getData(VELOCITY), 
//                 //     Chassiss.getMotor(ChassisSubsystem::LEFT_BACK).getData(VELOCITY), 
//                 //     Chassis.getMotor(ChassisSubsystem::RIGHT_BACK).getData(VELOCITY));
//                 // printff("A_MPS: %.2f %.2f %.2f %.2f\n", 
//                 //     Chassis.getMotorSpeed(ChassisSubsystem::LEFT_FRONT, ChassisSubsystem::METER_PER_SECOND), 
//                 //     Chassis.getMotorSpeed(ChassisSubsystem::RIGHT_FRONT, ChassisSubsystem::METER_PER_SECOND), 
//                 //     Chassis.getMotorSpeed(ChassisSubsystem::LEFT_BACK, ChassisSubsystem::METER_PER_SECOND), 
//                 //     Chassis.getMotorSpeed(ChassisSubsystem::RIGHT_BACK, ChassisSubsystem::METER_PER_SECOND));
//             }

//             DJIMotor::s_sendValues();
//         }
//         DJIMotor::s_getFeedback();
//         ThisThread::sleep_for(1ms);
//     }
// }