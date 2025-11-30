//
// Created by ankit on 1/31/23.
//

// #ifndef TR_EMBEDDED_MAIN_H
// #define TR_EMBEDDED_MAIN_H

// #include "mbed.h"
// // #include "rtos"

// #include "algorithms/PID.h"
// #include "algorithms/TRMutex.h"
// #include "peripherals/imu/BNO055.h"
// #include "peripherals/oled/SSD1308.h"
// // #include "subsystems/Chassis.h"
// #include "subsystems/ChassisSubsystem.h"

// #include "motor/PWMMotor.h"
// #include "motor/DJIMotor.h"

// #include "communications/CANHandler.h"
// #include "communications/DJIRemote.h"
// #include "communications/SerialCommunication.h"
// #include "communications/referee/ref_serial.h"
// #include "communications/referee/ref_ui.h"
// #include "communications/referee/ref_operations.h"
// #include "communications/jetson/Jetson.h"

// #include <cstring>
// #define ESP_DEBUG 1 // 1 true, 0 false

// #define OLED_SDA PB_7
// #define OLED_SCL PB_8
// #define L25 PB_0
// #define L26 PC_1
// #define L27 PC_0
// #define USE_IMU

// static Remote remote(PA_10);
// static BufferedSerial referee(PC_10, PC_11, 115200); // Nucleo board: top left male pins.

// CANHandler canHandler1(PA_11, PA_12);
// CANHandler canHandler2(PB_12, PB_13);
// // I2C i2c(OLED_SDA, OLED_SCL);
// // SSD1308 oled(&i2c, 0x78);

// DigitalIn jumperPC9(PC_9);
// DigitalIn jumperPB1(PB_1);

// DigitalIn userButton(BUTTON1);
// TRMutex printer;
// Mutex mutex_test;
// Thread refThread;
// Thread imuThread;
// static BufferedSerial bc(PA_0, PA_1, 115200);
// static BufferedSerial usbSerial(USBTX, USBRX, 115200);
// // static SPI spi_imu(PB_15, PB_14, PA_9, PB_9); // MOSI, MISO, SCLK, NSS

// // LED Definitions
// DigitalOut led(L25);
// DigitalOut led2(L26);
// DigitalOut led3(L27);
// DigitalOut ledbuiltin(LED1);

// // CHASSIS Definitions
// I2C i2c(I2C_SDA, I2C_SCL);
// BNO055 imu(i2c, IMU_RESET, MODE_IMU);

// // CV STUFF
// static BufferedSerial bcJetson(PC_12, PD_2, 921600);  //JETSON PORT
// Jetson_read_data jetson_received_data;
// Jetson_read_odom jetson_received_odom;
// Jetson_send_ref jetson_send_ref;
// Jetson_send_data jetson_send_data;

// // imu
// #ifdef USE_IMU
// BNO055_ANGULAR_POSITION_typedef imuAngles;
// #endif

// // Variables for burst fire
// unsigned long timeSure;
// unsigned long prevTimeSure;
// bool shoot = false;
// int shootTargetPosition = 36*8190;
// bool shootReady = false;
// int remoteTimer = 0;

// // Remote control variables
// float scalar = 1;
// float jx = 0; // -1 to 1
// float jy = 0; // -1 to 1
// // Pitch, Yaw
// float jpitch = 0; // -1 to 1
// float jyaw = 0; // -1 to 1
// float myaw = 0;
// float mpitch = 0;
// int pitchVelo = 0;
// // joystick tolerance
// float tolerance = 0.05;
// // Keyboard Driving
// float mult = 0.7;
// float omega_speed = 0;
// float max_linear_vel = 0;

// //DEGREES PER SECOND AT MAX
// constexpr float JOYSTICK_SENSITIVITY_YAW_DPS = 180.0;
// constexpr float JOYSTICK_SENSITIVITY_PITCH_DPS = 180.0;

// // Mouse sensitivity initialized
// constexpr float MOUSE_SENSITIVITY_YAW_DPS = 10.0;
// constexpr float MOUSE_SENSITIVITY_PITCH_DPS = 10.0;

// // GENERAL VARIABLES
// // drive and shooting mode
// char drive = 'o'; //default o when using joystick
// char shot = 'o'; //default o when using joystick

// // turret controls variables
// float yaw_desired_angle = 0;
// float yaw_current_angle = 0;
// float pitch_current_angle = 0;
// float pitch_desired_angle = 0;

// // ref variables
// uint16_t chassis_power_limit;

// // timers
// unsigned long timeStart;
// unsigned long timeStartCV;
// unsigned long timeStartRef;
// unsigned long timeStartImu;
// unsigned long loopTimer = us_ticker_read();
// unsigned long loopTimerCV = loopTimer;
// unsigned long loopTimerRef = loopTimer;
// unsigned long loopTimerImu = loopTimer;
// float elapsedms;

// int readResult = 0;
// bool cv_enabled = false;
// char cv_shoot_status = 0;

// void updatePriority(priorityLevels desiredLevel)
// {
//     printer.updatePriority(desiredLevel);
// }

// void printfESP(const char *format, ...){
//     char temp[100];
//     va_list args;
//     va_start(args, format);
//     vsnprintf(temp, 100, format, args);
//     bc.write(temp, 100);
//     va_end(args);
// }

// void printff(const char *format, ...)
// {
//     char temp[50];
//     va_list args;
//     va_start(args, format);
//     int len = vsnprintf(temp, 50, format, args);
//     if (len > 0)
//        usbSerial.write(temp, len);
//     va_end(args);
// }

// inline static void remoteRead()
// {
//     remote.read();

//     //Keyboard-based drive and shoot mode
//     if(remote.keyPressed(Remote::Key::R)){
//         drive = 'm';
//     }else if(remote.keyPressed(Remote::Key::E)){
//         drive = 'u';
//     }else if(remote.keyPressed(Remote::Key::Q)){
//         drive = 'd';
//     }
//     if(remote.keyPressed(Remote::Key::V)){
//         shot = 'm';
//     }else if(remote.keyPressed(Remote::Key::C)){
//         shot = 'd';
//     }

//     if(remote.getMouseR() || remote.leftSwitch() == Remote::SwitchState::MID){
//         cv_enabled = true;
//     }else if(!remote.getMouseR() ){
//         cv_enabled = false;
//     }

//     //Driving input
//     scalar = 1;
//     jx = remote.leftX() / 660.0 * scalar; // -1 to 1
//     jy = remote.leftY() / 660.0 * scalar; // -1 to 1
//     //Pitch, Yaw
//     jpitch = remote.rightY() / 660.0 * scalar; // -1 to 1
//     jyaw = remote.rightX() / 660.0 * scalar; // -1 to 1

//     myaw = remote.getMouseX();
//     mpitch = -remote.getMouseY();

//     jx = (abs(jx) < tolerance) ? 0 : jx;
//     jy = (abs(jy) < tolerance) ? 0 : jy;
//     jpitch = (abs(jpitch) < tolerance) ? 0 : jpitch;
//     jyaw = (abs(jyaw) < tolerance) ? 0 : jyaw;

//     // Shift to make robot go slower
//     if (remote.keyPressed(Remote::Key::SHIFT)) {
//         mult = 0.5;
//     }
//     if(remote.keyPressed(Remote::Key::CTRL)){
//         mult = 1;
//     }

//     jx += mult * ((remote.keyPressed(Remote::Key::D) ? 1 : 0) + (remote.keyPressed(Remote::Key::A) ? -1 : 0));
//     jy += mult * ((remote.keyPressed(Remote::Key::W) ? 1 : 0) + (remote.keyPressed(Remote::Key::S) ? -1 : 0));

//     float j_hypo = sqrt(jx * jx + jy * jy);
//     if(j_hypo > 1.0){
//         jx = jx / j_hypo;
//         jy = jy / j_hypo;
//     }
//     //Bounding the four j variables
//     jx = max(-1.0F, min(1.0F, jx));
//     jy = max(-1.0F, min(1.0F, jy));
//     jpitch = max(-1.0F, min(1.0F, jpitch));
//     jyaw = max(-1.0F, min(1.0F, jyaw));

//     max_linear_vel = -1.24 + 0.0513 * chassis_power_limit + -0.000216 * (chassis_power_limit * chassis_power_limit);
//     // float max_omega = 0.326 + 0.0857 * chassis_power_limit + -0.000183 * (chassis_power_limit * chassis_power_limit);
//     float max_omega = 4.8;

//     if(remote.keyPressed(Remote::Key::CTRL)){
//         jx = 0.0;
//         jy = 0.0;
//         max_omega = 6.1;
//     }

//     float linear_hypo = sqrtf(jx * jx + jy * jy);
//     if(linear_hypo > 0.8){
//         linear_hypo = 0.8;
//     }

//     float available_beyblade = 1.0 - linear_hypo;
//     omega_speed = max_omega * available_beyblade;
// }

// static void remotePrint()
// {
//     // for (int i = 0; i < 7; i++)
//     //     printf("%d\t", dats[i]);
//     printf("%d\t%d\t%d\t%d\t%d\t%d\t", remote.leftX(), remote.leftY(), remote.rightX(), remote.rightY(), remote.leftSwitch(), remote.rightSwitch());
//     printf("\n");
// }

// // CANHandler canPorts(PA_11,PA_12,PB_12,PB_13);

// // Thread threadingRemote(osPriorityNormal);
// // Thread threadingReferee(osPriorityLow);
// // Thread threadingPrint(osPriorityBelowNormal);
// // Thread threadingLogger(osPriorityLow);

// float floatmod(float input, float mod){
//     float f = input;
//     while(f < 0){
//         f += mod;
//     }
//     while(f >= mod){
//         f -= mod;
//     }
//     return f;
// }

// constexpr float MAX_METERSPERSSECOND_60W = 1.0;
// constexpr float MAX_RADIANSPERSSECOND_60W = 6.3;
// float BeybladeModulation(ChassisSpeeds cs){
//     float v = sqrt(cs.vX * cs.vX + cs.vY * cs.vY);
//     if(v < MAX_METERSPERSSECOND_60W){
//         return (MAX_METERSPERSSECOND_60W - v) * (MAX_RADIANSPERSSECOND_60W * 0.8);
//     }else{
//         return 0;
//     }
// }

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

// #endif // TR_EMBEDDED_MAIN_H
