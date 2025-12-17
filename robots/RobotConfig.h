
#pragma once

#include "mbed.h"
// #include "rtos"

#include "util/algorithms/PID.h"
#include "util/algorithms/TRMutex.h"
#include "util/peripherals/imu/BNO055.h"
#include "util/peripherals/oled/SSD1308.h"

#include "util/motor/PWMMotor.h"
#include "util/motor/DJIMotor.h"

#include "util/communications/CANHandler.h"
#include "util/communications/DJIRemote.h"
#include "util/communications/SerialCommunication.h"
#include "util/communications/referee/ref_serial.h"
#include "util/communications/referee/ref_ui.h"
#include "util/communications/referee/ref_operations.h"
#include "util/communications/jetson/Jetson.h"


// #include "subsystems/Chassis.h"
#include "subsystems/Subsystem.h"
#include "subsystems/ChassisSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

#include <cstring>
#define ESP_DEBUG 1 // 1 true, 0 false

#define OLED_SDA PB_7
#define OLED_SCL PB_8
#define L25 PB_0
#define L26 PC_1
#define L27 PC_0
#define USE_IMU

namespace TR
{
    // TODO add a configure method to every subsystem
    extern ShooterSubsystem shooter_subsystem;
    extern TurretSubsystem turret_subsystem;
    extern ChassisSubsystem chassis_subsystem;

    // TODO remove static?
    extern Remote remote;
    extern BufferedSerial referee; // Nucleo board: top left male pins.

    extern CANHandler canHandler1;
    extern CANHandler canHandler2;

    extern DigitalIn jumperPC9;
    extern DigitalIn jumperPB1;

    extern DigitalIn userButton;
    extern TRMutex printer;
    extern Mutex mutex_test;
    extern Thread refThread;
    extern Thread imuThread;
    extern BufferedSerial bc;
    extern BufferedSerial usbSerial;
    // static SPI spi_imu(PB_15, PB_14, PA_9, PB_9); // MOSI, MISO, SCLK, NSS

    // LED Definitions
    extern DigitalOut led;
    extern DigitalOut led2;
    extern DigitalOut led3;
    extern DigitalOut ledbuiltin;

    // CHASSIS Definitions
    extern I2C i2c;
    extern BNO055 imu;

    // CV STUFF
    // TODO make Jetson class for this
    // extern JetsonSubsystem jetson;
    extern BufferedSerial bcJetson; // JETSON PORT
    extern Jetson jetson;
    extern Jetson::Jetson_read_data jetson_received_data;
    extern Jetson::Jetson_read_odom jetson_received_odom;
    extern Jetson::Jetson_send_ref jetson_send_ref;
    extern Jetson::Jetson_send_data jetson_send_data;

// imu
#ifdef USE_IMU
    extern BNO055_ANGULAR_POSITION_typedef imuAngles;
#endif

    // Variables for burst fire
    // unsigned long timeSure;
    // unsigned long prevTimeSure;
    // bool shoot = false;
    // int shootTargetPosition = 36 * 8190;
    // bool shootReady = false;
    // int remoteTimer = 0;

    // // Remote control variables
    // float scalar = 1;
    // float jx = 0; // -1 to 1
    // float jy = 0; // -1 to 1
    // // Pitch, Yaw
    // float jpitch = 0; // -1 to 1
    // float jyaw = 0;   // -1 to 1
    // float myaw = 0;
    // float mpitch = 0;
    // int pitchVelo = 0;
    // // joystick tolerance
    // float tolerance = 0.05;
    // // Keyboard Driving
    // float mult = 0.7;
    // float omega_speed = 0;
    // float max_linear_vel = 0;

    // // DEGREES PER SECOND AT MAX
    // constexpr float JOYSTICK_SENSITIVITY_YAW_DPS = 180.0;
    // constexpr float JOYSTICK_SENSITIVITY_PITCH_DPS = 180.0;

    // // Mouse sensitivity initialized
    // constexpr float MOUSE_SENSITIVITY_YAW_DPS = 10.0;
    // constexpr float MOUSE_SENSITIVITY_PITCH_DPS = 10.0;

    // // GENERAL VARIABLES
    // // drive and shooting mode
    // char drive = 'o'; // default o when using joystick
    // char shot = 'o';  // default o when using joystick

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
}