//
// Created by ankit on 1/31/23.
//

#ifndef TR_EMBEDDED_MAIN_H
#define TR_EMBEDDED_MAIN_H

#include "mbed.h"
// #include "rtos"

#include "algorithms/PID.h"
#include "algorithms/TRMutex.h"
#include "peripherals/imu/BNO055.h"
#include "peripherals/oled/SSD1308.h"
// #include "subsystems/Chassis.h"
#include "subsystems/ChassisSubsystem.h"

#include "motor/PWMMotor.h"
#include "motor/DJIMotor.h"

#include "communications/CANHandler.h"
#include "communications/DJIRemote.h"
#include "communications/SerialCommunication.h"
#include "communications/referee/ref_serial.h"
#include "communications/referee/ref_ui.h"
#include "communications/referee/ref_operations.h"
#include "communications/jetson/Jetson.h"


#include <cstring>
#define ESP_DEBUG 1 // 1 true, 0 false

#define OLED_SDA PB_7
#define OLED_SCL PB_8
#define L25 PB_0
#define L26 PC_1
#define L27 PC_0
#define USE_IMU

static Remote remote(PA_10);
static BufferedSerial referee(PC_10, PC_11, 115200); // Nucleo board: top left male pins.

CANHandler canHandler1(PA_11, PA_12);
CANHandler canHandler2(PB_12, PB_13);
// I2C i2c(OLED_SDA, OLED_SCL);
// SSD1308 oled(&i2c, 0x78);

DigitalIn jumperPC9(PC_9);
DigitalIn jumperPB1(PB_1);

DigitalIn userButton(BUTTON1);
TRMutex printer;
Mutex mutex_test;
Thread refThread;
Thread imuThread;
static BufferedSerial bc(PA_0, PA_1, 115200);
static BufferedSerial usbSerial(USBTX, USBRX, 115200);
// static SPI spi_imu(PB_15, PB_14, PA_9, PB_9); // MOSI, MISO, SCLK, NSS

DigitalOut led(L25);
DigitalOut led2(L26);
DigitalOut led3(L27);
DigitalOut ledbuiltin(LED1);


// CHASSIS DEFINING
I2C i2c(I2C_SDA, I2C_SCL);
BNO055 imu(i2c, IMU_RESET, MODE_IMU);

// CV STUFF
static BufferedSerial bcJetson(PC_12, PD_2, 115200);  //JETSON PORT
Jetson_send_data jetson_send_data;
Jetson_read_data jetson_received_data;

// imu
#ifdef USE_IMU
BNO055_ANGULAR_POSITION_typedef imuAngles;
#endif

// Variables for burst fire
unsigned long timeSure;
unsigned long prevTimeSure;
bool shoot = false;
int shootTargetPosition = 36*8190 ;
bool shootReady = false;
int remoteTimer = 0;


float scalar = 1;
float jx = 0; // -1 to 1
float jy = 0; // -1 to 1
// Pitch, Yaw
float jpitch = 0; // -1 to 1
float jyaw = 0; // -1 to 1
float myaw = 0;
float mpitch = 0;
float yaw_desired_angle = 0;
float yaw_current_angle = 0;
int pitchVelo = 0;
float pitch_current_angle = 0;
float pitch_desired_angle = 0;

// GENERAL VARIABLES
// drive and shooting mode
char drive = 'o'; //default o when using joystick
char shot = 'o'; //default o when using joystick

// joystick tolerance
float tolerance = 0.05; 

// Keyboard Driving
float mult = 0.7;

// ref variables
uint16_t chassis_power_limit;

// timers
unsigned long timeStart;
unsigned long timeStartCV;
unsigned long timeStartRef;
unsigned long timeStartImu;
unsigned long loopTimer = us_ticker_read();
unsigned long controlStart = us_ticker_read();
unsigned long loopTimerCV = loopTimer;
unsigned long loopTimerRef = loopTimer;
unsigned long loopTimerImu = loopTimer;
float elapsedms;

int readResult = 0;
bool cv_enabled = false;
char cv_shoot_status = 0;

void updatePriority(priorityLevels desiredLevel)
{
    printer.updatePriority(desiredLevel);
}

void printfESP(const char *format, ...){
    char temp[100];
    va_list args;
    va_start(args, format);
    vsnprintf(temp, 100, format, args);
    bc.write(temp, 100);
    va_end(args);
}

void print(char statement[], priorityLevels priority = DEFAULT)
{
    printer.print(statement, priority);
}

void print(int integer, priorityLevels priority = DEFAULT)
{
    printer.print(integer, priority);
}

void printff(const char *format, ...)
{
    char temp[50];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(temp, 50, format, args);
    if (len > 0)
       usbSerial.write(temp, len);
    va_end(args);
}

void fprintff(priorityLevels priority, const char *format, ...)
{
    char temp[50];
    va_list args;
    va_start(args, format);
    vsnprintf(temp, 50, format, args);
    printer.print(temp, priority);
    va_end(args);
}

void println(char statement[], priorityLevels priority = DEFAULT)
{
    printer.println(statement, priority);
}

void println(int integer, priorityLevels priority = DEFAULT)
{
    printer.println(integer, priority);
}

inline static void remoteRead()
{
    remote.read();
}

static void remotePrint()
{
    // for (int i = 0; i < 7; i++)
    //     printf("%d\t", dats[i]);
    printf("%d\t%d\t%d\t%d\t%d\t%d\t", remote.leftX(), remote.leftY(), remote.rightX(), remote.rightY(), remote.leftSwitch(), remote.rightSwitch());
    printf("\n");
}

// CANHandler canPorts(PA_11,PA_12,PB_12,PB_13);

// Thread threadingRemote(osPriorityNormal);
// Thread threadingReferee(osPriorityLow);
// Thread threadingPrint(osPriorityBelowNormal);
// Thread threadingLogger(osPriorityLow);

float floatmod(float input, float mod){
    float f = input;
    while(f < 0){
        f += mod;
    }
    while(f >= mod){
        f -= mod;
    }
    return f;
}

constexpr float MAX_METERSPERSSECOND_60W = 1.0;
constexpr float MAX_RADIANSPERSSECOND_60W = 6.3;
float BeybladeModulation(ChassisSpeeds cs){
    float v = sqrt(cs.vX * cs.vX + cs.vY * cs.vY);
    if(v < MAX_METERSPERSSECOND_60W){
        return (MAX_METERSPERSSECOND_60W - v) * (MAX_RADIANSPERSSECOND_60W * 0.8);
    }else{
        return 0;
    }
}

#endif // TR_EMBEDDED_MAIN_H
