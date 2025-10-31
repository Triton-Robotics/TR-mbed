//
// Created by ankit on 1/31/23.
//

#ifndef TR_EMBEDDED_MAIN_H
#define TR_EMBEDDED_MAIN_H

#include "mbed.h"

#include "algorithms/PID.h"
#include "algorithms/TRMutex.h"
#include "peripherals/imu/BNO055.h"

#include "peripherals/imu/ISM330.h"

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
// I2C i2c(OLED_SDA, OLED_SCL);

// SSD1308 oled(&i2c, 0x78);

DigitalIn jumperPC9(PC_9);
DigitalIn jumperPB1(PB_1);

// SPI MISO, MOSI, CS, and SCL define

#define MOSI PA_9
#define MISO PB_9
#define CS PB14
#define CLK PB15

DigitalIn userButton(BUTTON1);
TRMutex printer;
static BufferedSerial bc(PA_0, PA_1, 115200);
static BufferedSerial usbSerial(USBTX, USBRX, 115200);

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
#define L25 PB_0
#define L26 PC_1
#define L27 PC_0

static Remote remote(PA_10);
static BufferedSerial referee(PC_10, PC_11, 115200); // Nucleo board: top left male pins.

CANHandler canHandler1(PA_11, PA_12);
CANHandler canHandler2(PB_12, PB_13);

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
