#include "main.h"
#include "subsystems/ChassisSubsystem.h"
#include "util/algorithms/PIDTools.h"

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

// ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2794); // radius is 9 in
DJIMotor wheel(1,CANHandler::CANBUS_2, M3508, "Wheel");

PIDTools tester(&wheel, 1000, TYPE::VELOCITY);


int main(){

    wheel.setSpeedPID(3,0,0);
    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();
    usbSerial.set_blocking(false);

    //Variables for burst fire
    unsigned long timeSure;
    unsigned long prevTimeSure;
    unsigned long shootTimer;

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    int refLoop = 0;
    int printLoop = 0;

    tester.init();
    
    while(true){
        timeStart = us_ticker_read();
        
        if ((timeStart - loopTimer) / 1000 > OUTER_LOOP_DT_MS){
            float elapsedms = (timeStart - loopTimer) / 1000;
            loopTimer = timeStart;
            led = !led;
            
            tester.run();
            
            printLoop++;
            if (printLoop >= PRINT_FREQUENCY){
                printLoop = 0;
            }

            DJIMotor::s_sendValues();



        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}