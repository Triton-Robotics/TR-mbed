#include "../src/main.hpp"
#include <cstdlib>
//#include "../util/communications/DJIRemote.cpp"

// #include "o-led/Adafruit_SSD1306.h"
// #include "o-led/Adafruit_SSD1306.cpp"
// #include "o-led/Adafruit_GFX.h"
// #include "o-led/Adafruit_GFX.cpp"
// #include "o-led/Adafruit_GFX_Config.h"

// SPIPreInit gSpi(p5,NC,p7);
// Adafruit_SSD1306_Spi gOled1(gSpi,p26,p25,p24);

//I2C gI2C(PB_9,PB_8);
//Adafruit_SSD1306_I2c gOled2(gI2C,PA_13);

// sentry only from 1230 - 1830

CANMotor m3508_1(1,NewCANHandler::CANBUS_1,M3508);
CANMotor m3508_2(2,NewCANHandler::CANBUS_1,M3508);
CANMotor m3508_3(3,NewCANHandler::CANBUS_1,M3508);
CANMotor m3508_4(4,NewCANHandler::CANBUS_1,M3508);
CANMotor m3508_5(5,NewCANHandler::CANBUS_1,M3508);
CANMotor m2006_7(7,NewCANHandler::CANBUS_1,M2006);
CANMotor gimbly8(4,NewCANHandler::CANBUS_1,GIMBLY);
CANMotor gimbly9(5,NewCANHandler::CANBUS_1,GIMBLY);
CANMotor gimbly10(6,NewCANHandler::CANBUS_1,GIMBLY);
CANMotor gimbly11(7,NewCANHandler::CANBUS_1,GIMBLY);


DigitalOut led(LED1);
DigitalIn button(BUTTON1);

//Remote remote1(PA_3);

CANMotor getMotor(int i){
    switch (i) {
        
        case 1:
            return m3508_1;
            break;
        case 2:
            return m3508_2;
            break;
        case 3:
            return m3508_3;
            break;
        case 4:
            return m3508_4;
            break;
        case 5:
            return m3508_5;
            break;
        case 6:
            return m3508_1;
            break;
        case 7:
            return m3508_1;
            break;
        case 8:
            return gimbly8;
            break;
        case 9:
            return gimbly9;
            break;
        case 10:
            return gimbly10;
            break;
        case 11:
            return gimbly11;
            break;
        default:
            return m3508_1;
            break;
    }
}

int main()
{
    CANMotor::setCANHandlers(&canHandler1,&canHandler2, false, false);

    int numMotor = 0;
    int stateLS = 0;

    unsigned long loopTimer = us_ticker_read() / 1000;

    m3508_1.setSpeedPID(1.79, 0.0, 10.57);
    m3508_2.setSpeedPID(1.79, 0.0, 10.57);
    m3508_3.setSpeedPID(1.79, 0.0, 10.57);
    m3508_4.setSpeedPID(1.79, 0.0, 10.57);
    m3508_5.setSpeedPID(1.79, 0.0, 10.57);
    gimbly8.setPositionPID(0.5,0,0);
    gimbly9.setPositionPID(0.5,0,0);
    gimbly10.setPositionPID(0.5,0,0);
    gimbly11.setPositionPID(0.5,0,0);

    int countLoops = 0;
    int refLoop=0;

    while (true) {
        unsigned long timeStart = us_ticker_read() / 1000;
        if(timeStart - loopTimer > 25){
            loopTimer = timeStart;

            //printf("*Potato*  %d\n", countLoops);
            led = !led;
            // if(countLoops % 50 == 1)
            //     led = !led;

            remoteRead();
            //remote1.read();
            refLoop++;
            if(refLoop > 25){
                refereeThread();
                refLoop = 0;
            }

            if(lS == 4){
                // char str[80];
                // sprintf(str, "%d:%d:%d:%d\n", m3508_2.getData(ANGLE),m3508_2.getData(VELOCITY),m3508_2.getData(TORQUE),m3508_2.getData(TEMPERATURE));
                // printf(str);
            }else if(lS == 1){
                // m3508_1.setSpeed(800 * (rS - 2));
                // m3508_2.setSpeed(1000 * (rS - 2));
                // m3508_3.setSpeed(800 * (rS - 2));
                // m3508_4.setSpeed(800 * (rS - 2));
                // m2006_7.setSpeed(800 * (rS - 2));
                gimbly8.setPosition(2000 * (rS - 2));
                gimbly9.setPosition(2000 * (rS - 2));
                gimbly10.setPosition(2000 * (rS - 2));
                //gimbly11.setPosition(2000 * (rS - 2));
                
            }else if(lS == 3){
                // printf("print:%s<\n",Robot_Commute);
                m3508_1.setPower(0);
                m3508_2.setPower(0);
                m3508_3.setPower(0);
                m3508_4.setPower(0);
                m2006_7.setPower(0);
                gimbly8.setPower(0);
                gimbly9.setPower(0);
                gimbly10.setPower(0);
                //gimbly11.setPower(0);
            }else{
                m3508_1.setPower(0);
                m3508_2.setPower(0);
                m3508_3.setPower(0);
                m3508_4.setPower(0);
                m2006_7.setPower(0);
                gimbly8.setPower(0);
                gimbly9.setPower(0);
                gimbly10.setPower(0);
                //gimbly11.setPower(0);
            }
            //gimbly9.setPower(1000);
            // if(remote1.getSwitch(Remote::Switch::LEFT_SWITCH)  == Remote::SwitchState::MID){
            //     printf("YEETUS\n");
            // }
            // if(rS == 2){
            //     oled.writeChar('a');
            // }
            // else
            //     oled.writeString(0, 0, "Hello World ! 1234567890 abcdefghijklmnopqrstuvwxyz");

            //gOled2.printf("YEETUS DELETuS\n");
            //gOled2.display();

            CANMotor::sendValues();
        }
        unsigned long timeEnd = us_ticker_read() / 1000;
        CANMotor::getFeedback();
        ThisThread::sleep_for(1ms);
        countLoops ++;
        
    }
}

