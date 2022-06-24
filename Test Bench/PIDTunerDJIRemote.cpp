#include "mbed.h"
#include "../src/main.hpp"

// PWMMotor leftFlywheelTop(PA_5);
// PWMMotor rightFlywheelTop(PA_6);
// PWMMotor leftFlywheelBot(PB_6);
// PWMMotor rightFlywheelBot(PA_7);

// Either MULTITURNANGLE or VELOCITY 

#define motorID 2
#define motorType M3508
#define MOTORMODE VELOCITY
#define outputCap 0
#define integralCap 0
#define feedForwardVal 0 //useful for overcoming gravity

int posMotorCommands[] = {1000,1500,2000};
int speedMotorCommands[] = {0,45*19,90*19};

CANMotor testMotor(motorID, NewCANHandler::CANBUS_1, motorType);

float PIDvals[] = {0, 0, 0};

void updateTestMotorCommand() {
    if (myremote.getSwitchData(LSWITCH) == 1) { 
            if (MOTORMODE == MULTITURNANGLE)
                testMotor.setPosition(posMotorCommands[0]);
            else 
                testMotor.setSpeed(speedMotorCommands[0]);
    }
    if (myremote.getSwitchData(LSWITCH) == 2) {
            if (MOTORMODE == MULTITURNANGLE) 
                testMotor.setPosition(posMotorCommands[1]);
            else
                testMotor.setSpeed(speedMotorCommands[1]);
    }
    if (myremote.getSwitchData(LSWITCH) == 3) {
            if (MOTORMODE == MULTITURNANGLE) 
                testMotor.setPosition(posMotorCommands[2]);
            else
                testMotor.setSpeed(speedMotorCommands[2]);
    }
}


void printInfo() {
        if (myremote.getSwitchData(RSWITCH) == 1) {
            if (MOTORMODE == MULTITURNANGLE) 
                printf("Des: %d \t Act: %d", testMotor.getValue(), testMotor.getData(MULTITURNANGLE));
            if (MOTORMODE == VELOCITY)
                printf("Des: %d \t Act: %d", testMotor.getValue(), testMotor.getData(VELOCITY));
        }
        else if (myremote.getSwitchData(RSWITCH) == 2) {
            for (int i = 0; i< 3; i++) {
                printf("%d", (int)(PIDvals[i]*1000));
                //printFloat(PIDvals[i], 3);
                printf("\t");
            }
            if (MOTORMODE == MULTITURNANGLE) 
                printf("Des: %d \t Act: %d \t", testMotor.getValue(), testMotor.getData(MULTITURNANGLE));
            if (MOTORMODE == VELOCITY)
                printf("Des: %d \t Act: %d", testMotor.getValue(), testMotor.getData(VELOCITY));
            
            //printf("Pwr: %d", testMotor.getPowerOut());
        }
        else {
        }
        printf("\n");
}


int main(void) {
    CANMotor::setCANHandlers(&canHandler1,&canHandler2);
    threadingRemote.start(remoteThread);
    testMotor.pidPosition.feedForward = feedForwardVal;

    if (outputCap != 0)
        testMotor.setPositionOutputCap(outputCap);
    if (integralCap != 0)
        testMotor.setPositionIntegralCap(integralCap);

    int count = 0;


    while (1) {
        if (MOTORMODE == MULTITURNANGLE)
            testMotor.setPositionPID(PIDvals[0], PIDvals[1], PIDvals[2]);
        if (MOTORMODE == VELOCITY)
            testMotor.setSpeedPID(PIDvals[0], PIDvals[1], PIDvals[2]);

        updateTestMotorCommand();
        printInfo();

        PIDvals[0] += myremote.getStickData(LEFTJOYX, 0, .01);
        PIDvals[1] -= myremote.getStickData(WHEEL, 0, 0.001);
        PIDvals[2] += myremote.getStickData(RIGHTJOYX, 0, .01);

        for (int i = 0; i < 3; i++) 
            if (PIDvals[i] < 0)
                PIDvals[i] = 0;

        ThisThread::sleep_for(1ms); //dunno why but it is VERY IMPORTANT!!
    }
}
