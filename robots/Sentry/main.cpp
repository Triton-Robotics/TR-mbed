#include <mbed.h>
#include <main.h>
#include "peripherals/oled/Adafruit_SSD1306.h"

DJIMotor yaw1(5, CANHandler::CANBUS_1, GIMBLY);
DJIMotor yaw2(7, CANHandler::CANBUS_1, GIMBLY);
DJIMotor pitch(6, CANHandler::CANBUS_2, GIMBLY);
DJIMotor indexerL(7, CANHandler::CANBUS_2, C610);
DJIMotor indexerR(8, CANHandler::CANBUS_2, C610);


I2C i2c(I2C_SDA, I2C_SCL);
Chassis chassis(1, 2, 3, 4, &i2c);
Adafruit_SSD1306_I2c oled(i2c, LED1);

DJIMotor RTOPFLYWHEEL(1, CANHandler::CANBUS_2, M3508_FLYWHEEL);
DJIMotor LTOPFLYWHEEL(2, CANHandler::CANBUS_2, M3508_FLYWHEEL);
DJIMotor RBOTTOMFLYWHEEL(3, CANHandler::CANBUS_2, M3508_FLYWHEEL);
DJIMotor LBOTTOMFLYWHEEL(4, CANHandler::CANBUS_2, M3508_FLYWHEEL);

void setFlyWheelSpeed(int speed) {
    LTOPFLYWHEEL.setSpeed(-speed);
    LBOTTOMFLYWHEEL.setSpeed(speed);
    RTOPFLYWHEEL.setSpeed(speed);
    RBOTTOMFLYWHEEL.setSpeed(-speed);
}

int main(){
    int yawSetpoint = 0;

    pitch.pidPosition.feedForward = 1900;

    pitch.setPositionPID(21.3, 0.3, 15.5);
    pitch.setPositionIntegralCap(10000);
    pitch.setPositionOutputCap(20000);
    pitch.useAbsEncoder = 1;
    pitch.justPosError = 1;

    yaw1.setPositionPID(10.5, 0.2, 4.4);
    yaw1.setPositionIntegralCap(10000);
    yaw1.useAbsEncoder = 0;
    yaw1.justPosError = 1;
    yaw1.pidPosition.setOutputCap(100000);
    yaw1.outCap = 32000;
    

    indexerL.setSpeedPID(1.94, 0.002, 0.166);
    indexerL.setSpeedIntegralCap(500000);

    // LTOPFLYWHEEL.setSpeedPID(1, 0, 0);
    // RBOTTOMFLYWHEEL.setSpeedPID(1, 0, 0);
    // RTOPFLYWHEEL.setSpeedPID(1, 0, 0);
    // RBOTTOMFLYWHEEL.setSpeedPID(1, 0, 0);

    DigitalOut led(L26);

    DJIMotor::setCANHandlers(&canHandler1,&canHandler2, false, false);
    DJIMotor::getFeedback();

    unsigned long loopTimer = us_ticker_read() / 1000;

    while(true){

        unsigned long timeStart = us_ticker_read() / 1000;
        
        if(timeStart - loopTimer > 25) {
            loopTimer = timeStart;

            led = !led;
//        double ref_chassis_power = ext_power_heat_data.data.chass
//                yawSetpoint =  - 3 * rX;is_power;
//        printf("Ref power: %i\n", (int) (ref_chassis_power * 100));
//
            remoteRead();
            //printf("RS: %i\n", rS);
            //printff("Pitch:%d PWR: %d\n",pitch.getData(ANGLE),pitch.getData(POWEROUT));
            //printff("M2:%d %d\n",chassis.getMotor(1)>>VELOCITY,chassis.getMotor(1)>>POWEROUT);
            //chassis.printMotorAngle();
            
            chassis.driveTurretRelative({lX * 5.0, lY * 5.0, 0}, 0);

            if(rS == Remote::SwitchState::UP){
                yawSetpoint -= rX / 4.5;
                yaw1.setPosition(-yawSetpoint);
                pitch.setPosition((2*rY / 3) + 6700);
                yaw2.setPower(yaw1.powerOut);
            }else{
                yaw1.setPower(0);
                yaw2.setPower(0);
                pitch.setPower(0);
            }

            if (lS == Remote::SwitchState::UP) {
//              indexerL.setSpeed(-2000);
                setFlyWheelSpeed(8000);
            } else { //disable serializer
                indexerL.setPower(0);
                indexerR.setPower(0);
                setFlyWheelSpeed(0);
            }
            DJIMotor::sendValues();
        }
        DJIMotor::getFeedback();
        ThisThread::sleep_for(1ms);
    }
}