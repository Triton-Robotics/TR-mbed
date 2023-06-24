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

    pitch.setPositionPID(24.3, 0.3, 35.5);
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
    DigitalOut led2(L27);

    DJIMotor::setCANHandlers(&canHandler1,&canHandler2, false, false);
    DJIMotor::getFeedback();

    int refLoop = 0;

    unsigned long loopTimer = us_ticker_read() / 1000;

    while(true){

        unsigned long timeStart = us_ticker_read() / 1000;
        
        if(timeStart - loopTimer > 25) {
            loopTimer = timeStart;

            led = !led;
//        double ref_chassis_power = ext_power_heat_data.data.chass
//                yawSetpoint =  - 3 * rX;is_power;
//
            remoteRead();

            refLoop++;
            if (refLoop >= 5)
            {
                refereeThread(&referee);
                // printf("thread\n");
                refLoop = 0;
                //                 led = ext_power_heat_data.data.chassis_power > 0;
                //                 printf("%d\n",ext_power_heat_data.data.chassis_power);
                led2 = !led2;
            }

            //printf("RS: %i\n", rS);
            //printff("Pitch:%d PWR: %d\n",pitch.getData(ANGLE),pitch.getData(POWEROUT));
            //printff("M2:%d %d\n",chassis.getMotor(1)>>VELOCITY,chassis.getMotor(1)>>POWEROUT);
            //chassis.printMotorAngle();

            //printff("%d\n",Wh);
            
            double bb = 0;

            if(!(rS == Remote::SwitchState::MID)){
                yawSetpoint -= rX / 4.5;
                yaw1.setPosition(-yawSetpoint);
                pitch.setPosition((2*rY / 3) + 6700);
                yaw2.setPower(yaw1.powerOut);
                bb = 2000;
            }else{
                yaw1.setPower(0);
                yaw2.setPower(0);
                pitch.setPower(0);
            }
            if(rS == Remote::SwitchState::DOWN){
                bb = 0;
            }

            double ref_chassis_power = ext_power_heat_data.data.chassis_power;
            int max_power = ext_game_robot_state.data.chassis_power_limit;

            printff("Ref power: %i\n", (int) (ref_chassis_power * 100));

            chassis.driveTurretRelativePower(ref_chassis_power, max_power,{-lX * 5.0, -lY * 5.0, bb},  -yaw1.getData(MULTITURNANGLE) * 360.0 / 8192 + 90);

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