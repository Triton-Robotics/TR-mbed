
#include <mbed.h>
#include <main.h>
#include "peripherals/oled/Adafruit_SSD1306.h"

Thread cv;

DJIMotor yaw1(5, CANHandler::CANBUS_1, GIMBLY);
DJIMotor yaw2(7, CANHandler::CANBUS_1, GIMBLY);
DJIMotor pitch(6, CANHandler::CANBUS_2, GIMBLY);
//DJIMotor indexer(7, CANHandler::CANBUS_2, C610);

I2C i2c(I2C_SDA, I2C_SCL);
Chassis chassis(1, 2, 3, 4, &i2c);
Adafruit_SSD1306_I2c oled(i2c, LED1);

DJIMotor RTOPFLYWHEEL(1, CANHandler::CANBUS_2, M3508);
DJIMotor LTOPFLYWHEEL(2, CANHandler::CANBUS_2, M3508);
DJIMotor RBOTTOMFLYWHEEL(5, CANHandler::CANBUS_2, M3508);
DJIMotor LBOTTOMFLYWHEEL(6, CANHandler::CANBUS_2, M3508);\

void setFlyWheelSpeed(int speed) {
    LTOPFLYWHEEL.setSpeed(-speed);
    LBOTTOMFLYWHEEL.setSpeed(speed);
    RTOPFLYWHEEL.setSpeed(speed);
    RBOTTOMFLYWHEEL.setSpeed(-speed);
}

int main(){
    int yawSetpoint = 0;

    pitch.pidPosition.feedForward = 1900;

    pitch.setPositionPID(6.3, 0.2, 0.5);
    pitch.setPositionIntegralCap(10000);
    pitch.useAbsEncoder = 1;
    pitch.justPosError = 1;

    yaw1.setPositionPID(5.5, 0.2, 0.4);
    yaw1.setPositionIntegralCap(10000);
    yaw1.useAbsEncoder = 0;
    yaw1.justPosError = 1;
    yaw1.pidPosition.setOutputCap(100000);
    yaw1.outCap = 32000;

//    indexer.setSpeedPID(1.94, 0.002, 0.166);
//    indexer.setSpeedIntegralCap(500000);

    LTOPFLYWHEEL.setSpeedPID(1, 0, 0);
    RBOTTOMFLYWHEEL.setSpeedPID(1, 0, 0);
    RTOPFLYWHEEL.setSpeedPID(1, 0, 0);
    RBOTTOMFLYWHEEL.setSpeedPID(1, 0, 0);

    DigitalOut led(LED1);

    DJIMotor::setCANHandlers(&canHandler1,&canHandler2, false, false);
    DJIMotor::getFeedback();

    while(true){
        led = !led;

//        double ref_chassis_power = ext_power_heat_data.data.chass
//                yawSetpoint =  - 3 * rX;is_power;
//        printf("Ref power: %i\n", (int) (ref_chassis_power * 100));
//
        remoteRead();
        pitch.setPosition((rY / 2) + 7000);
        chassis.driveTurretRelative({lX * 5.0, lY * 5.0, 0}, 0);

//        if (rX >= 0) {
//            yawSetpoint = 0;
//        } else {
//            yawSetpoint = -2500;
//        }
        yawSetpoint -= rX / 4.5;
        yaw1.setPosition(-yawSetpoint);
        yaw2.setPower(yaw1.powerOut);

        if (lS == Remote::SwitchState::UP) {
//            indexer.setSpeed(-2000);
//            indexer.setPower(0);
            setFlyWheelSpeed(8000);
        } else { //disable serializer
//            indexer.setPower(0);
            setFlyWheelSpeed(0);
        }
        DJIMotor::sendValues();
        DJIMotor::getFeedback();
        ThisThread::sleep_for(1ms);
    }
}