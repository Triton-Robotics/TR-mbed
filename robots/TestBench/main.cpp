#include "main.h"
#include "subsystems/MecanumChassisSubsystem.h"

I2C i2c(I2C_SDA, I2C_SCL); // defines I2C bus (wires necessary for protocal)
BNO055 imu(i2c, IMU_RESET, MODE_IMU); // create new object that communicates with IMU 

DigitalOut led(L25);
DigitalOut led2(L26);
DigitalOut led3(L27);
DigitalOut ledbuiltin(LED1);

MecanumChassisSubsystem Chassis(4, 1, 3, 2, imu, 0.17);

int main(){

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback(); //initialization

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    
    while (true){
        timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > 1){

            loopTimer = timeStart; //updating values
            led = !led;
            ledbuiltin = !ledbuiltin;

            Chassis.periodic();
            remoteRead();

            //Driving input
            float scalar = 1;
            float jx = remote.leftX() / 660.0 * scalar; // -1 to 1
            float jy = remote.leftY() / 660.0 * scalar; // -1 to 1
            //Pitch, Yaw
            float jpitch = remote.rightY() / 660.0 * scalar; // -1 to 1
            float jyaw = remote.rightX() / 660.0 * scalar; // -1 to 

            //joystick tolerance
            float tolerance = 0.05; 
            jx = (abs(jx) < tolerance) ? 0 : jx;
            jy = (abs(jy) < tolerance) ? 0 : jy;
            jpitch = (abs(jpitch) < tolerance) ? 0 : jpitch;
            jyaw = (abs(jyaw) < tolerance) ? 0 : jyaw;
                        
            //Bounding the four j variables
            jx = max(-1.0F, min(1.0F, jx));
            jy = max(-1.0F, min(1.0F, jy));
            jpitch = max(-1.0F, min(1.0F, jpitch));
            jyaw = max(-1.0F, min(1.0F, jyaw));  

            Chassis.setChassisSpeeds({jx * Chassis.m_OmniKinematicsLimits.max_Vel, 
                                        jy * Chassis.m_OmniKinematicsLimits.max_Vel,
                                        jyaw * Chassis.m_OmniKinematicsLimits.max_vOmega},
                                        MecanumChassisSubsystem::YAW_ORIENTED);
            DJIMotor::s_sendValues(true);
        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }

}
