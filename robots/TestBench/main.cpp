#include "main.h"
#include "subsystems/ChassisSubsystem.h"

I2C i2c(I2C_SDA, I2C_SCL);
BNO055 imu(i2c, IMU_RESET, MODE_IMU);
DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);
DigitalOut ledbuiltin(LED1);
// ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.22617); // radius is 9 in - DISABLED FOR RAW TESTING
DJIMotor feeder(5, CANHandler::CANBUS_1, M2006);
DJIMotor RFLYWHEEL(6, CANHandler::CANBUS_1, M3508,"RightFly");
DJIMotor LFLYWHEEL(7, CANHandler::CANBUS_1, M3508,"LeftFly");

// RAW MOTOR CONTROL - NO PID, NO CHASSIS SUBSYSTEM
DJIMotor motor1(1, CANHandler::CANBUS_1, M3508, "Motor1");
DJIMotor motor2(2, CANHandler::CANBUS_1, M3508, "Motor2");
DJIMotor motor3(3, CANHandler::CANBUS_1, M3508, "Motor3");
DJIMotor motor4(4, CANHandler::CANBUS_1, M3508, "Motor4");

PwmIn pwm_input(PA_7); 

#define IMPULSE_DT 100
#define IMPULSE_STRENGTH 16383

bool send_power = false;

int main(){    

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback(); 


    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    unsigned long loopTimer2 = us_ticker_read();
    int powerCnt = 0;
    int refLoop = 0;    
    
    // Set up feeder PID for C610/M2006
    feeder.setSpeedPID(4, 0, 1);
    feeder.setSpeedIntegralCap(8000);
    feeder.setSpeedOutputCap(16000);

    LFLYWHEEL.setSpeedPID(7.1849, 0.000042634, 0);
    RFLYWHEEL.setSpeedPID(7.1849, 0.000042634, 0);
    
    while(true){
        timeStart = us_ticker_read();       

        if ((timeStart - loopTimer) / 1000 > 1){
            
            float elapsedms = (timeStart - loopTimer) / 1000;
            loopTimer = timeStart;
            led = !led;
            ledbuiltin = !ledbuiltin;            
            
            refLoop++;
            if (refLoop >= 5){                
                refereeThread(&referee);
                refLoop = 0;
                led2 = !led2;            
            
            }    
            
            // Chassis.periodic();
            remoteRead(); 

            // Get yaw position from encoder (0-360 degrees)
            float yaw_position = pwm_input.dutycycle() * 360.0;

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

            // Debug PID behavior - Status output every 100ms
            static int statusCounte = 0;
            if(statusCounte++ % 100 == 0) {
                printff("encoder angle: %.2f | pwm period: %.2f ms | pulsewidth: %.2f ms | dutycycle: %.2f\n", 
                    yaw_position, 
                    pwm_input.period(), 
                    pwm_input.pulsewidth(), 
                    pwm_input.dutycycle() * 100.0);

            }
            DJIMotor::s_sendValues();  // Enable debug output
        }


        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}