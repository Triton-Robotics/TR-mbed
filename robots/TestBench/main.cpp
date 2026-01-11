#include "main.h"
#include "subsystems/ChassisSubsystem.h"

I2C i2c(I2C_SDA, I2C_SCL);
BNO055 imu(i2c, IMU_RESET, MODE_IMU);
DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);
DigitalOut ledbuiltin(LED1);
// ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.22617); // radius is 9 in - DISABLED FOR RAW TESTING
DJIMotor feeder(5, CANHandler::CANBUS_2, M2006);
DJIMotor RFLYWHEEL(6, CANHandler::CANBUS_1, M3508,"RightFly");
DJIMotor LFLYWHEEL(7, CANHandler::CANBUS_1, M3508,"LeftFly");

// RAW MOTOR CONTROL - NO PID, NO CHASSIS SUBSYSTEM
DJIMotor motor1(1, CANHandler::CANBUS_1, M3508, "Motor1");
DJIMotor motor2(2, CANHandler::CANBUS_1, M3508, "Motor2");
DJIMotor motor3(3, CANHandler::CANBUS_1, M3508, "Motor3");
DJIMotor motor4(4, CANHandler::CANBUS_1, M3508, "Motor4");

#define IMPULSE_DT 100
#define IMPULSE_STRENGTH 16383

bool send_power = false;

float TorqueFromCurrent(int torque_raw) {
const float Kt = 0.18f;
const float GEAR_RATIO = 36.0f;

float current_A = (torque_raw / 16384.0f) * 20.0f;
float motor_torque = Kt * current_A;
float output_torque = motor_torque * GEAR_RATIO;

return output_torque;
}

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
    feeder.setSpeedPID(1, 0, 0);
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
            // Chassis.setChassisSpeeds({jx * Chassis.m_OmniKinematicsLimits.max_Vel,
            //                               jy * Chassis.m_OmniKinematicsLimits.max_Vel,
            //                               0 * Chassis.m_OmniKinematicsLimits.max_vOmega}, 100,
            //                               ChassisSubsystem::YAW_ORIENTED);
            // motor1.setPower(remote.leftX() * 8000 / 660);
            // motor2.setPower(remote.leftY() * 8000 / 660);
            // motor3.setPower(remote.rightX() * 8000 / 660);
            // motor4.setPower(remote.rightY() * 8000 / 660);
            // static int statusCounter = 0;
            // if(statusCounter++ % 50 == 0) {
            //     if (send_power == true) {
            //         send_power = false;
            //     }
            //     else if (send_power == false) {
            //         send_power = true;
            //     }
            // }

            //INDEXER CODE
            if (remote.leftSwitch() == Remote::SwitchState::UP) {
                // if(robot_status.shooter_barrel_heat_limit < 10 || power_heat_data.shooter_17mm_1_barrel_heat < robot_status.shooter_barrel_heat_limit - 30) {
                    feeder.setSpeed(5 * 16 * M2006_GEAR_RATIO);
                // }
                // else {
                //     feeder.setSpeed(0);
                // }
            } 
            else {
                feeder.setPower(0);
            }

            // if (remote.rightSwitch() == Remote::SwitchState::UP) {
            //     motor1.setSpeed(130*19);
            //     motor2.setSpeed(130*19);
            //     motor3.setSpeed(130*19);
            //     motor4.setSpeed(130*19);
            //     feeder.setPower(2500);
            //     RFLYWHEEL.setSpeed(3000);
            //     LFLYWHEEL.setSpeed(3000);
            // } else {
            //     feeder.setPower(0);
            // }


            // if (send_power) {
            //     motor1.setPower(1000);
            //     motor2.setPower(1000);
            //     motor3.setPower(1000);
            //     motor4.setPower(1000);
            //     feeder.setSpeed(130*19);
            //     RFLYWHEEL.setPower(300);
            //     LFLYWHEEL.setPower(300);
            // } else {
            //     motor1.setPower(-1000);
            //     motor2.setPower(-1000);
            //     motor3.setPower(-1000);
            //     motor4.setPower(-1000);
            //     feeder.setPower(0);
            //     RFLYWHEEL.setPower(-300);
            //     LFLYWHEEL.setPower(-300);
            // }
            
                      
  
            // testMot.setSpeed(remote.leftX() * 3000 / 660);  // Use speed control instead of raw power
            
            // Debug PID behavior - Status output every 100ms
            // static int statusCounte = 0;
            // if(statusCounte++ % 100 == 0) {
            //     printff("V=%d P=%d | CHILL?:%s\n",
            //     feeder.getData(VELOCITY), feeder.getData(POWEROUT), 
            //     feeder.isConnected() ? "Y" : "N");
            // }

            // feeder.setSpeed((remote.rightX()/660) * 3000);
            // RFLYWHEEL.setSpeed((remote.rightY()/660) * 3000);
            // LFLYWHEEL.setSpeed((remote.rightY()/660) * 3000);
            
            // // Debug output - reduce frequency to save memory
            // static int debugCounter = 0;
            // if(debugCounter++ % 50 == 0) {  // Only print every 50 loops
            //     printff("%d %d %d %d\n", 
            //     remote.rightX(),
            //     remote.rightY(),
            //     remote.leftX(),
            //     remote.leftY());
            // }
            
            // printff("%d,%d\n", testMot>>VELOCITY, testMot>>POWEROUT);            
            
        //      static int statusCounter = 0;
        //      if(statusCounter++ % 50 == 0) {

            DJIMotor::s_sendValues();  // Enable debug output
        //             printff("%d %d %d %d\n",
        // motor1.getData(VELOCITY)/24,
        // motor2.getData(VELOCITY)/24,
        // motor3.getData(VELOCITY)/24,
        // motor4.getData(VELOCITY)/24);
        //             }
        

        

        // Debug output - reduce frequency to save memory
        static int debugCounter = 0;
        if(debugCounter++ % 50 == 0) {  // Only print every 50 loops
            int torque_raw = feeder.getData(TORQUE);
            printff("Torque-raw: %d\n", torque_raw);
        
            float value = TorqueFromCurrent(torque_raw);
            printff("Value: %.3f\n",value);
        }
        // printf("Motor Data: ");
        // feeder.printAllMotorData();


        
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
                    // Check if all 4 motors are connected
            // DJIMotor* motors[4] = {&motor1, &motor2, &motor3, &motor4};
            // bool allConnected = DJIMotor::s_theseConnected(motors, 4, true);
            // if (!allConnected) {
            //     printff("WARNING: Some motors disconnected!\n");
            // }
    }
}
}