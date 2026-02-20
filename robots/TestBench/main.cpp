#include "main.h"
#include "subsystems/ChassisSubsystem.h"
#include "mbed.h"
#include "util/peripherals/INA226_new/ina226.hpp"

I2C i2c(PB_7, PB_8);
DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);
DigitalOut ledbuiltin(LED1);

AnalogIn ain(PA_7);
DigitalOut dout(LED1);

// Serial pc(USBTX, USBRX);
DigitalOut myled(LED1);

unsigned const int I2C_FREQ = 100000;
const int ina_addr = 0x80;
const float current_limit = 1.0;
ina226 ina(i2c, ina_addr, I2C_FREQ); 


// ACS712-30A Current Sensor Configuration
// Constants for ACS712-30A (66mV/A sensitivity, VCC/2 at 0A)
const float ACS712_SENSITIVITY = 0.066f;    // 66 mV per Ampere for 30A model
const float ACS712_VOLTAGE_AT_ZERO_A = 3.3f / 2.0f;  // Assuming 3.3V VCC
const float VCC = 3.3f;                     // Your board's analog reference voltage

const float Kt = 0.18f;
const float GEAR_RATIO = 36.0f;


// ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.22617); // radius is 9 in - DISABLED FOR RAW TESTING
DJIMotor feeder(5, CANHandler::CANBUS_2, M2006);
DJIMotor RFLYWHEEL(6, CANHandler::CANBUS_1, M3508,"RightFly");
DJIMotor LFLYWHEEL(7, CANHandler::CANBUS_1, M3508,"LeftFly");

// RAW MOTOR CONTROL - NO PID, NO CHASSIS SUBSYSTEMAb
DJIMotor motor1(1, CANHandler::CANBUS_1, M3508, "Motor1");
DJIMotor motor2(2, CANHandler::CANBUS_1, M3508, "Motor2");
DJIMotor motor3(3, CANHandler::CANBUS_1, M3508, "Motor3");
DJIMotor motor4(4, CANHandler::CANBUS_1, M3508, "Motor4");

#define IMPULSE_DT 100
#define IMPULSE_STRENGTH 16383

bool send_power = false;

float TorqueFromCurrent(int torque_raw) {

    float current_A = (torque_raw / 16384.0f) * 20.0f;
    float motor_torque = Kt * current_A;
    float output_torque = motor_torque * GEAR_RATIO;

    return output_torque;
}

float readCurrentACS712() {
    // Read raw analog value (0.0 to 1.0)
    float analog_value = ain.read();   // output voltage
    
    // Convert to voltage (0.0 to VCC)
    float sensor_voltage = analog_value * VCC;
    
    // For bidirectional sensing (positive and negative current)
    float current = (ACS712_VOLTAGE_AT_ZERO_A - sensor_voltage) / ACS712_SENSITIVITY;
    
    return current;
}

float calculateTorqueFromACS712(float current_A) {
    float motor_torque = Kt * current_A;          // Motor shaft torque
    float output_torque = motor_torque * GEAR_RATIO; // Output torque
    
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

    //INA stuff
    printf("INA226 TEST Program. (BUILD:[" __DATE__ "/" __TIME__ "])\n");

    // Device configuration. See header file for details.
    printf("INA226 Config return: %d\n", ina.setConfig());  // default configuration
    printf("INA226 Calibration return: %d\n", ina.setCalibration());  // default calibration
    ina.enableShuntOverVoltageAlert();
    ina.setOverCurrentLimit(current_limit);
    while(true){
        timeStart = us_ticker_read();       

        if ((timeStart - loopTimer) / 1000 > 1){
            loopTimer = timeStart;

            static int debugCounter = 0;
            if(debugCounter++ % 100 == 0) {  // Only print every 50 loops
                led = !led;
                ledbuiltin = !ledbuiltin; 
                
                // //INA stuff
                // printf("Device %xh: ManID %d, DieID %d, Cal %d, ShuntV %+2.6fV, %+2.6fV, %+2.6fA, %+2.6fW\n",
                // ina_addr,
                // ina.readManufacturerID(),
                // ina.readDieID(),
                // ina.readCalibration(),
                // ina.readShuntVoltage(),
                // ina.readBusVoltage(),
                // ina.readCurrent(),
                // ina.readPower());
                // if (ina.isAlert()) {
                // printf("Overcurrent detected\n");
                // }  

                printf("\n--- Starting I2C Scanner ---\n");
                int ack_count = 0;
                for (int i = 1; i < 128; i++) {
                    // Mbed I2C write returns 0 on success (ACK received)
                    if (i2c.write(i << 1, NULL, 0) == 0) { 
                        printf("SUCCESS: Found I2C device at 7-bit address 0x%02X (8-bit: 0x%02X)\n", i, i << 1);
                        ack_count++;
                    }
                }
                if (ack_count == 0) {
                    printf("ERROR: No I2C devices found. Check wiring and power!\n");
                }
                printf("--- Scanner Complete ---\n\n");
            }

        
                 
            
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

            //INDEXER CODE
            if (remote.leftSwitch() == Remote::SwitchState::UP) {
                feeder.setSpeed(5 * 16 * M2006_GEAR_RATIO);
            } 
            else {
                feeder.setPower(0);
            }
            
            DJIMotor::s_sendValues();  // Enable debug output
        }

        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}