#include "main.h"
#include "subsystems/ChassisSubsystem.h"
#include "util/peripherals/imu/ISM330.h"
#include "util/peripherals/imu/BNO055.h"

DigitalOut led(L25);
DigitalOut led2(L26);
DigitalOut led3(L27);
DigitalOut ledbuiltin(LED1);

//CONSTANTS
constexpr float LOWERBOUND = -35.0;
constexpr float UPPERBOUND = 40.0;

//DEGREES PER SECOND AT MAX
constexpr float JOYSTICK_SENSITIVITY_YAW_DPS = 180.0; 
constexpr float JOYSTICK_SENSITIVITY_PITCH_DPS = 180.0;

// Mouse sensitivity initialized
constexpr float MOUSE_SENSITIVITY_YAW_DPS = 10.0;
constexpr float MOUSE_SENSITIVITY_PITCH_DPS = 10.0;

constexpr int OUTER_LOOP_DT_MS = 15;

constexpr int PRINT_FREQUENCY = 8; //the higher the number, the less often

constexpr float CHASSIS_FF_KICK = 0.065;

constexpr float pitch_zero_offset_ticks = 1500;

constexpr int NUM_BALLS_SHOT = 3;
constexpr int FLYWHEEL_VELO = 5500;

#define READ_DEBUG 0
#define MAGICBYTE 0xEE
#define USE_IMU



#define I2C_SDA2 PC_12
#define I2C_SCL2 PB_10



//CHASSIS DEFINING

//SPI spiIMU(PB_15, PB_14, PA_9, PB_9); // MOSI, MISO, SCK, NSS/CS
//ISM330 imu2(spiIMU, PB_9);

// ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.22617); // radius is 9 in
// DJIMotor yaw(4, CANHandler::CANBUS_1, GIMBLY,"Yeah");
// DJIMotor pitch(7, CANHandler::CANBUS_2, GIMBLY,"Peach"); // right

// DJIMotor indexer(7, CANHandler::CANBUS_2, C610,"Indexer");
// DJIMotor RFLYWHEEL(1, CANHandler::CANBUS_2, M3508,"RightFly");
// DJIMotor LFLYWHEEL(2, CANHandler::CANBUS_2, M3508,"LeftFly");

// //CV STUFF
// static BufferedSerial bcJetson(PC_12, PD_2, 115200);  //JETSON PORT



#ifdef USE_IMU
BNO055_ANGULAR_POSITION_typedef imuAngles;
#endif

BNO055_VECTOR_TypeDef magField;
BNO055_ANGULAR_POSITION_typedef magYaw;

ISM330::ISM330_VECTOR_TypeDef imuAccelISM;
ISM330::ISM330_VECTOR_TypeDef imuGyroISM;
ISM330::ISM330_ANGULAR_POSITION_typedef imuAnglesISM;


I2C i2c(I2C_SDA, I2C_SCL);

I2C i2c2(I2C_SDA2, I2C_SCL2);



ISM330 imu2(i2c, 0x6B);


float getPsi(float pitch, float roll, float mX, float mY, float mZ) {
    float pitchRad = pitch * M_PI / 180.0f;
    float rollRad = roll * M_PI / 180.0f;


    float Xh = mX * cos(pitchRad) - mZ * sin(pitchRad);
    float Yh = mX * sin(rollRad) * sin(pitchRad) + mY * cos(rollRad) + mZ * sin(rollRad) * cos(pitchRad);
return atan2(Yh, Xh)*180/M_PI;
}
  

BNO055 imu(i2c, IMU_RESET, MODE_NDOF);

int main(){

    imu2.begin();
    
    // // Basic IMU Readings Check
    // while (true) {
    // auto [ax, ay, az] = imu.readAccel();
    // auto [gx, gy, gz] = imu.readGyro();
    
    // printf("Accel %.2f, %.2f, %.2f | Gyro %.2f, %.2f, %.2f\n", ax, ay, az, gx, gy, gz);
    // }



    int timer = us_ticker_read();
    int prev_time = us_ticker_read();
    while (true) {
        timer = us_ticker_read();

        #ifdef USE_IMU
        imu.get_angular_position_quat(&imuAngles);     
        #endif   
        
        imu.get_mag(&magField);

        magYaw.yaw= atan2(magField.y,magField.x)/2/M_PI*360;

        float psi_s = getPsi(imuAngles.pitch, imuAngles.roll, magField.x, magField.y, magField.z);

        imu2.getAGVectors(imuAccelISM, imuGyroISM);
        imu2.getEulerAngles(imuAnglesISM, (timer-prev_time) / 1000, psi_s);

        printf("BNO Yaw: %.2f| ISM Yaw: %.2f | Mag Yaw: %.2f | Psi_s: %.2f\n", imuAngles.yaw, imuAnglesISM.yaw, magYaw.yaw, psi_s);

        //printf("Accel %.2f, %.2f, %.2f | Gyro %.2f, %.2f, %.2f | KF Pitch: %.2f | KF Yaw: %.2f\n", ax, ay, az, gx, gy, gz, kf_pitch, kf_yaw);
        
        //printf("ISM Accel: %.2f, %.2f, %.2f | ISM Gyro: %.2f, %.2f, %.2f | ISM Pitch: %.2f | ISM Roll: %.2f\n", imuAccelISM.x, imuAccelISM.y, imuAccelISM.z, imuGyroISM.x, imuGyroISM.y, imuGyroISM.z, imuAnglesISM.pitch, imuAnglesISM.roll);

        //printf("BNO Yaw: %.2f | Pitch: %.2f | Roll: %.2f| ISM Yaw: %.2f | Pitch: %.2f | Roll: %.2f | w_z: %.2f\n", imuAngles.yaw, imuAngles.pitch, imuAngles.roll, imuAnglesISM.yaw, imuAnglesISM.pitch, imuAnglesISM.roll, imuGyroISM.z);
        
        //printf("BNO Yaw delta: %.2f| ISM True yaw: %.2f| ISM Yaw delta: %.2f | Yaw Delta: %.2f\n", BNO_OffsetYaw, imuAnglesISM.yaw, ISM_OffsetYaw, yawDiff);

        prev_time = timer;
        ThisThread::sleep_for(1ms);

    }


}