#include "main.h"
#include "subsystems/ChassisSubsystem.h"
#include "util/peripherals/imu/ISM330.h"
#include "util/peripherals/imu/BNO055.h"
#include "util/peripherals/imu/LIS3MDL.h"

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
ISM330::ISM330_ANGULAR_POSITION_typedef imuAnglesISM_Mah;
ISM330::ISM330_ANGULAR_POSITION_typedef liMagYaw;

LIS3MDL::LIS3MDL_VECTOR_TypeDef liMagField;
LIS3MDL::LIS3MDL_VECTOR_TypeDef liMagFieldRaw;
LIS3MDL::LIS3MDL_VECTOR_TypeDef liMagFieldCalibrated;



I2C i2c(I2C_SDA, I2C_SCL);

I2C i2c2(I2C_SDA2, I2C_SCL2);



ISM330 imu2(i2c, 0x6B);

LIS3MDL mag(i2c, 0x3D);


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
    

    mag.begin();



    int timer = us_ticker_read();
    int prev_time = us_ticker_read();
    while (true) {
        timer = us_ticker_read();

        #ifdef USE_IMU
        imu.get_angular_position_quat(&imuAngles);     
        #endif   
        
        //imu.get_mag(&magField);

        //float psi_s_BNO = getPsi(imuAngles.pitch, imuAngles.roll, magField.x, magField.y, magField.z);

        
        //mag.getRawMagVector(liMagFieldRaw);

        //mag.getMagVector(liMagField);

        mag.calibratedMagXY(liMagField);

        float psi_li = getPsi(imuAnglesISM.pitch, imuAnglesISM.roll, liMagField.x, liMagField.y, liMagField.z);

        liMagYaw.yaw = psi_li +11.0; // Adding declination angle to get true North referenced yaw


        imu2.getAGVectors(imuAccelISM, imuGyroISM);
        float mag = sqrtf(imuAccelISM.x*imuAccelISM.x + imuAccelISM.y*imuAccelISM.y + imuAccelISM.z*imuAccelISM.z);
        
        //imu2.getEulerAngles(imuAnglesISM, (timer-prev_time) / 1000, psi_li);

        float dt = (timer-prev_time) / 1000000.0f;
        //imu2.madgwickUpdate(liMagField.x, liMagField.y, liMagField.z, dt);
        
        imu2.madgwickUpdateIMU(dt);
        
        imu2.computeAngles(imuAnglesISM);
        imu2.mahonyUpdate(liMagField.x, liMagField.y, liMagField.z, dt);
        //imu2.mahonyUpdateIMU((timer-prev_time) / 1000);

        imu2.computeAnglesMah(imuAnglesISM_Mah);

        //printf("%0.6f,%0.6f,%0.6f\r\n", liMagField.x, liMagField.y, liMagField.z);

        prev_time = timer;
        ThisThread::sleep_for(1ms);

    }


}