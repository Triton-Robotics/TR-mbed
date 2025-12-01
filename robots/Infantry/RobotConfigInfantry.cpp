#include "RobotConfig.h"

namespace TR
{
    // TODO add a configure method to every subsystem
    ShooterSubsystem shooter_subsystem();
    TurretSubsystem turret_subsystem();
    ChassisSubsystem chassis_subsystem();

    // TODO remove static?
    Remote remote(PA_10);
    BufferedSerial referee(PC_10, PC_11, 115200); // Nucleo board: top left male pins.

    CANHandler canHandler1(PA_11, PA_12);
    CANHandler canHandler2(PB_12, PB_13);

    DigitalIn jumperPC9(PC_9);
    DigitalIn jumperPB1(PB_1);

    DigitalIn userButton(BUTTON1);
    TRMutex printer;
    Mutex mutex_test;
    Thread refThread;
    Thread imuThread;
    BufferedSerial bc(PA_0, PA_1, 115200);
    BufferedSerial usbSerial(USBTX, USBRX, 115200);

    // LED Definitions
    DigitalOut led(L25);
    DigitalOut led2(L26);
    DigitalOut led3(L27);
    DigitalOut ledbuiltin(LED1);

    // CHASSIS Definitions
    I2C i2c(I2C_SDA, I2C_SCL);
    BNO055 imu(i2c, IMU_RESET, MODE_IMU);

    // CV STUFF
    BufferedSerial bcJetson(PC_12, PD_2, 921600); // JETSON PORT
    Jetson_read_data jetson_received_data;
    Jetson_read_odom jetson_received_odom;
    Jetson_send_ref jetson_send_ref;
    Jetson_send_data jetson_send_data;

// imu
#ifdef USE_IMU
    BNO055_ANGULAR_POSITION_typedef imuAngles;
#endif

}