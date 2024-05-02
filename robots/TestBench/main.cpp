//mbed-tools sterm -b 115200


#include "main.h"
#include "subsystems/ChassisSubsystem.h"
#include <algorithm>
#include <cstdint>  // For uint64_t
DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);
static BufferedSerial bcJetson(PA_0, PA_1, 115200);
I2C i2c(I2C_SDA, I2C_SCL);
BNO055 imu(i2c, IMU_RESET, MODE_IMU);
DJIMotor yawOne(5, CANHandler::CANBUS_1, GM6020, "testMotor");
//DJIMotor yaw(5, CANHandler::CANBUS_1, GIMBLY);
DJIMotor pitch(7, CANHandler::CANBUS_2, GIMBLY); // right


ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2286); // radius is 9 in
float xRotated, yRotated;
BNO055_ANGULAR_POSITION_typedef imuANGLE; //(-180) - (180)

char yaw_angle_char[4];
char yaw_velocity_char[4];
char pitch_angle_char[4];
char pitch_velocity_char[4];


static void rotatePoint(float x, float y, double theta, float &xOut, float &yOut) {
    float rad = theta * M_PI / 180.0; // Convert theta to radians
    xOut = x * cos(rad) - y * sin(rad);
    yOut = x * sin(rad) + y * cos(rad);
}


void getBytesFromFloat(char* byteArr, float value) {
    std::memcpy(byteArr, &value, sizeof(float));
}



void copy4Char(char* toCopy, char* copyInto, int begin){
    for(int i = 0; i < 4; i ++){
        copyInto[begin+i] = toCopy[i];
    }
}



static uint8_t calculateLRC(const char* data, size_t length) {
    unsigned char lrc = 0;
    for (size_t i = 0; i < length; ++i) {
        lrc += data[i];
        lrc &= 0xff;
    }
    lrc = ((lrc ^ 0xff) + 1) & 0xff;
    return lrc;
}


void read_and_print_simple(){
    char temp[50] = {0};
    //imu.get_angular_position_quat(&imuANGLE);


    //ChassisSpeeds cs = Chassis.m_chassisSpeeds;
    //rotatePoint(cs.vX, cs.vY, imuANGLE.yaw, xRotated, yRotated);
    //float yaw_angle = ChassisSubsystem::ticksToRadians(yawOne.getData(ANGLE));

    bcJetson.set_blocking(false);
    temp[0] = 'W';
//temp[0] = 'h';
    //double num = 1234;
    //split_double(temp, num);

    bcJetson.write(temp, 50);

    //bcJetson.read(temp, 50);
    printff("%s\n",temp);
    //printff("formatted output %f\n", 9.0);
}

void read_and_print(){
    char temp[50] = {0};
    ChassisSpeeds cs = Chassis.m_chassisSpeeds;
    imu.get_angular_position_quat(&imuANGLE);
    //printf("chassis:%f|%f imu:%f\n", cs.vX, cs.vY, imuANGLE.yaw);
    //printf("yaw angle %f, yaw velocity %f\n", yawOne.getData(ANGLE), yawOne.getData(VELOCITY));
    //printf("pitch angle %f, pitch velocity %f \n", pitch.getData(ANGLE), pitch.getData(VELOCITY));
    //printf("IMU Angle: %f\n", float(imuANGLE.yaw));

    //GET the x, y value from chassis
    rotatePoint(cs.vX, cs.vY, imuANGLE.yaw, xRotated, yRotated);
    //printf("Rx: %f | Ry: %f", xRotated, yRotated);

    char rotate_x_char[sizeof(float)];
    char rotate_y_char[sizeof(float)];
    getBytesFromFloat(rotate_x_char, xRotated);
    getBytesFromFloat(rotate_y_char, yRotated);

    //GET yaw and pitch data
    float yaw_angle = ChassisSubsystem::ticksToRadians(yawOne.getData(ANGLE)); //Ticks
    float yaw_velocity = yawOne.getData(VELOCITY)/60; //RPM

    float pitch_angle = ChassisSubsystem::ticksToRadians(pitch.getData(ANGLE));
    float pitch_velocity = pitch.getData(VELOCITY)/60;

    //printf("yaw A: %f | yaw v: %f | pitch a: %f | pitch v: %f\n", yaw_angle, yaw_velocity, pitch_angle, pitch_velocity);

    char yaw_angle_char[4];
    char yaw_velocity_char[4];
    char pitch_angle_char[4];
    char pitch_velocity_char[4];

    getBytesFromFloat(yaw_angle_char, yaw_angle);
    getBytesFromFloat(yaw_velocity_char, yaw_velocity);
    getBytesFromFloat(pitch_angle_char, pitch_angle);
    getBytesFromFloat(pitch_velocity_char, pitch_velocity);


    //put the data into temp
    int startPositions[7] = {0, 4, 8,12, 16, 20, 24};
    copy4Char(rotate_x_char, temp, startPositions[0]);
    copy4Char(rotate_y_char, temp, startPositions[1]);
    copy4Char(pitch_angle_char, temp, startPositions[2]);
    copy4Char(yaw_angle_char, temp, startPositions[3]);
    copy4Char(pitch_velocity_char, temp, startPositions[4]);
    copy4Char(yaw_velocity_char, temp, startPositions[5]);

    //printf("Rx: %d temp: %d | t2nd %d | t3d %d| t4th %d\n",rotate_x_char[0], temp[0], temp[1], temp[2], temp[3]);
    //printf("temp: %s\n", temp);


    //get lrc
    uint8_t lrc = calculateLRC(temp, 24);
    char lrc_char = static_cast<uint8_t>(lrc);
    //printf("lrc: %d\n", lrc);
    temp[24] = lrc_char;
    //printf("lrc: %d\n", temp[24]);



    bcJetson.set_blocking(false);
    bcJetson.write(temp, 50);
    printf("Rx: %f | Ry: %f | p_a: %f | y_a: %f | p_v: %f | y_v: %f | lrc: %d \n", xRotated, yRotated, pitch_angle, yaw_angle, pitch_velocity, yaw_velocity, lrc);

}


void move_yaw(float value){
    yawOne.setPosition(value);

}

void move_pitch(float value){
    pitch.setPosition(value);
}

//Yaw Incorporated
int calculateDeltaYaw(int ref_yaw, int beforeBeybladeYaw)
{
    int deltaYaw = beforeBeybladeYaw - ref_yaw;

    if (abs(deltaYaw) > 180)
    {
        if (deltaYaw > 0)
            deltaYaw -= 360;
        else
            deltaYaw += 360;
    }
    return deltaYaw;
}

Thread imuThread;

void runImuThread()
{
    //chassis.initializeImu();
    while (true)
    {
        //chassis.readImu();
        ThisThread::sleep_for(25);
    }
}


int main(){

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    int refLoop = 0;
    //read_and_print();
    int counter = 1;

    Chassis.setYawReference(&yawOne, 2050); // "5604" is the number of ticks of yawOne considered to be robot-front
    Chassis.setSpeedFF_Ks(0.065);

    yawOne.setSpeedPID(0.5, 0, 200);
    PID yawBeyblade(50, 0, 5);
    PID yawNonBeyblade(100, 0, 50);

    yawOne.setSpeedIntegralCap(1000);
    yawOne.useAbsEncoder = false;

    int ref_yaw;

    int yawSetPoint = imuANGLE.yaw;
    double rotationalPower = 0;

    unsigned long yawTime = us_ticker_read();

    PID sure(0.5,0,1);
    sure.setOutputCap(4000);
    unsigned long timeSure;
    unsigned long prevTimeSure;



    while(true){
        timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > 25){
            led2 = !led2;

            refLoop++;
            int arr[5] = {1, 2, 3, 4, 5};

            if (refLoop >= 5){
                refereeThread(&referee);
                refLoop = 0;
                led = !led;

                //counter = counter ;
                counter += 1;
                move_yaw((counter*50)%360);
                printf("moved to this degree %d\n", counter*20);

                //printf("%f\n", 999.9);
                //


               // read_and_print();
           //printf("TEST");






            }

            //read_and_print();


            remoteRead();
            Chassis.periodic();

            loopTimer = timeStart;
            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}
