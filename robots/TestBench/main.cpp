//mbed-tools sterm -b 115200


#include "main.h"
#include "subsystems/ChassisSubsystem.h"
#include <algorithm>
#include <cstdint>  // For uint64_t
#include <iostream>
#include <chrono>


#define PI 3.14159265

#define UPPERBOUND_DEG 45.0 // Bound of how high turret can point in degrees
#define LOWERBOUND_DEG -30.0 // Bound of how low turret can point in degrees

#define UPPERBOUND_RAD 0.785
#define LOWERBOUND_RAD - 0.524

#define UPPERBOUND_TICKS (UPPERBOUND_DEG/360.0) * 8192 // 1137 ticks above/CCW to 4250, ie 4250-1137 = 3112 absolute position in ticks
#define LOWERBOUND_TICKS (LOWERBOUND_DEG/360.0) * 8192 // 682 ticks below/CW to 4250. ie 4932 abs pos in ticks

#define PITCH_LEVEL_TICKS 4160 // The tick value of level turret pitch. Also used for initial offset


#define PITCH_SPEED_JOYSTICK_SENSE 10.0/330
// [8129/360] = ticks per deg

// add radius measurement here
#define RADIUS 0.5
#define RUNSPIN 1.0

#define JOYSTICK_SENSE_YAW 1.0/90
#define JOYSTICK_SENSE_PITCH 1.0/150
#define MOUSE_SENSE_YAW 1.0/5
#define MOUSE_SENSE_PITCH 1.0/5
#define MOUSE_KB_MULT 0.2

#define PID_POS 1 // Use pitch speed PID
#define PID_SPD 0 // Use pitch position PID


DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);

static BufferedSerial bcJetson(PA_0, PA_1, 115200);
// static BufferedSerial bcJetson(PC_12, PD_2, 115200);
I2C i2c(I2C_SDA, I2C_SCL);
// BNO055 imu(i2c, IMU_RESET, MODE_IMU);

DJIMotor yaw(7, CANHandler::CANBUS_1, GIMBLY, "yaw");
DJIMotor pitch(7, CANHandler::CANBUS_2, GIMBLY);

BufferedSerial pc(USBTX, USBRX); // tx, rx
float yaw_angle;
float yaw_velocity;
float yaw_value = 0;
float pitch_value = 0;
float Pitch_value;
float Yaw;
char nucleo_value[30] = {0};
char jetson_value[30] = {0};




// ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2286); // radius is 9 in
float xRotated, yRotated;
// BNO055_ANGULAR_POSITION_typedef imuAngles; //(-180) - (180)

char yaw_angle_char[4];
char yaw_velocity_char[4];
char pitch_angle_char[4];
char pitch_velocity_char[4];

// idk what this is
static void rotatePoint(float x, float y, double theta, float &xOut, float &yOut) {
    float rad = theta * M_PI / 180.0; // Convert theta to radians
    xOut = x * cos(rad) - y * sin(rad);
    yOut = x * sin(rad) + y * cos(rad);
}


/**
 * Copy float `value` bytes into single bytes in `byteArr` array
 * @param byteArr destination char array for value individual bytes
 * @param value float value to copy into byteArr
 */
void getBytesFromFloat(char* byteArr, float value) {
    std::memcpy(byteArr, &value, sizeof(float));
}

/**
 * Writes 9 bytes of read_buf into received_one and received_two as floats for pitch & yaw positions
 * Used for receiving desired position data from CV in read_buf, write out as 
 * floats to received_one/two.
 * @param read_buf - Source data
 * @param received_one - Destination buffer
 */
void decode_toSTM32(char *read_buf, float &received_one, float &received_two, uint8_t &checksum){
    memcpy(&received_one, read_buf, sizeof(float));
    memcpy(&received_two, read_buf + 4, sizeof(float));
    checksum = read_buf[8];
}

/**
 * Coipes 4 bytes from srcBuf[0] into destBuf[offset]
 * @param srcBuf source buffer
 * @param destBuf destination buffer
 * @param offset the starting position into destBuf
 */
void copy4Char(char* srcBuf, char* destBuf, int offset){
    for(int i = 0; i < 4; i ++){
        destBuf[offset+i] = srcBuf[i];
    }
}

/**
 * Performs a Longitudinal Redundancy Check
 * @param data the data to compute the checksum on
 * @param length the length of data
 */
static uint8_t calculateLRC(const char* data, size_t length) {
    unsigned char lrc = 0;
    for (size_t i = 0; i < length; ++i) {
        lrc += data[i];
        lrc &= 0xff;
    }
    lrc = ((lrc ^ 0xff) + 1) & 0xff;
    return lrc;
}


void jetson_send_feedback_simple(){
    char temp[50] = {0};
    //imu.get_angular_position_quat(&imuAngles);

    //ChassisSpeeds cs = Chassis.m_chassisSpeeds;
    //rotatePoint(cs.vX, cs.vY, imuAngles.yaw, xRotated, yRotated);
    //float yaw_angle = ChassisSubsystem::ticksToRadians(yaw.getData(ANGLE));

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

/**
 * Read motor values and send to CV
 */
float jetson_send_feedback() {
    //char temp[50] = {0};
    // ChassisSpeeds cs = Chassis.m_chassisSpeeds;
    // imu.get_angular_position_quat(&imuAngles);
    //printf("chassis:%f|%f imu:%f\n", cs.vX, cs.vY, imuAngles.yaw);
    //printf("yaw angle %f, yaw velocity %f\n", yaw.getData(ANGLE), yaw.getData(VELOCITY));
    //printf("pitch angle %f, pitch velocity %f \n", pitch.getData(ANGLE), pitch.getData(VELOCITY));
    //printf("IMU Angle: %f\n", float(imuAngles.yaw));

    //GET the x, y value from chassis
    // rotatePoint(cs.vX, cs.vY, imuAngles.yaw, xRotated, yRotated);
    //printf("Rx: %f | Ry: %f", xRotated, yRotated);

    char rotate_x_char[sizeof(float)];
    char rotate_y_char[sizeof(float)];
    getBytesFromFloat(rotate_x_char, xRotated);
    getBytesFromFloat(rotate_y_char, yRotated);

    //GET yaw and pitch data
    float yaw_angle = ChassisSubsystem::ticksToRadians(yaw.getData(ANGLE)); //Ticks
    float yaw_velocity = yaw.getData(VELOCITY)/60.0; //RPM

    float pitch_angle = ChassisSubsystem::ticksToRadians(PITCH_LEVEL_TICKS - pitch.getData(ANGLE));
    float pitch_velocity = pitch.getData(VELOCITY)/60.0;

    // printf("yaw A: %f | yaw v: %f | pitch a: %f | pitch v: %f\n", yaw_angle, yaw_velocity, pitch_angle, pitch_velocity);

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
    copy4Char(rotate_x_char, nucleo_value, startPositions[0]);
    copy4Char(rotate_y_char, nucleo_value, startPositions[1]);
    copy4Char(pitch_angle_char, nucleo_value, startPositions[2]);
    copy4Char(yaw_angle_char, nucleo_value, startPositions[3]);
    copy4Char(pitch_velocity_char, nucleo_value, startPositions[4]);
    copy4Char(yaw_velocity_char, nucleo_value, startPositions[5]);

    //printf("Rx: %d temp: %d | t2nd %d | t3d %d| t4th %d\n",rotate_x_char[0], temp[0], temp[1], temp[2], temp[3]);
    //printf("temp: %s\n", temp);


    //get lrc
    uint8_t lrc = calculateLRC(nucleo_value, 24);
    char lrc_char = static_cast<uint8_t>(lrc);
    //printf("lrc: %d\n", lrc);
    nucleo_value[24] = lrc_char;
    //printf("lrc: %d\n", temp[24]);

    bcJetson.set_blocking(false);
    bcJetson.write(nucleo_value, 30);
    //printf("Rx: %f | Ry: %f | p_a: %f | y_a: %f | p_v: %f | y_v: %f | lrc: %d \n", xRotated, yRotated, pitch_angle, yaw_angle, pitch_velocity, yaw_velocity, lrc);
    
    //make a struct to send data
    ThisThread::sleep_for(3);
    return yaw_angle;
}

/**
 * Read desired pitch and yaw position data from Jetson
 * 
 * @param pitch_move buffer to store desired pitch position
 * @param yaw_move buffer to store desired yaw position
 */
void jetson_read_values(float &pitch_move, float & yaw_move) {
    bcJetson.set_blocking(false);

    ssize_t result = bcJetson.read(jetson_value, 30);
    if (result != -EAGAIN) { // If buffer not empty, decode data. Else do nothing
        // Print raw buffer bytes as decimal integers
        // printf("Raw buffer data: ");
        // printf("\n");

        uint8_t checkSum;
        decode_toSTM32(jetson_value, pitch_move, yaw_move, checkSum);

        if (pitch_move > 100) {
            pitch_move = 0;
        }
        //printf("*** pitch: %f, yaw: %f, checkSum: %d\n", pitch_move, yaw_move, (int)checkSum);
    }
    else{
        // printf("result was empty \n");
    }
}


// think this is bad copy of jetson_read_values
void eccode_value(char *buf, float &received_one, float &received_two, uint8_t &checksum) {
    memcpy(buf, &received_one, sizeof(float));   // 4 bytes
    //memcpy(received_one, buf, sizeof(float));
    memcpy(buf + 4, &received_two, sizeof(float));   // 4 bytes
    // Copy the uint8_t
    checksum = buf[8];   // 1 byte
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


int main(){
    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();

    printf("DesiredPosition CurrentPosition DesiredPitchDegrees");

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    //printf("Current timestamp (us): %lu\n", timeStart);  // Print the timestamp
    int refLoop = 0;
    //jetson_send_feedback();
    int counter = 1;

    /* Pitch Position PID*/
    pitch.setPositionPID(29, 0.17, 6200); // think about D-cap and potentially raising FF. if the setpoint is always higher than actual,
    // then could try to up FF to get there
    // pitch.setSpeedPID(0,0,0);
    pitch.setPositionIntegralCap(3000);

    // /* Yaw Position PID */
    // yaw.setPositionPID(5, 0, 0); // Very simple for now

    /* Yaw Speed PID */
    // yaw.setSpeedPID(3.5, 0, 150);
    // yaw.setSpeedIntegralCap(1000);

    // Old tune
    // pitch.setPositionPID(15, 0, 1700);
    // pitch.setPositionOutputCap(32000);

    // Chassis.setYawReference(&yaw, 2050); // "5604" is the number of ticks of yaw considered to be robot-front
//    Chassis.setSpeedFF_Ks(0.065);

    yaw.setSpeedPID(540, 0, 400);

    PID yawBeyblade(0.32, 0, 550);
    PID yawNonBeyblade(0.15, 0, 550);

    yaw.setSpeedIntegralCap(1000);
    yaw.useAbsEncoder = false;

    yaw.setSpeedOutputCap(24000);

    int ref_yaw;

    // int yawSetPoint = imuAngles.yaw;
    double rotationalPower = 0;

    unsigned long yawTime = us_ticker_read();

    PID sure(0.5,0,1);
    sure.setOutputCap(4000);
    unsigned long timeSure;
    unsigned long prevTimeSure;
    pitch.pidPosition.feedForward = 0;
    pitch.outputCap = 32760;
    pitch.useAbsEncoder = true;

    float currentPitch = 0;
    float desiredPitchPos = 0;
    float pitch_phase = 33 / 180.0 * PI; // 5.69 theoretical
    float InitialOffset_Ticks = 2500;
    float K = 0.38; //0.75 //0.85

    float pitch_ANGLE = 0.0;


    pitch.setPositionPID(22, 0.12, 4000);
    pitch.setPositionIntegralCap(3800);

    int des_pitch_in_ticks = 0;
    int pitch_in_ticks = 0;

    float des_yaw_speed = 0;
    float yaw_ANGLE;
    float yaw_speed;

    //debug
    float pitch_in_deg = 0;
    int ticks_to_motor = 0;
    float currPitch = 0;
    float desiredPosition = 0;
    float diffRealExpectedPitch = 0;

    while(true){
        pitch_ANGLE = 0.0;
        timeStart = us_ticker_read();


        if ((timeStart - loopTimer) / 1000 > 15){
            led = !led;

            //int sizePacket = bcJetson.available();

            jetson_read_values(pitch_ANGLE, yaw_speed);
            //printf("pitchAngle %X  %.3f size %i  \n", pitch_ANGLE, pitch_ANGLE, sizePacket);
                // calculate desired 
                if (pitch_ANGLE != 0) {
                    pitch_in_ticks = ChassisSubsystem::radiansToTicks(pitch_ANGLE);
                }
                pitch_in_deg = (pitch_in_ticks/8192.0) * 360;
                ticks_to_motor = PITCH_LEVEL_TICKS - pitch_in_ticks;


            refLoop++;
            // remoteRead();

            if (refLoop >= 5){
                
                refereeThread(&referee);
                refLoop = 0;
                
                // jetson_read_values(pitch_ANGLE, yaw_ANGLE);
                
                des_yaw_speed += yaw_speed;
                //desire = ChassisSubsystem::radiansToTicks(jetson_send_feedback());

                // calculate desired 
                if (pitch_ANGLE != 0) {
                    pitch_in_ticks = ChassisSubsystem::radiansToTicks(pitch_ANGLE);
                }
                pitch_in_deg = (pitch_in_ticks/8192.0) * 360;
                ticks_to_motor = PITCH_LEVEL_TICKS - pitch_in_ticks;

                //-----------DEBUG PITCH DATA RECIEVED-------------------------

                //current angle and desired change
                currPitch = (pitch.getData(ANGLE)/8192.0)*360.0;
                desiredPosition = ( (PITCH_LEVEL_TICKS - pitch_in_ticks ) /8192.0)*360;

                // // //actual change - expected change. negative means absolutely undershooting, positive means overshooting.
                diffRealExpectedPitch =  abs(currPitch) - abs(desiredPosition);

                // //print
                // if (printCount >= 2) {
                    printf("%.3f %.3f %.3f \n", desiredPosition, currPitch, pitch_in_deg);
                    //printf("DeP: %.3f CuP: %.3f DiReExP: %.3f \n", desiredPosition, currPitch, diffRealExpectedPitch);
                    //printf("pitch in ticks: %i  \n", pitch_in_ticks);
                    //printf("pitch in degrees: %f  \n", pitch_in_deg);
                    //printf("ticks to motor: %i \n \n \n", ticks_to_motor);

                /* Original code idk what this is about */
                float yaw_in_degrees = (ChassisSubsystem::radiansToTicks(yaw_ANGLE)/8192)*360;
                
                yaw_in_degrees += 360;
                while(yaw_in_degrees>360){
                    yaw_in_degrees -= 360;
                }
                
                /* ****** */

                /* Catch pitch beyond thresholds*/
                if (pitch_in_ticks <= LOWERBOUND_TICKS) {
                    pitch_in_ticks = LOWERBOUND_TICKS;
                }
                else if (pitch_in_ticks >= UPPERBOUND_TICKS) {
                    pitch_in_ticks = UPPERBOUND_TICKS;
                }

                /* Feed forward to account for gravity*/
                float FF = -8500 * cos(pitch_ANGLE);
                pitch.pidPosition.feedForward = FF;

                // pitch_in_ticks is relative to level = 0 ticks. PITCH_LEVEL_TICKS - pitch_in_ticks = abs position in ticks
                pitch.setPosition(PITCH_LEVEL_TICKS - pitch_in_ticks);
                
                // yaw.setPosition(yaw_in_degrees * (360.0/8192));
                // des_yaw_speed = 0;
                // yaw.setSpeed(des_yaw_speed * 100);

                // printf("%d, %d\n", (int)(des_yaw_speed * 100), yaw>>POWEROUT);

                //BEYBLADE CODE

//                if (desiredPitchPos >= LOWERBOUND) {
//                    desiredPitchPos = LOWERBOUND;
//                }
//                else if (desiredPitchPos <= UPPERBOUND) {
//                    desiredPitchPos = UPPERBOUND;
//                }
//
//                float FF = K * sin((desiredPitchPos / 180 * PI) - pitch_phase); // output: [-1,1]
//                pitch.pidPosition.feedForward = int((INT16_T_MAX) * FF);
//                pitch.setPosition(int((desiredPitchPos / 360) * TICKS_REVOLUTION + InitialOffset_Ticks));
//
//                yawSetPoint = yaw_in_degrees;
//                //yawSetPoint -= (remote.rightX() / 90 + 360) % 360;
//                yaw.setSpeed(5 * yawNonBeyblade.calculatePeriodic(DJIMotor::s_calculateDeltaPhase(yawSetPoint,imuAngles.yaw+180, 360), timeSure - prevTimeSure));

                //BEYBLADE CODE


                // imu.get_angular_position_quat(&imuAngles);

                //pitch.setPosition(yaw_value); //need to be a value between like 2000 - 8000s, in ticks
                //yaw_in_ticks = 0;
                //pitch_in_ticks = 0;
                //printf("yaw value: %d, pitch value: %d \n", yaw.getData(ANGLE), pitch.getData(ANGLE)); //the value ticks is in int
                //pitch.setPosition(pitch_in_ticks);

                /*
                pitch.setPosition(pitch_in_ticks);
                yaw.setPosition(yaw_in_ticks);
                 */
                //pitch.setPosition(1800);
                //yaw.setPosition(1800);




                // ad_and_print();
           //printf("TEST");

                //PRINTFF doesn't WROK
                //printff("ang%f t%d d%f FF%f\n", (((pitch>>ANGLE) - InitialOffset_Ticks) / TICKS_REVOLUTION) * 360, pitch>>ANGLE, desiredPitchPos, K * sin((desiredPitchPos / 180 * PI) - pitch_phase)); //(desiredPitchPos / 360) * TICKS_REVOLUTION + InitialOffset_Ticks

            }

            remoteRead();
            // Chassis.periodic();

            loopTimer = timeStart;
            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        jetson_send_feedback();

        unsigned long timeEnd = us_ticker_read();
        ThisThread::sleep_for(1ms);
        // printf("Loop execution time: %lu us (%.2f ms)\n", (timeEnd - timeStart), (timeEnd - timeStart) / 1000.0);
    }
}


//mbed-tools sterm -b 115200

