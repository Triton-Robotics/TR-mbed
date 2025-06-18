//mbed-tools sterm -b 115200


#include "main.h"
#include "subsystems/ChassisSubsystem.h"
#include <algorithm>
#include <cstdint>  // For uint64_t
#include <iostream>
#include <chrono>

#define READ_DEBUG 0
#define MAGICBYTE 0xEE
#define PI 3.14159265

#define UPPERBOUND_DEG 10.0 // Bound of how high turret can point in degrees
//sentry 2025. -9 is probably more accurate but this is a buffer so the usb doesn't keep breaking

#define LOWERBOUND_DEG -26.0 // Bound of how low turret can point in degrees

#define UPPERBOUND_RAD 0.174533
#define LOWERBOUND_RAD -0.453786

// #define UPPERBOUND_RAD 0.785
// #define LOWERBOUND_RAD - 0.524

#define UPPERBOUND_TICKS (UPPERBOUND_DEG/360.0) * 8192 // 1137 ticks above/CCW to 4250, ie 4250-1137 = 3112 absolute position in ticks
#define LOWERBOUND_TICKS (LOWERBOUND_DEG/360.0) * 8192 // 682 ticks below/CW to 4250. ie 4932 abs pos in ticks

#define GEAR_RATIO 2
#define PITCH_LEVEL_TICKS 6445 // The tick value of level turret pitch. Also used for initial offset


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

// add radius measurement here
#define RADIUS 0.5

#define PID_POS 1 // Use pitch speed PID
#define PID_SPD 0 // Use pitch position PID


DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);

static BufferedSerial bcJetson(PC_12, PD_2, 115200);  //JETSON PORT
// static BufferedSerial bcJetson(PC_12, PD_2, 115200);
I2C i2c(I2C_SDA, I2C_SCL);
BNO055 imu(i2c, IMU_RESET, MODE_IMU);

DJIMotor yaw(6, CANHandler::CANBUS_1, GIMBLY, "yaw");
DJIMotor pitch(5, CANHandler::CANBUS_2, GIMBLY);
DJIMotor indexerL(5, CANHandler::CANBUS_2, C610);
DJIMotor indexerR(6, CANHandler::CANBUS_2, C610);

ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2286); // radius is 9 in

DJIMotor RTOPFLYWHEEL(1, CANHandler::CANBUS_2, M3508_FLYWHEEL);
DJIMotor LTOPFLYWHEEL(2, CANHandler::CANBUS_2, M3508_FLYWHEEL);
DJIMotor RBOTTOMFLYWHEEL(4, CANHandler::CANBUS_2, M3508_FLYWHEEL);
DJIMotor LBOTTOMFLYWHEEL(3, CANHandler::CANBUS_2, M3508_FLYWHEEL);

BufferedSerial pc(USBTX, USBRX); // tx, rx
float yaw_angle;
float yaw_velocity;
float yaw_value = 0;
float pitch_value = 0;
float Pitch_value;
float Yaw;
char nucleo_value[30] = {0};
char jetson_value[200] = {0};

struct fromJetson{
    float pitch_angle;
    float yaw_angle;
    uint8_t checksum;
};

// ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2286); // radius is 9 in
float xRotated, yRotated;
// BNO055_ANGULAR_POSITION_typedef imuAngles; //(-180) - (180)

char yaw_angle_char[4];
char yaw_velocity_char[4];
char pitch_angle_char[4];
char pitch_velocity_char[4];

void setFlyWheelSpeed(int speed)
{
    LTOPFLYWHEEL.setSpeed(-speed);
    LBOTTOMFLYWHEEL.setSpeed(speed);
    RTOPFLYWHEEL.setSpeed(speed);
    RBOTTOMFLYWHEEL.setSpeed(-speed);
}
void setFlyWheelPower(int speed)
{
    LTOPFLYWHEEL.setPower(-speed);
    LBOTTOMFLYWHEEL.setPower(speed);
    RTOPFLYWHEEL.setPower(speed);
    RBOTTOMFLYWHEEL.setPower(-speed);
}


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
void decode_toSTM32(char *read_buf, float &received_one, float &received_two, char &received_three, uint8_t &checksum){
    memcpy(&received_one, read_buf, sizeof(float));
    memcpy(&received_two, read_buf + 4, sizeof(float));
    memcpy(&received_three, read_buf + 8, sizeof(char)); // shooting indicator
    checksum = read_buf[9];
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
    char rotate_x_char[sizeof(float)];
    char rotate_y_char[sizeof(float)];
    getBytesFromFloat(rotate_x_char, xRotated);
    getBytesFromFloat(rotate_y_char, yRotated);

    //GET yaw and pitch data
    float yaw_angle = ChassisSubsystem::ticksToRadians(yaw.getData(ANGLE)); //Ticks
    float yaw_velocity = yaw.getData(VELOCITY)/60.0; //RPM

    float pitch_angle = ChassisSubsystem::ticksToRadians((PITCH_LEVEL_TICKS - pitch.getData(ANGLE)) / GEAR_RATIO);
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

    // 0  1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25    - 26 total bytes
    // EE x x x x y y y y p p  p  p  y  y  y  y  pv pv pv pv yv yv yv yv checksum
    //put the data into temp
    int startPositions[6] = {1, 5, 9, 13, 17, 21};
    nucleo_value[0] = MAGICBYTE;
    copy4Char(rotate_x_char, nucleo_value, startPositions[0]);
    copy4Char(rotate_y_char, nucleo_value, startPositions[1]);
    copy4Char(pitch_angle_char, nucleo_value, startPositions[2]);
    copy4Char(yaw_angle_char, nucleo_value, startPositions[3]);
    copy4Char(pitch_velocity_char, nucleo_value, startPositions[4]);
    copy4Char(yaw_velocity_char, nucleo_value, startPositions[5]);

    //printf("Rx: %d temp: %d | t2nd %d | t3d %d| t4th %d\n",rotate_x_char[0], temp[0], temp[1], temp[2], temp[3]);
    //printf("temp: %s\n", temp);


    //get lrc
    uint8_t lrc = calculateLRC(nucleo_value + 1, 24); //exclude header byte
    char lrc_char = static_cast<uint8_t>(lrc);
    nucleo_value[25] = lrc_char;

    bcJetson.set_blocking(false);
    bcJetson.write(nucleo_value, 26); //changed from 30
    //printf("Rx: %f | Ry: %f | p_a: %f | y_a: %f | p_v: %f | y_v: %f | lrc: %d \n", xRotated, yRotated, pitch_angle, yaw_angle, pitch_velocity, yaw_velocity, lrc);
    
    //make a struct to send data
    // ThisThread::sleep_for(3);
    return yaw_angle;
}

/**
 * Read desired pitch and yaw position data from Jetson
 * 
 * @param pitch_move buffer to store desired pitch position
 * @param yaw_move buffer to store desired yaw position
 */

 //return 1 if successful match, 0 if no match but buffer available, -1 for unreadable
ssize_t jetson_read_values(float &pitch_move, float & yaw_move, char &shoot_switch) {
    bcJetson.set_blocking(false);
    ssize_t fillArrayCheck = 10;
    int i;
    unsigned int numBytes = 0;            //counts bytes read aka size of jetson_values
    uint8_t checkSum;                     //last bit in packet            

    if ( bcJetson.readable() ) {
        while ( (fillArrayCheck >= 10) && (numBytes < 200) ) {
            fillArrayCheck = bcJetson.read(jetson_value+numBytes, 10); //keep adding 10 bytes throughout array until buffer empty
            numBytes += fillArrayCheck;
        }

        if (READ_DEBUG) { 
            for ( i = 0; i< numBytes ; ++i) {
                printf("%d ", jetson_value[i]);
            }
            printf("\nbytes: %d\n", numBytes);
        }

        //starting at the very last index of jetson_values (numbytes - 1)
        //i - 10 is the magic byte, and we will use next 9 bytes for checksum which is at i; exclude magic byte in checksum
        //if we meet the match conditions, decode values, clear buffer, return last amount of bytes read
        //if no match found, print no match
        //if buffer empty, print empty
        for( i = numBytes - 1 ; i >= 10 ; --i) {
            if (READ_DEBUG) { 
                for (int j = i - 10; j <= i; ++j) {
                    printf("%d ",jetson_value[j]);
                }
                printf("\n");
                printf("%d %d %d\n", calculateLRC(&jetson_value[i - 9], 9), jetson_value[i], jetson_value[i-10]);
             }

            //calculating checksum w/ header bytes
            if ( (calculateLRC(&jetson_value[i - 9], 9) == jetson_value[i]) && (jetson_value[i] != 0) && (jetson_value[i-10] == MAGICBYTE) ){
                
                if (READ_DEBUG) { printf("match\n"); }

                decode_toSTM32(&jetson_value[i-9], pitch_move, yaw_move, shoot_switch, checkSum);
                return fillArrayCheck;
            }
        }
        if (READ_DEBUG) {  printf("\nno match\n"); }
        return 0;
        
    } else {
        if (READ_DEBUG) { printf("\nempty\n"); }
    }
    return fillArrayCheck;
}
        

//Send yourself packets through buffer
void self_sending_data() {
    //Send Buffer stuff
    char firstByteIndicator = 0;
    char SecondByteIndicator = 0;
    char txCheckSum = 0;
    ssize_t readResult;
    ssize_t fillBufferDebug;
    ssize_t readBufferDebug;

    float test_yaw = 0;
    float test_pitch = 0;
    bool direction = 0;
    char test_packet[11] = {0};
    char shoot_on = 0;
    int shoot_count = 0;
    uint8_t check_sum;

    unsigned int magicByte = 0xEE;




    //PRINTLOOP
    int printLoop = 0;
    int byteCount = 0;
    
    if ( printLoop > 7 ) { //determines how many bad bytes we want to send, starting from end of buffer you will have to sort through 10 - # packets 

        if (DEBUG) {
            printf("S: ");
            for (int i = 0 ; i < 11 ; ++i) {
                test_packet[i] = i;
                printf("%d ", test_packet[i]);
            }
            printf("\n");
        }
        fillBufferDebug = bcJetson.write(test_packet, 11);
        bcJetson.sync();
        
    } else {           
        if (shoot_count == 50) {
            if (shoot_on == 0) {
                ++shoot_on;
            }
            else { shoot_on = 1; }
            shoot_count = 0;
        }

        //incrementing pitch and Yaw myself Code--------------
        if ( direction == 0) {
            test_yaw += 0.01;
            test_pitch += 0.001; //don't need to worry about bounds, handled below

            if (test_yaw >= 0.4){
                direction = 1;
            }
        }
        else {
            test_yaw -= 0.01;
            test_pitch -= 0.001;
            if (test_yaw <= -0.4){
                direction = 0;
            } 
        }
        if ( abs(test_yaw) <= 0.0001 ) {
            test_yaw = 0;
        }

        //Forming packet
        memcpy(test_packet, &magicByte, sizeof(char));
        memcpy(test_packet + 1, &test_pitch, sizeof(float));
        memcpy(test_packet + 5, &test_yaw, sizeof(float));
        memcpy(test_packet + 9, &shoot_on, sizeof(char)); // shooting indicator

        // //checksum
        check_sum = calculateLRC(test_packet, 10);
        memcpy(test_packet + 10, &check_sum, sizeof(char)); // shooting indicator
        
        if (DEBUG) {
            printf("S: ");
            for (int i = 0 ; i < 11 ; ++i) {
                printf("%d ", test_packet[i]);
            }
            printf("\n");
        }

        //fill buffer
        bcJetson.write( &test_packet, 11);
        bcJetson.sync();
        //basic_bitch_read();
    }
    if (DEBUG) { printf("\nloop: %d byteCnt: %d\n",printLoop, byteCount); }
    
    ++printLoop;
    ++shoot_count;
    byteCount += 11;
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

void basic_bitch_read(){
    bool yes = false;
    fromJetson in = {0};
    int point = 0;
    while(bcJetson.readable()){
        yes = true;
        // char inByte;
        // bcJetson.read(&inByte, 1);
        // printf("%x ",inByte);
        // char* bytes = reinterpret_cast<char*>(&in)
        // bytes[point++] = inByte;
        // if(point == 9){
        //     bcJetson.flush()
        //     break;
        // }
        bcJetson.read(&in, 9);
    }
    if(yes)
        printf("\n");
}

#if 1
int main(){
    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();

    

    unsigned long loopTimer = us_ticker_read();
    unsigned long loopTimerCV = us_ticker_read();
    unsigned long timeStart;
    
    //printf("Current timestamp (us): %lu\n", timeStart);  // Print the timestamp
    int refLoop = 0;
    jetson_send_feedback();
    int counter = 1;

    //shooting
    char shoot_toggle;

    //shooting mechanics
    bool shoot = 0;
    int shootTargetPosition = 36*8190 ;
    bool shootReady = false;
    int debugShooting;



    //Buffer stuff
    int totalBytes = 0;
    int packetLoopCount = 0;
    char test_packetRx[200] = {0};
    char *arrayPtr = test_packetRx;

        //Send Buffer stuff
        char firstByteIndicator = 0;
        char SecondByteIndicator = 0;
        char txCheckSum = 0;
        ssize_t readResult;
        ssize_t fillBufferDebug;
        ssize_t readBufferDebug;

        float test_yaw = 0;
        float test_pitch = 0;
        bool direction = 0;
        char test_packet[11] = {0};
        char shoot_on = 0;
        int shoot_count = 0;
        uint8_t check_sum;
        unsigned int magicByte = 0xEE;
    
    //Rx
    float pitch_CV_angle = 0;
    float yaw_CV_angle = 0;



    //PRINTLOOP
    int printLoop = 0;
    int byteCount = 0;

    ChassisSpeeds cs;

    int dticks = 0;

    while(true){
        // pitch_CV_angle = 0.0;
        timeStart = us_ticker_read();

        //CV loop runs every 1ms
        if((timeStart - loopTimerCV) / 1000 > 0) { 
            loopTimerCV = timeStart;
            //jetson_send_feedback(); //  __COMENTED OUT LOOLOOKOKOLOOOOKO HERHEHRERHEHRHE

                if ( printLoop > 7 ) { //determines how many bad bytes we want to send, starting from end of buffer you will have to sort through 10 - # packets 
                    if ( DEBUG ) {
                        printf("S: ");
                        for (int i = 0 ; i < 11 ; ++i) {
                            test_packet[i] = i;
                            printf("%d ", test_packet[i]);
                        }
                        printf("\n");
                    }
                    //self_sending_data();
                    fillBufferDebug = bcJetson.write(test_packet, 11);
                    bcJetson.sync();
                    
                } else {           
                    if (shoot_count == 50) {
                        if (shoot_on == 0) {
                            ++shoot_on;
                        }
                        else { shoot_on = 1; }
                        shoot_count = 0;
                    }

                    // //incrementing pitch and Yaw myself Code--------------
                    if ( direction == 0) {
                        test_yaw += 0.01;
                        test_pitch += 0.001; //don't need to worry about bounds, handled below

                        if (test_yaw >= 0.4){
                            direction = 1;
                        }
                    }
                    else {
                        test_yaw -= 0.01;
                        test_pitch -= 0.001;
                        if (test_yaw <= -0.4){
                            direction = 0;
                        } 
                    }
                    if ( abs(test_yaw) <= 0.0001 ) {
                        test_yaw = 0;
                    }

                    //Forming packet
                    memcpy(test_packet, &magicByte, sizeof(char));
                    memcpy(test_packet + 1, &test_pitch, sizeof(float));
                    memcpy(test_packet + 5, &test_yaw, sizeof(float));
                    memcpy(test_packet + 9, &shoot_on, sizeof(char)); // shooting indicator

                    // //checksum
                    check_sum = calculateLRC(test_packet+1, 9);
                    memcpy(test_packet + 10, &check_sum, sizeof(char)); // shooting indicator
                    if ( DEBUG ) {
                        printf("S: ");
                        for (int i = 0 ; i < 11 ; ++i) {
                            printf("%d ", test_packet[i]);
                        }
                        printf("\n");
                    }
                    //fill buffer
                    bcJetson.write( &test_packet, 11);
                    bcJetson.sync();
                    //basic_bitch_read();
                }
            if (DEBUG) { printf("\nloop: %d byteCnt: %d\n",printLoop, byteCount); }
            
            ++printLoop;
            ++shoot_count;
            byteCount += 11;
            led2 = !led2;
        }

        if ((timeStart - loopTimer) / 1000 > 15){
            //printf("\ntimeStart: %lu tim\n");
            loopTimer = timeStart;
            led = !led;
            
            refLoop++;
            // remoteRead();
            remoteRead();
            Chassis.periodic();
            cs = Chassis.getChassisSpeeds();

            if (refLoop >= 5){
                refereeThread(&referee);

                refLoop = 0;
            }


            //recieve one packet of data
            // printf("\n\nRe\n");
            // if ( bcJetson.readable() ) {
            //     //printf("received: ");
            //     readBufferDebug = bcJetson.read( (arrayPtr + packetLoopCount), 10);
            //     // for (int i = 0 ; i < 100 ; ++i) {
            //     //     printf("%d ", test_packetRx[i]);
            //     // }
            // }
            //read 9 bytes of buffer, first two being indicators
            // packetLoopCount += 10;

            // //printf("\ntest loop: %d\n", packetLoopCount);

            // //clear the buffer of all data
            // if ( packetLoopCount >= 50 ) {
            //     printf("\n\n-CLEAR-\n");
            //     if (bcJetson.readable() ){ 
            //         while ( bcJetson.readable() ) {
            //             readBufferDebug = bcJetson.read(test_packetRx, 1);
            //             //printf("bytes read: %d", readBufferDebug);
            //             // for (int i = 0 ; i < 10 ; ++i) {
            //             //     printf("%d ", test_packetRx[i]);
            //             // }
            //             //printf(" ");
            //             totalBytes += 10;
            //         }
            //         //printf("total: %d\n", totalBytes);
            //     }
            //     else{ printf("empty\n\n"); }
            //     totalBytes = 0;
            //     packetLoopCount = 0;
            // }

            if ( printLoop >= 10 ) {
                readResult = jetson_read_values(pitch_CV_angle, yaw_CV_angle, shoot_toggle);
                printf("\ndeBg: %d Pi: %.3f Ya: %.3f Sh: %d\n\n\n", readResult, pitch_CV_angle, yaw_CV_angle, shoot_toggle);
                printLoop = 0;
                byteCount = 0;
            }
            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        // jetson_send_feedback();
        // ThisThread::sleep_for(1ms);
    }
}
#endif

//CV PACKET DESCRIPTION
//EMBED TO CV:
//4 ChassisXVelo
//4 ChassisYVelo
//4 Pitch f32
//4 Yaw f32
//4 PitchVelo
//4 YawVelo
//ADD 4 IMUHeading
//ADD 12 XYZAccel


#include "main.h"
#include "subsystems/ChassisSubsystem.h"

DigitalOut led(L25);
DigitalOut led2(L26);
DigitalOut led3(L27);
DigitalOut ledbuiltin(LED1);

//CONSTANTS
constexpr float GEAR_RATIO = 2.0;
constexpr float LOWERBOUND = 35.0;
constexpr float UPPERBOUND = -15.0;

constexpr float BEYBLADE_OMEGA = 4.0;

// constexpr float JOYSTICK_SENSITIVITY_YAW = 1.0/90;
// constexpr float JOYSTICK_SENSITIVITY_PITCH = 1.0/150;
// constexpr float MOUSE_SENSITIVITY_YAW = 1.0/5;
// constexpr float MOUSE_SENSITIVITY_PITCH = 1.0/5;

//DEGREES PER SECOND AT MAX
constexpr float JOYSTICK_SENSITIVITY_YAW_DPS = 180.0; 
constexpr float JOYSTICK_SENSITIVITY_PITCH_DPS = 180.0;
constexpr float MOUSE_SENSITIVITY_YAW_DPS = 1.0;
constexpr float MOUSE_SENSITIVITY_PITCH_DPS = 1.0;

constexpr int OUTER_LOOP_DT_MS = 15;

constexpr int PRINT_FREQUENCY = 20; //the higher the number, the less often

constexpr float CHASSIS_FF_KICK = 0.065;

constexpr int FLYWHEEL_SPEED = 7000;

#define USE_IMU

//CV STUFF
static BufferedSerial bcJetson(PC_12, PD_2, 115200);  //JETSON PORT
float xRotated, yRotated;

char yaw_angle_char[4];
char yaw_velocity_char[4];
char pitch_angle_char[4];
char pitch_velocity_char[4];

char nucleo_value[30] = {0};
char jetson_value[30] = {0};

//CV
float CV_pitch_angle_radians = 0.0;
float CV_yaw_angle_radians = 0.0;
char CV_shoot = 0;

//CHASSIS DEFINING
I2C i2c(I2C_SDA, I2C_SCL);

BNO055 imu(i2c, IMU_RESET, MODE_IMU);
ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2286); // radius is 9 in
DJIMotor yaw(7, CANHandler::CANBUS_1, GIMBLY,"Yeah");
DJIMotor pitch(5, CANHandler::CANBUS_2, GIMBLY,"Peach"); // right
DJIMotor yaw2(6, CANHandler::CANBUS_1, GIMBLY,"Ye2"); // left, not functioning

DJIMotor indexerL(5, CANHandler::CANBUS_2, C610,"IndexerL");
DJIMotor indexerR(6, CANHandler::CANBUS_2, C610,"IndexerR");
DJIMotor RFLYWHEEL_U(1, CANHandler::CANBUS_2, M3508,"RightFlyU");
DJIMotor LFLYWHEEL_U(2, CANHandler::CANBUS_2, M3508,"LeftFlyU");
DJIMotor RFLYWHEEL_D(3, CANHandler::CANBUS_2, M3508,"RightFlyD");
DJIMotor LFLYWHEEL_D(4, CANHandler::CANBUS_2, M3508,"LeftFlyD");

#ifdef USE_IMU
BNO055_ANGULAR_POSITION_typedef imuAngles;
#endif

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
void decode_toSTM32(char *read_buf, float &received_one, float &received_two, char &received_three, uint8_t &checksum){
    memcpy(&received_one, read_buf, sizeof(float));
    memcpy(&received_two, read_buf + 4, sizeof(float));
    memcpy(&received_three, read_buf + 8, sizeof(char)); // shooting indicator
    checksum = read_buf[9];
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

    float pitch_angle = ChassisSubsystem::ticksToRadians((6500 - pitch.getData(ANGLE)) / GEAR_RATIO);
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
    int startPositions[7] = {0, 4, 8, 12, 16, 20, 24};
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
    // ThisThread::sleep_for(3);
    return yaw_angle;
}

/**
 * Read desired pitch and yaw position data from Jetson
 * 
 * @param pitch_move buffer to store desired pitch position
 * @param yaw_move buffer to store desired yaw position
 */
ssize_t jetson_read_values(float &pitch_move, float & yaw_move, char &shoot_switch) {
    bcJetson.set_blocking(false);
    char dumbByte;

    ssize_t result = bcJetson.read(jetson_value, 10);
    // for (int i = 0 ; i < 10 ; ++i ) {
    //     printf("%d ", jetson_value[i]);
    // }
    // printf("\n");

    if (result != -EAGAIN) { // If buffer not empty, decode data. Else do nothing
        // Print raw buffer bytes as decimal integers
        // printf("Raw buffer data: ");
        // printf("\n");

        uint8_t checkSum = jetson_value[9];
        uint8_t theoryCheck = calculateLRC(jetson_value,9);
        if(checkSum == theoryCheck){
            decode_toSTM32(jetson_value, pitch_move, yaw_move, shoot_switch, checkSum);
            //printf("Rx Pitch: %.3f Yaw: %.3f Shoot: %d Check: %d\nFIN\n\n", jetson_value, pitch_move, yaw_move, shoot_switch, checkSum);
        }
        else{
            led3 = !led3;
        }  

        //printf("\n\nclearing buffer: ");
        while ( bcJetson.readable() ) {
            ssize_t resultClear = bcJetson.read(&dumbByte, 1);
            //printf("%d ", dumbByte);
        }
        //printf("\n------CLEARED------\n");
    } 

    else {
        //printf("Err\n");
    }
    return result;
}

int main(){

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();

    /*
    * MOTORS SETUP AND PIDS
    */
    //YAW
    PID yawBeyblade(0.5, 0, 0); //yaw PID is cascading, so there are external position PIDs for yaw control
    yawBeyblade.setOutputCap(30);
    yaw.setSpeedPID(100, 0.48305, 0);
    yaw.setSpeedIntegralCap(5000);
    yaw.setSpeedOutputCap(32000);
    yaw.outputCap = 12000;
    yaw.useAbsEncoder = false;

    int yawVelo = 0;
    #ifdef USE_IMU
    imu.get_angular_position_quat(&imuAngles);
    float yaw_desired_angle = imuAngles.yaw + 180;
    #else
    float yaw_desired_angle = (yaw>>ANGLE) * 360.0 / TICKS_REVOLUTION;
    float yaw_current_angle = (yaw>>ANGLE) * 360.0 / TICKS_REVOLUTION;
    #endif

    //PITCH
    pitch.setPositionPID(23.0458, 0.022697, 756.5322);
    pitch.setPositionOutputCap(32000);
    pitch.pidPosition.feedForward = 0;
    pitch.outputCap = 16000;
    pitch.useAbsEncoder = true;

    float pitch_current_angle = 0;
    float pitch_desired_angle = 0;
    float pitch_phase_angle = 33 / 180.0 * PI; // 5.69 theoretical //wtf is this?
    float pitch_zero_offset_ticks = 6500;
    float K = 0.38; // 0.75 //0.85

    //FLYWHEELS
    LFLYWHEEL_U.setSpeedPID(7.1849, 0.000042634, 0);
    RFLYWHEEL_U.setSpeedPID(7.1849, 0.000042634, 0);
    LFLYWHEEL_D.setSpeedPID(7.1849, 0.000042634, 0);
    RFLYWHEEL_D.setSpeedPID(7.1849, 0.000042634, 0);

    //INDEXER
    indexerL.setSpeedPID(1, 0, 1);
    indexerL.setSpeedIntegralCap(8000);
    indexerR.setSpeedPID(1, 0, 1);
    indexerR.setSpeedIntegralCap(8000);
    //Cascading PID for indexer angle position control. Surely there are better names then "sure"...
    // PID sure(0.5,0,0.4);
    // sure.setOutputCap(4000);
    //Variables for burst fire
    unsigned long timeSure;
    unsigned long prevTimeSure;
    // bool shoot = false;
    // int shootTargetPosition = 36*8190 ;
    // bool shootReady = false;

    //CHASSIS
    Chassis.setYawReference(&yaw, 4608); //the number of ticks of yaw considered to be robot-front
    //Common values for reference are 6500 and 2500
    Chassis.setSpeedFF_Ks(CHASSIS_FF_KICK); //feed forward "kick" for wheels, a constant multiplier of max power in the direcion of movment

    //GENERAL VARIABLES

    //drive and shooting mode
    char drive = 'o'; //default o when using joystick
    char shot = 'o'; //default o when using joystick
    char driveMode = 'j'; //j for joystick, m for mouse/keyboard

    //user button (doesnt work?)
    bool userButton;
    bool prev_userButton;

    //ref variables
    uint16_t chassis_buffer;
    uint16_t chassis_power_limit;
    uint16_t barrel_heat1;
    uint16_t barrel_heat_max1;

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    unsigned long loopTimerCV = loopTimer;
    int refLoop = 0;
    int printLoop = 0;

    ChassisSpeeds cs;

    while(true){
        timeStart = us_ticker_read();

        //CV loop runs every 2ms
        if((timeStart - loopTimerCV) / 1000 > 1) { 
            loopTimerCV = timeStart;
            jetson_send_feedback(); //  __COMENTED OUT LOOLOOKOKOLOOOOKO HERHEHRERHEHRHE
            //basic_bitch_read();
            led2 = !led2;
        }

        if ((timeStart - loopTimer) / 1000 > OUTER_LOOP_DT_MS){
            float elapsedms = (timeStart - loopTimer) / 1000;
            loopTimer = timeStart;
            led = !led;
            refLoop++;
            if (refLoop >= 5){
                led3 = referee.readable();
                refereeThread(&referee);
                refLoop = 0;
                led2 = !led2;

                //POWER LIMIT OVERRIDE INCASE
                if(robot_status.chassis_power_limit < 10){
                    chassis_power_limit = 50;
                }else{
                    chassis_power_limit = robot_status.chassis_power_limit;
                }
                
                Chassis.power_limit = (float)chassis_power_limit;
            }
            Chassis.periodic();
            cs = Chassis.getChassisSpeeds();
            remoteRead();

            int readResult = jetson_read_values(CV_pitch_angle_radians, CV_yaw_angle_radians, CV_shoot);

            //like idk why ig floats are fucked oop
            if ( abs(CV_yaw_angle_radians) > 0.001 ) {
                //printf("you are now zero\n");
                // CV_yaw_angle_radians = 0;
                yaw_desired_angle = CV_yaw_angle_radians / M_PI * 180;
            }
            
            if ( abs(CV_pitch_angle_radians) > 0.001 ) {
                // CV_pitch_angle_radians = 0;
                pitch_desired_angle = CV_pitch_angle_radians / M_PI * 180;
            }
            
            #ifdef USE_IMU
            imu.get_angular_position_quat(&imuAngles);
            #else
            yaw_current_angle = (yaw>>ANGLE) * 360.0 / TICKS_REVOLUTION;
            #endif

            //Keyboard-based drive and shoot mode
            if(remote.keyPressed(Remote::Key::R)){
                drive = 'm';
            }else if(remote.keyPressed(Remote::Key::E)){
                drive = 'u';
            }else if(remote.keyPressed(Remote::Key::Q)){
                drive = 'd';        
            }
            if(remote.keyPressed(Remote::Key::C)){
                shot = 'm';
            }else if(remote.keyPressed(Remote::Key::V)){
                shot = 'u';
            }else if(remote.keyPressed(Remote::Key::B)){
                shot = 'd';        
            }

            //Driving input
            float scalar = 1;
            float jx = remote.leftX() / 660.0 * scalar; // -1 to 1
            float jy = remote.leftY() / 660.0 * scalar; // -1 to 1
            //Pitch, Yaw
            float jpitch = remote.rightY() / 660.0 * scalar; // -1 to 1
            float jyaw = remote.rightX() / 660.0 * scalar; // -1 to 1

            //joystick tolerance
            float tolerance = 0.1; 
            jx = (abs(jx) < tolerance) ? 0 : jx;
            jy = (abs(jy) < tolerance) ? 0 : jy;
            jpitch = (abs(jpitch) < tolerance) ? 0 : jpitch;
            jyaw = (abs(jyaw) < tolerance) ? 0 : jyaw;
            
            //Keyboard Driving
            float mult = 1;
            jx += mult * ((remote.keyPressed(Remote::Key::D) ? 1 : 0) + (remote.keyPressed(Remote::Key::A) ? -1 : 0));
            jy += mult * ((remote.keyPressed(Remote::Key::W) ? 1 : 0) + (remote.keyPressed(Remote::Key::S) ? -1 : 0));
            
            //Bounding the four j variables
            jx = max(-1.0F, min(1.0F, jx));
            jy = max(-1.0F, min(1.0F, jy));
            jpitch = max(-1.0F, min(1.0F, jpitch));
            jyaw = max(-1.0F, min(1.0F, jyaw));


            if (remote.leftSwitch() == Remote::SwitchState::UP){
                indexerL.setPower(8000.0 * remote.leftY() / 660.0);
            } else if (remote.leftSwitch() == Remote::SwitchState::DOWN) {
                indexerR.setPower(8000.0 * remote.leftY() / 660.0);
            } else {
                pitch.setPower(0);
            }

            printLoop ++;
            if (printLoop >= PRINT_FREQUENCY){
                printLoop = 0;
                //printff("Prints:\n");
                //printff("lX:%.1f lY:%.1f rX:%.1f rY:%.1f lS:%d rS:%d\n", remote.leftX(), remote.leftY(), remote.rightX(), remote.rightY(), remote.leftSwitch(), remote.rightSwitch());
                //printff("jx:%.3f jy:%.3f jpitch:%.3f jyaw:%.3f\n", jx, jy, jpitch, jyaw);
                #ifdef USE_IMU
                // printff("yaw_des_v:%d yaw_act_v:%d ", yawVelo, yaw>>VELOCITY);
                // printff("yaw_des:%.3f yaw_act:%.3f\n", yaw_desired_angle, imuAngles.yaw + 180);
                // printff("%.3f, %.3f, %.3f\n", imuAngles.yaw + 180, imuAngles.roll + 180, imuAngles.pitch + 180 );
                #else
                // printff("yaw_des_v:%d yaw_act_v:%d PWR:%d ", yawVelo, yaw>>VELOCITY, yaw>>POWEROUT);
                // printff("yaw_des:%.3f yaw_act:%.3f [%d]\n", yaw_desired_angle, yaw_current_angle, yaw>>ANGLE);
                #endif
                //printff("pitch_des_v:%d yaw_act_v:%d", yawVelo, yaw>>VELOCITY);
                //printff("pitch_des:%.3f pitch_act:%.3f [%d]\n", pitch_desired_angle, pitch_current_angle, pitch>>ANGLE);
                //printff("cX%.1f cY%.1f cOmega%.3f cRPM%.1f\n", cs.vX, cs.vY, cs.vOmega, cs.vOmega * 60 / (2*M_PI) * 4);
                // printff("Chassis: LF:%c RF:%c LB:%c RB:%c Yaw:%c Pitch:%c Flywheel_L:%c Flywheel_R:%c Indexer:%c\n", 
                //     Chassis.getMotor(ChassisSubsystem::LEFT_FRONT).isConnected() ? 'y' : 'n', 
                //     Chassis.getMotor(ChassisSubsystem::RIGHT_FRONT).isConnected() ? 'y' : 'n', 
                //     Chassis.getMotor(ChassisSubsystem::LEFT_BACK).isConnected() ? 'y' : 'n', 
                //     Chassis.getMotor(ChassisSubsystem::RIGHT_BACK).isConnected() ? 'y' : 'n',
                //     yaw.isConnected() ? 'y' : 'n', 
                //     pitch.isConnected() ? 'y' : 'n');

                // printff("Flywheels: LU:%c LD:%c RU:%c RD:%c IndexerL:%c IndexerR:%c\n", 
                //     LFLYWHEEL_U.isConnected() ? 'y' : 'n', 
                //     LFLYWHEEL_D.isConnected() ? 'y' : 'n', 
                //     RFLYWHEEL_U.isConnected() ? 'y' : 'n', 
                //     RFLYWHEEL_D.isConnected() ? 'y' : 'n', 
                //     indexerL.isConnected() ? 'y' : 'n',
                //     indexerR.isConnected() ? 'y' : 'n');
                //printff("%d %d %d\n", Chassis.getMotor(ChassisSubsystem::LEFT_FRONT)>>VELOCITY,  Chassis.getMotor(ChassisSubsystem::LEFT_FRONT)>>VALUE, Chassis.getMotor(ChassisSubsystem::LEFT_FRONT).getMode());
                #ifdef USE_IMU
                //printff("IMU %.3f %.3f %.3f\n",imuAngles.yaw, imuAngles.pitch, imuAngles.roll);
                #endif
                // printff("pwr:%u max:%d heat:%d\n", chassis_buffer, robot_status.chassis_power_limit, power_heat_data.shooter_17mm_1_barrel_heat);
                //printff("ID:%d LVL:%d HP:%d MAX_HP:%d\n", robot_status.robot_id, robot_status.robot_level, robot_status.current_HP, robot_status.maximum_HP);
                //printff("elap:%.5fms\n", elapsedms);
            }

            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}