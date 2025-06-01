//mbed-tools sterm -b 115200


#include "main.h"
#include "subsystems/ChassisSubsystem.h"
#include <algorithm>
#include <cstdint>  // For uint64_t
#include <iostream>
#include <chrono>


#define PI 3.14159265
#define MAGICBYTE 0xEE
#define DEBUG 0

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
    int packetFlag = MAGICBYTE;
    int i;
    char clear;
    ssize_t fillArrayCheck = 10;
    //char *jetsonValuePtr = jetson_value;        //ptr pointing to memory address of jetson values
    unsigned int jetsonIndexShift = 0;            //relates to index of last byte in jetsonValues
    int numBytes = 0;
    uint8_t checkSum;                              //last bit in results array              

    if ( bcJetson.readable() ) {
        while ( (fillArrayCheck >= 10) && (jetsonIndexShift < 200) ) {
            fillArrayCheck = bcJetson.read(jetson_value+numBytes, 10); //keep adding 10 bytes throughout array until buffer empty
            numBytes += fillArrayCheck;
        }

        if (DEBUG) { 
            for ( i = 0; i< numBytes ; ++i) {
                printf("%d ", jetson_value[i]);
            }
            printf("\nbytes: %d\n", numBytes);
        }

        //starting at the very last index of jetson_values
        //we will be checking 9 bytes before the byte i is at; exclude magic byte in checksum
        //if we meet the match conditions, decode values, clear buffer, return 1
        //if no match found, print bad
        for( i = numBytes - 1 ; i >= 10 ; --i) {
            if (DEBUG) { 
                for (int j = i - 10; j <= i; ++j) {
                    printf("%d ",jetson_value[j]);
                }
                printf("\n");
                printf("%d %d %d\n", calculateLRC(&jetson_value[i - 9], 9), jetson_value[i], jetson_value[i-10]);
             }

            //calculating checksum w/ header bytes
            if ( (calculateLRC(&jetson_value[i - 9], 9) == jetson_value[i]) && (jetson_value[i] != 0) && (jetson_value[i-10] == packetFlag) ){
                printf("match\n");
                
                //debug
                if (DEBUG) {
                    for (int j = jetsonIndexShift - 10 ; j < jetsonIndexShift ; ++j) {
                        printf("%d ", jetson_value[j]);
                    } 
                    printf("\n");
                }

                decode_toSTM32(&jetson_value[i-9], pitch_move, yaw_move, shoot_switch, checkSum);
                return fillArrayCheck;
            }
        }
        printf("\nno match\n");
        return 0;
        
    } else {
        printf("\nErr\n");
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

DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);
DigitalOut ledbuiltin(LED1);

DJIMotor testMot(1, CANHandler::CANBUS_1, GM6020, "testbench_motor");

#define IMPULSE_DT 100
#define IMPULSE_STRENGTH 16383

int main(){

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    // unsigned long loopTimer2 = us_ticker_read();
    // int powerCnt = 0;
    int refLoop = 0;

    testMot.setSpeedPID(250,0,0);
    PID yawBeyblade(0.04,0,4);
    yawBeyblade.setOutputCap(60);
    int yawVelo = 0;
    float yaw_desired_angle = (testMot>>ANGLE) * 360.0 / TICKS_REVOLUTION;
    float yaw_current_angle = (testMot>>ANGLE) * 360.0 / TICKS_REVOLUTION;
    unsigned long timeSure;
    unsigned long prevTimeSure;

    bool prevL = false;
    bool switL = false;

    int motorSpeed = 0;

    while(true){
        timeStart = us_ticker_read();

        // if ((timeStart - loopTimer2) / 1000 > 3000 && (powerCnt < 16000)){
        //     loopTimer2 = timeStart;
        //     testMot.setPower(powerCnt);
        //     powerCnt += 1000;
        // }

        if ((timeStart - loopTimer) / 1000 > 15){
            
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
            prevL = switL;
            remoteRead();
            switL = (remote.leftSwitch() == Remote::SwitchState::UP);

            // testMot.setPower(remote.leftX() * 3);
            // if(!prevL && switL){
            //     motorSpeed += 10;
            // }
            //Regular Yaw Code
            yaw_desired_angle = remote.leftX() * 5;
            prevTimeSure = timeSure;
            timeSure = us_ticker_read();
            motorSpeed = yawBeyblade.calculatePeriodic(DJIMotor::s_calculateDeltaPhase(yaw_desired_angle, testMot>>ANGLE, 8192), timeSure - prevTimeSure);
            
            int dir = 0;
            if(motorSpeed > 0){
                dir = 1;
            }else if(motorSpeed < 0){
                dir = -1;
            }
            testMot.pidSpeed.feedForward = dir * ((15.4 + abs(motorSpeed)) / 0.0083);
            if(remote.rightSwitch() == Remote::SwitchState::UP) {
                testMot.setSpeed(motorSpeed);
            } else {
                testMot.setPower(0);
            }
            printff("%d\t%d\t%d\t%d\t%d\n", yaw_desired_angle, testMot>>ANGLE, motorSpeed, testMot>>VELOCITY, testMot>>POWEROUT);

            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}