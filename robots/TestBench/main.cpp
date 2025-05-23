//mbed-tools sterm -b 115200


#include "main.h"
#include "subsystems/ChassisSubsystem.h"
#include <algorithm>
#include <cstdint>  // For uint64_t
#include <iostream>
#include <chrono>


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

 //return 1 if successful match, 0 if no match but buffer available, -1 for unreadable
ssize_t jetson_read_values(float &pitch_move, float & yaw_move, char &shoot_switch, int &packetFlag) {
    bcJetson.set_blocking(false);
    int i;
    char clear;
    ssize_t fillArrayCheck = 10;
    //char *jetsonValuePtr = jetson_value;        //ptr pointing to memory address of jetson values
    unsigned int jetsonIndexShift = 0;            //relates to index of last byte in jetsonValues
    int numBytes = 0;
    uint8_t checkSum;                              //last bit in results array              

    if ( bcJetson.readable() ) {

        // printf("\nread\n");
        // while ( (fillArrayCheck >= 10) && (jetsonIndexShift < 200) ) {
        //     fillArrayCheck = bcJetson.read(jetsonValuePtr+jetsonIndexShift, 10); //keep adding 10 bytes throughout array until buffer empty
        //     jetsonIndexShift += 10;
        // }
        // jetsonIndexShift -= 10;

        while ( (fillArrayCheck >= 10) && (jetsonIndexShift < 200) ) {
            fillArrayCheck = bcJetson.read(jetson_value+numBytes, 10); //keep adding 10 bytes throughout array until buffer empty
            numBytes += fillArrayCheck;
        }
        printf("\nbytes: %d\n", numBytes);

        //starting at the very last index of jetson_values
        //we will be checking 9 bytes before the byte i is at
        //if we meet the match conditions, decode values, clear buffer, return 1
        //if no match found, print bad
        for( int i = numBytes - 1 ; i >= 9 ; --i) {
            if (debug) { printf("%d %d %d\n", calculateLRC(&jetson_value[i - 10], 9), jetson_value[i], jetson_value[i-10]); }

            if ( (calculateLRC(&jetson_value[i - 10], 9) == jetson_value[i]) && (jetson_value[i] != 0) && (jetson_value[i-10] == packetFlag) ){
                printf("match\n");
                
                //debug
                if (debug) {
                    for (int j = jetsonIndexShift - 10 ; j < jetsonIndexShift ; ++j) {
                        printf("%d ", jetson_value[j]);
                    } 
                    printf("\n");
                }

                decode_toSTM32(&jetson_value[i-9], pitch_move, yaw_move, shoot_switch, checkSum);


                while (bcJetson.readable() ) {
                    fillArrayCheck = bcJetson.read(&clear, 1);
                //printf("c");
                }
                return fillArrayCheck;
            }
        }
        return 0;
        //we have moved 10 bytes back to account for extra last increment
        //calculateLRC works from the start of address to the next x bytes
        //checks if the checksum of the last packet is consistent
        //goal: decrement jetsonindexshift until it reaches then end of a good packet
        //ptr+jetsonindex-10 starts us at the beginning of a packet we're checking
        // while ( calculateLRC(jetsonValuePtr+jetsonIndexShift-10, 9) != jetson_value[jetsonIndexShift - 1] ) {
        //     //printf("check: %d = %d? - strt: %d - ptrStrt:%d - ", calculateLRC(jetsonValuePtr+jetsonIndexShift-10, 9), jetson_value[jetsonIndexShift-1], jetson_value[jetsonIndexShift-10],*(jetsonValuePtr+jetsonIndexShift-10));
        //     for ( i = jetsonIndexShift - 10 ; i < jetsonIndexShift ; ++i) {
        //         printf("%d ", jetson_value[i]);
        //     }    
        //     --jetsonIndexShift;
        //     printf(" bad\n");
        // }

        //--jetsonIndexShift; //while loop does this when it checks, however it will not be in the outcome

        // printf("\nmatched packet: ");
        // for ( i = jetsonIndexShift - 10 ; i < jetsonIndexShift ; ++i) {
        //     printf("%d ", jetson_value[i]);
        // }   

        // checkSum = jetson_value[jetsonIndexShift];
        // decode_toSTM32(jetsonValuePtr+jetsonIndexShift-10, pitch_move, yaw_move, shoot_switch, checkSum);
        // jetsonIndexShift = 0;

        // while (bcJetson.readable() ) {
        //     fillArrayCheck = bcJetson.read(&clear, 1);
        //     //printf("c");
        // }
        // jetsonIndexShift = 0;
    } else {
        printf("\nErr\n");
    }
    return fillArrayCheck;
}
        

//Send yourself packets through buffer
void self_sending_data() {
    static float test_yaw = 0;
    static float test_pitch = 0;
    static bool direction = 0;
    static char test_packet[10] = {0};
    static char shoot_on = 0;
    static int loop_count = 0;
    uint8_t check_sum;

    if (loop_count == 100) {
        if (shoot_on == 0) {
            ++shoot_on;
        }
        else { shoot_on = 1; }
        loop_count = 0;
    }

    // //incrementing pitch and Yaw myself Code--------------
    if ( direction == 0) {
        test_yaw += 0.01;
        test_pitch += 0.001; //don't need to worry about bounds, handled below

        if (test_yaw >= 1){
            direction = 1;
        }
    }
    else {
        test_yaw -= 0.01;
        test_pitch -= 0.001;
        if (test_yaw <= -1){
            direction = 0;
        } 
    }

    //Forming packet
    memcpy(test_packet, &test_pitch, sizeof(float));
    memcpy(test_packet + 4, &test_yaw, sizeof(float));
    memcpy(test_packet + 8, &shoot_on, sizeof(char)); // shooting indicator

    // //checksum
    check_sum = calculateLRC(test_packet,9);
    memcpy(test_packet + 9, &check_sum, sizeof(char)); // shooting indicator

    //fill buffer
    bcJetson.write( &test_packet, 10);
    bcJetson.sync();
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
    //jetson_send_feedback();
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

                    printf("S: ");
                    for (int i = 0 ; i < 11 ; ++i) {
                        test_packet[i] = i;
                        printf("%d ", test_packet[i]);
                    }
                    printf("\n");
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
                    check_sum = calculateLRC(test_packet, 10);
                    memcpy(test_packet + 10, &check_sum, sizeof(char)); // shooting indicator

                    printf("S: ");
                    for (int i = 0 ; i < 11 ; ++i) {
                        printf("%d ", test_packet[i]);
                    }
                    printf("\n");

                    //fill buffer
                    bcJetson.write( &test_packet, 10);
                    bcJetson.sync();
                    //basic_bitch_read();
                }
            printf("\nloop: %d\n",printLoop);
            
            ++printLoop;
            ++shoot_count;
            byteCount += 11;;
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

