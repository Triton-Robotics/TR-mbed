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
// BNO055 imu(i2c, IMU_RESET, MODE_IMU);

DJIMotor yaw(6, CANHandler::CANBUS_1, GIMBLY, "yaw");
DJIMotor pitch(5, CANHandler::CANBUS_2, GIMBLY);
DJIMotor indexerL(5, CANHandler::CANBUS_2, C610);
DJIMotor indexerR(6, CANHandler::CANBUS_2, C610);

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
char jetson_value[30] = {0};

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
void jetson_read_values(float &pitch_move, float & yaw_move, char &shoot_switch) {
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
        printf("Err\n");
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

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    unsigned long loopTimerCV = us_ticker_read();
    //printf("Current timestamp (us): %lu\n", timeStart);  // Print the timestamp
    int refLoop = 0;
    //jetson_send_feedback();
    int counter = 1;

    /* Pitch Position PID*/
    // These values are for new sentry 2025
    pitch.setPositionPID(20.3544, 0.020221, 1078.4383); // think about D-cap and potentially raising FF. if the setpoint is always higher than actual,
    pitch.setPositionIntegralCap(3000);
    // then could try to up FF to get there
    // pitch.setSpeedPID(0,0,0);

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

    yaw.setSpeedPID(10, 0, 0);

    PID yawBeyblade(1.5, 0, 550);
    PID yawNonBeyblade(0.15, 0, 550);
    yaw.setSpeedIntegralCap(0);
    yaw.useAbsEncoder = false;

    yaw.setSpeedIntegralCap(0);
    yaw.useAbsEncoder = false;

    yaw.setSpeedOutputCap(24000);

    int ref_yaw;

    // int yawSetPoint = imuAngles.yaw;
    double rotationalPower = 0;

    unsigned long yawTime = us_ticker_read();

    //Indexer 
    indexerR.setSpeedPID(1, 0, 1);
    indexerR.setSpeedIntegralCap(8000);

    PID sure(0.5,0,0.4);
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

    int des_pitch_in_ticks = 0;
    int pitch_in_ticks = 0;

    //yaw
    float yaw_in_deg;
    int yaw_in_ticks;
    float yaw_angle;
    float curr_yaw_angle;
    int yawVelo = 0;



    //shooting
    char shoot_toggle;

    //debug
    float pitch_in_deg = 0;
    int ticks_to_motor = 0;
    float currPitch = 0;
    float desiredPosition = 0;
    float diffRealExpectedPitch = 0;
    


    //Buffer stuff
    char jetsonByte;
    int numBytes = 0;
    int totalBytes = 0;
    char testPacketTx[10] = {0}, testChecksumTx;       //Tx self test
    
    //Rx
    char testPacketRx[10] = {0}, test_shoot_Rx, testChecksumRx;         //Rx self test
    float test_pitch_Rx, test_yaw_Rx;

        //buffer packet testing  -test_... is for writing data to myself
        float yaw_test_radians = 0;
        bool test_direction = 0;

        float pitch_test_radians = 0;
        char test_shoot = 0;

        //shooting mechanics
        bool shoot = 0;
        int shootTargetPosition = 36*8190 ;
        bool shootReady = false;
        int debugShooting;

    

    //PRINTLOOP
    int printLoop = 0;

    while(true){
        pitch_ANGLE = 0.0;
        timeStart = us_ticker_read();

        //CV loop runs every 2ms
        if((timeStart - loopTimerCV) / 1000 > 2) { 
            loopTimerCV = timeStart;
            //jetson_send_feedback();   __COMENTED OUT LOOLOOKOKOLOOOOKO HERHEHRERHEHRHE
            //basic_bitch_read();
            led2 = !led2;
        }

        if ((timeStart - loopTimer) / 1000 > 15){
            led = !led;

            //int sizePacket = bcJetson.available();
            refLoop++;
            // remoteRead();
            remoteRead();
            // Chassis.periodic();

            if (refLoop >= 5){
                refereeThread(&referee);

                refLoop = 0;
            }

            //-----------Sending data to myself-------------------------
            // uint8_t data[100];


            // for (uint8_t i = 0; i < 100 ; i++) {
            //     data[i] = i;
            //     ssize_t bytesBuffer = bcJetson.write(&data[i], 1);
            //     printf("number: %d bytes: %d \n", data[i], bytesBuffer);
            // }
            // ThisThread::sleep_for(100ms);

            // while (bcJetson.readable()) {
            //     ssize_t result = bcJetson.read(&jetsonByte, 1);
            //     printf("%d ", jetsonByte);
            //     ++numBytes;
            //     ++totalBytes;
            // }
            // printf("\n num: %d total: %d result:  \n \n", numBytes, totalBytes);
            // numBytes = 0;
            //-----------sending data to myself w/ buffer-------------------------



            //incrementing pitch and Yaw myself Code--------------
            if ( test_direction == 0) {
                yaw_test_radians += 0.01;
                pitch_test_radians += 0.001; //don't need to worry about bounds, handled below

                if (yaw_test_radians >= 1){
                    test_direction = 1;
                }
            }
            else {
                yaw_test_radians -= 0.01;
                pitch_test_radians -= 0.001;
                if (yaw_test_radians <= -1){
                    test_direction = 0;
                } 
            }

            //Forming packet
            memcpy(testPacketTx, &pitch_test_radians, sizeof(float));
            memcpy(testPacketTx + 4, &yaw_test_radians, sizeof(float));
            memcpy(testPacketTx + 8, &test_shoot, sizeof(char)); // shooting indicator

            //checksum
            testChecksumTx = calculateLRC(testPacketTx,9);
            memcpy(testPacketTx + 9, &testChecksumTx, sizeof(char)); // shooting indicator
            

            //printff("\n------SENDING-----\n");
            // for (int i = 0 ; i < 10 ; ++i ) {
            //     printf("%d ", testPacketTx[i]);
            // }
            //printff("Tx pitch: %.3f yaw: %.3f shoot: %d Check: %d\n", pitch_test_radians, yaw_test_radians, test_shoot, testChecksumTx);


            // //CLEARING BUFFER
            // printf("\n\nclearing buffer: ");
            // while ( bcJetson.readable() ) {
            //     ssize_t resultClear = bcJetson.read(&jetsonByte, 1);
            //     printf("%d ", jetsonByte);
            // }
            // printf("\n------CLEARED------\n");

            //fill buffer with packet
            bcJetson.write( &testPacketTx, 10);
            bcJetson.sync();


            //RECIEVING----
            //printff("\n------RECIEVING-----\n");
            // ssize_t resultClear = bcJetson.read(&testPacketRx, 10);
            // for (int i = 0 ; i < 10 ; ++i ) {
            //     printf("%d ", testPacketRx[i]);
            // }
            // printf("\n");


            // if (resultClear == 10) {
            //     memcpy(&test_pitch_Rx, testPacketRx, sizeof(float));   //memcpy( destination, source, size)
            //     memcpy(&test_yaw_Rx, testPacketRx + 4, sizeof(float));
            //     memcpy(&test_shoot_Rx, testPacketRx + 8, sizeof(char));
            //     memcpy(&testChecksumRx, testPacketRx + 9, sizeof(char));
            
            //     printf("Rx Pitch: %.3f Yaw: %.3f Shoot: %d Check: %d\nFIN\n\n", test_pitch_Rx, test_yaw_Rx, test_shoot_Rx, testChecksumRx);
            // } else {
            //     printf("Read failed or incomplete. Bytes read: %d\n", (int)resultClear);
            // }


            jetson_read_values(pitch_ANGLE, yaw_angle, shoot_toggle);
            //printf("Rx Pitch: %.3f Yaw: %.3f Shoot: %d\n\n\n", pitch_ANGLE, yaw_angle, shoot_toggle);

            if (pitch_ANGLE != 0) {
                pitch_in_ticks = ChassisSubsystem::radiansToTicks(pitch_ANGLE);
                //printf("%d ticks here\n", pitch_in_ticks);
                /* Catch pitch beyond thresholds*/
                if (pitch_in_ticks <= LOWERBOUND_TICKS) {
                    pitch_in_ticks = LOWERBOUND_TICKS;
                }
                else if (pitch_in_ticks >= UPPERBOUND_TICKS) {
                    pitch_in_ticks = UPPERBOUND_TICKS;
                }

                pitch_in_ticks *= GEAR_RATIO; //gear ratio;
            }

            //Regular Yaw Code
            prevTimeSure = timeSure;
            timeSure = us_ticker_read();

            yaw_in_ticks = ChassisSubsystem::radiansToTicks(yaw_angle);
            yaw.setPosition(yaw_in_ticks);

            //yaw_in_ticks = ChassisSubsystem::radiansToTicks(yaw_test_radians); //same as above but for self buffer
            //curr_yaw_angle = yaw.getData(ANGLE);
            //printf("desPitch currPitch desYaw currYaw\n"); //comment out for arduino data  
            //printf("%d %d %d %d\n", pitch_in_ticks, pitch.getData(ANGLE), yaw_in_ticks, yaw.getData(ANGLE));

            

            //----------------------------CV SHOOTING------------------------

            if ( (remote.leftSwitch() == Remote::SwitchState::UP) )
            {
                //              indexerL.setSpeed(-2000);
                setFlyWheelPower(8000);
                //indexerL.setSpeed(2000);
                if (shoot_toggle != 0) {
                    indexerR.setSpeed(30*36);
                }
                else { indexerR.setSpeed(0); }
            }
            else if (remote.leftSwitch() == Remote::SwitchState::MID)
            {
                //              indexerL.setSpeed(-2000);
                setFlyWheelPower(8000);
                indexerL.setPower(0);
                indexerR.setPower(0);
                // printff("indexer.s%d\n")
            }
            //FLYWHEELS
            if (remote.leftSwitch() != Remote::SwitchState::DOWN &&
                remote.leftSwitch() != Remote::SwitchState::UNKNOWN){
                RTOPFLYWHEEL.setSpeed(-7000);
                LTOPFLYWHEEL.setSpeed(-7000);
                RBOTTOMFLYWHEEL.setSpeed(7000);
                LBOTTOMFLYWHEEL.setSpeed(7000);
            } else{
                // left SwitchState set to up/mid/unknown
                RTOPFLYWHEEL.setSpeed(0);
                LTOPFLYWHEEL.setSpeed(0);
                RBOTTOMFLYWHEEL.setSpeed(0);
                LBOTTOMFLYWHEEL.setSpeed(0);
            }

            // //INDEXER CODE
            // if ( (remote.leftSwitch() == Remote::SwitchState::UP || remote.getMouseL() ) || (shoot_toggle != 0) ){
            //     if (shootReady){
            //         shootReady = false;
            //         shootTargetPosition = 8192 * 12 + (indexerR>>MULTITURNANGLE);

            //         //shoot limit
            //         // if(robot_status.shooter_barrel_heat_limit < 10 || power_heat_data.shooter_17mm_1_barrel_heat < robot_status.shooter_barrel_heat_limit - 40) {
            //         // }

            //         shoot = true;
            //         printff("FIRE\n");
            //     }
            // } else {
            //     //SwitchState state set to mid/down/unknown
            //     //shootReady = true;
            // }

            // // burst fire, turn the indexer to shoot 3-5 balls a time and stop indexer
            // // only shoot when left switch changes from down/unknown/mid to up
            // // if left switch remains at up state, indexer stops after 3-5 balls
            // if (shoot){
            //     if ( indexerR>>MULTITURNANGLE >= shootTargetPosition ) {
            //         indexerR.setSpeed(0); 
            //         shoot = false;
            //         shootReady = true;
            //     } else {
            //         timeSure = us_ticker_read();
            //         debugShooting = sure.calculatePeriodic((float)(shootTargetPosition - (indexerR>>MULTITURNANGLE)), timeSure - prevTimeSure);
            //         //indexerR.setSpeed(debugShooting);
            //         indexerR.setSpeed(30 * 36);

            //         prevTimeSure = timeSure;
            //     }
            // } else {
            //     indexerR.setSpeed(0);
            // }


            //FLYWHEELS
            if (remote.leftSwitch() != Remote::SwitchState::DOWN &&
                remote.leftSwitch() != Remote::SwitchState::UNKNOWN){
                RTOPFLYWHEEL.setSpeed(-7000);
                LTOPFLYWHEEL.setSpeed(-7000);
                RBOTTOMFLYWHEEL.setSpeed(7000);
                LBOTTOMFLYWHEEL.setSpeed(7000);
            } else{
                // left SwitchState set to up/mid/unknown
                RTOPFLYWHEEL.setSpeed(0);
                LTOPFLYWHEEL.setSpeed(0);
                RBOTTOMFLYWHEEL.setSpeed(0);
                LBOTTOMFLYWHEEL.setSpeed(0);
            }

            //------------------CV SHOOTING---------------------------

            ++printLoop;
            if( printLoop >= 100){   //use for slower printing
                printLoop = 0;
                if (test_shoot == 0) { ++test_shoot; }
                else { test_shoot = 0; }
                // printf("Target: %d | Current: %d | Error: %d | Output: %d | dt: %lu\n",
                //     shootTargetPosition,
                //     indexerR >> MULTITURNANGLE,
                //     shootTargetPosition - (indexerR >> MULTITURNANGLE),
                //     debugShooting,
                //     timeSure - prevTimeSure);
                printff("%d %d %d %d\n", test_shoot, shootReady, shootTargetPosition, indexerR>>MULTITURNANGLE);
            }

            // pitch_in_ticks is relative to level = 0 ticks. PITCH_LEVEL_TICKS - pitch_in_ticks = abs position in ticks
            pitch.setPosition(PITCH_LEVEL_TICKS - pitch_in_ticks);
            loopTimer = timeStart;
            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        // jetson_send_feedback();

        unsigned long timeEnd = us_ticker_read();
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

