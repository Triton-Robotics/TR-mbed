//mbed-tools sterm -b 115200


#include "main.h"
#include "subsystems/ChassisSubsystem.h"
#include <algorithm>
#include <cstdint>  // For uint64_t
#include <iostream>
#include <chrono>


#define UPPERBOUND_DEG 15.0 
#define LOWERBOUND_DEG -26.0 // Bound of how low turret can point in degrees

#define UPPERBOUND_TICKS (UPPERBOUND_DEG/360.0) * 8192 // 1137 ticks above/CCW to 4250, ie 4250-1137 = 3112 absolute position in ticks
#define LOWERBOUND_TICKS (LOWERBOUND_DEG/360.0) * 8192 // 682 ticks below/CW to 4250. ie 4932 abs pos in ticks

#define GEAR_RATIO 2
#define PITCH_LEVEL_TICKS 6445 // The tick value of level turret pitch. Also used for initial offset

#define PID_POS 1 // Use pitch speed PID
#define PID_SPD 0 // Use pitch position PID


DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);

static BufferedSerial bcJetson(PC_12, PD_2, 115200);  //JETSON PORT

I2C i2c(I2C_SDA, I2C_SCL);
BNO055 imu(i2c, IMU_RESET, MODE_IMU);

DJIMotor yaw(6, CANHandler::CANBUS_1, GIMBLY, "yaw");
DJIMotor pitch(5, CANHandler::CANBUS_2, GIMBLY);
DJIMotor indexerL(5, CANHandler::CANBUS_2, C610);
DJIMotor indexerR(6, CANHandler::CANBUS_2, C610);

ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2286); // radius is 9 in

DJIMotor RTOPFLYWHEEL(2, CANHandler::CANBUS_2, M3508_FLYWHEEL);
DJIMotor LTOPFLYWHEEL(3, CANHandler::CANBUS_2, M3508_FLYWHEEL);
DJIMotor RBOTTOMFLYWHEEL(1, CANHandler::CANBUS_2, M3508_FLYWHEEL);
DJIMotor LBOTTOMFLYWHEEL(4, CANHandler::CANBUS_2, M3508_FLYWHEEL);

char nucleo_value[30] = {0};
char jetson_value[30] = {0};

struct fromJetson{
    float pitch_angle;
    float yaw_angle;
    uint8_t checksum;
};

float xRotated, yRotated;

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
    RTOPFLYWHEEL.setPower(-speed);// changes made
    RBOTTOMFLYWHEEL.setPower(speed);
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
    char rotate_x_char[sizeof(float)];
    char rotate_y_char[sizeof(float)];
    getBytesFromFloat(rotate_x_char, xRotated);
    getBytesFromFloat(rotate_y_char, yRotated);

    //GET yaw and pitch data
    float yaw_angle = ChassisSubsystem::ticksToRadians(yaw.getData(ANGLE)); //Ticks
    float yaw_velocity = yaw.getData(VELOCITY)/60.0; //RPM

    float pitch_angle = ChassisSubsystem::ticksToRadians((PITCH_LEVEL_TICKS - pitch.getData(ANGLE)) / GEAR_RATIO);
    float pitch_velocity = pitch.getData(VELOCITY)/60.0;

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


    //get lrc
    uint8_t lrc = calculateLRC(nucleo_value, 24);
    char lrc_char = static_cast<uint8_t>(lrc);
  
    nucleo_value[24] = lrc_char;
  

    bcJetson.set_blocking(false);
    bcJetson.write(nucleo_value, 30);
    
    
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

    if (result != -EAGAIN) { // If buffer not empty, decode data. Else do nothing
        uint8_t checkSum = jetson_value[9];
        uint8_t theoryCheck = calculateLRC(jetson_value,9);
        if(checkSum == theoryCheck){
            decode_toSTM32(jetson_value, pitch_move, yaw_move, shoot_switch, checkSum);
        }
        else{
            led3 = !led3;
        }  

        while ( bcJetson.readable() ) {
            ssize_t resultClear = bcJetson.read(&dumbByte, 1);
        }
    } 
    return result;
}



int main(){
    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    unsigned long loopTimerCV = us_ticker_read();
    
    int refLoop = 0;
    int counter = 1;

    /* Pitch Position PID*/
    // These values are for new sentry 2025
    pitch.setPositionPID(20.3544, 0.020221, 278.4383); 
    pitch.setPositionIntegralCap(3000);
    pitch.pidPosition.feedForward = 0;
    pitch.outputCap = 32760;
    pitch.useAbsEncoder = true;
   

    PID yawBeyblade(0.5, 0, 0);
    yaw.setSpeedPID(5.5, 0, 0);
    yaw.setSpeedIntegralCap(8000);
    yaw.setSpeedOutputCap(32000);
    yaw.outputCap = 16000;
    yaw.useAbsEncoder = false;


    //Indexer 
    indexerR.setSpeedPID(1, 0, 1);
    indexerR.setSpeedIntegralCap(8000);
    indexerL.setSpeedPID(1,0,1);
    indexerL.setSpeedIntegralCap(8000);

    unsigned long timeSure;
    unsigned long prevTimeSure;
    
    float pitch_ANGLE = 0.0;
    int pitch_in_ticks = 0;

    int yaw_in_ticks;
    float yaw_CV_angle;
    int yawVelo = 0;

    //shooting
    char shoot_toggle;

    //shooting mechanics
    bool shoot = 0;
    int shootTargetPosition = 36*8190 ;
    bool shootReady = false;
    int debugShooting;

    

    //PRINTLOOP
    int printLoop = 0;

    ChassisSpeeds cs;

    int dticks = 0;

    while(true){
        timeStart = us_ticker_read();

        //CV loop runs every 2ms
        if((timeStart - loopTimerCV) / 1000 > 2) { 
            loopTimerCV = timeStart;
            jetson_send_feedback(); 
            led2 = !led2;
        }

        if ((timeStart - loopTimer) / 1000 > 15){
            led = !led;

            refLoop++;
            remoteRead();
            Chassis.periodic();
            cs = Chassis.getChassisSpeeds();

            if (refLoop >= 5){
                refereeThread(&referee);
                refLoop = 0;
            }
          
            int readResult = jetson_read_values(pitch_ANGLE, yaw_CV_angle, shoot_toggle);

            //like idk why ig floats are fucked oop
            if ( abs(yaw_CV_angle) <= 0.001 ) {
                yaw_CV_angle = 0;
            }
            if ( abs(pitch_ANGLE) <= 0.001 ) {
                yaw_CV_angle = 0;
            }

            // --- PITCH ---
            if (readResult != 0) {
                pitch_in_ticks = ChassisSubsystem::radiansToTicks(pitch_ANGLE);
                /* Catch pitch beyond thresholds*/
                if (pitch_in_ticks <= LOWERBOUND_TICKS) {
                    pitch_in_ticks = LOWERBOUND_TICKS;
                }
                else if (pitch_in_ticks >= UPPERBOUND_TICKS) {
                    pitch_in_ticks = UPPERBOUND_TICKS;
                }

                pitch_in_ticks *= GEAR_RATIO; //gear ratio;
            }
            // pitch_in_ticks is relative to level = 0 ticks. PITCH_LEVEL_TICKS - pitch_in_ticks = abs position in ticks
            pitch.setPosition(PITCH_LEVEL_TICKS - pitch_in_ticks);



            // --- YAW ----
            yaw_CV_angle = floatmod(yaw_CV_angle, 2 * M_PI);
            yaw_in_ticks = ChassisSubsystem::radiansToTicks(yaw_CV_angle);

            float chassis_rotation_radps = cs.vOmega;
            int chassis_rotation_rpm = chassis_rotation_radps * 60 / (2*M_PI) * 4; //I added this 4 but I don't know why.
            
            prevTimeSure = timeSure;
            timeSure = us_ticker_read();
            dticks = DJIMotor::s_calculateDeltaPhase(yaw_in_ticks, yaw>>ANGLE, 8192);
            yawVelo = yawBeyblade.calculatePeriodic(dticks, timeSure - prevTimeSure);

            int dir = 0;
            if(yawVelo > 0){
                dir = 1;
            }else if(yawVelo < 0){
                dir = -1;
            }
            yaw.pidSpeed.feedForward = dir * (874 + (73.7 * abs(yawVelo)) + (0.0948 * yawVelo * yawVelo));
            yaw.setSpeed(yawVelo);
            

            //----------------------------CV SHOOTING------------------------
            if ( (remote.leftSwitch() == Remote::SwitchState::UP) )
            {
                setFlyWheelPower(8000);
                if (shoot_toggle != 0) {
                    indexerR.setSpeed(30*120);
                    indexerL.setSpeed(-30*120);
                }
                else { indexerR.setSpeed(0); }
            }
            else if (remote.leftSwitch() == Remote::SwitchState::MID)
            {
                setFlyWheelPower(8000);
                indexerL.setPower(0);
                indexerR.setPower(0);
            } else {
                RTOPFLYWHEEL.setSpeed(0);
                LTOPFLYWHEEL.setSpeed(0);
                RBOTTOMFLYWHEEL.setSpeed(0);
                LBOTTOMFLYWHEEL.setSpeed(0);
            }

            //------------------CV SHOOTING---------------------------

            ++printLoop;
            if( printLoop >= 15){   //use for slower printing
                printLoop = 0;
                // printff("Y: %d %d ,%d [%d] P: %d %d ,%d\n", yaw_in_ticks, yaw>>ANGLE, yaw>>POWEROUT, dticks, pitch_in_ticks, pitch>>ANGLE, pitch>>POWEROUT);
            }

            
            loopTimer = timeStart;
            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        unsigned long timeEnd = us_ticker_read();
    }
}

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

