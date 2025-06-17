#include "main.h"
#include "subsystems/ChassisSubsystem.h"

DigitalOut led(L25);
DigitalOut led2(L26);
DigitalOut led3(L27);
DigitalOut ledbuiltin(LED1);

//CONSTANTS
constexpr float LOWERBOUND = -35.0;
constexpr float UPPERBOUND = 40.0;

constexpr float BEYBLADE_OMEGA = 1.0;

// constexpr float JOYSTICK_SENSITIVITY_YAW = 1.0/90;
// constexpr float JOYSTICK_SENSITIVITY_PITCH = 1.0/150;
// constexpr float MOUSE_SENSITIVITY_YAW = 1.0/5;
// constexpr float MOUSE_SENSITIVITY_PITCH = 1.0/5;

//DEGREES PER SECOND AT MAX
constexpr float JOYSTICK_SENSITIVITY_YAW_DPS = 180.0; 
constexpr float JOYSTICK_SENSITIVITY_PITCH_DPS = 180.0;

constexpr int OUTER_LOOP_DT_MS = 15;

constexpr int PRINT_FREQUENCY = 20; //the higher the number, the less often

constexpr float CHASSIS_FF_KICK = 0.065;

constexpr float pitch_zero_offset_ticks = 1500;

#define READ_DEBUG 0
#define MAGICBYTE 0xEE
#define USE_IMU

//CHASSIS DEFINING
I2C i2c(I2C_SDA, I2C_SCL);
BNO055 imu(i2c, IMU_RESET, MODE_IMU);
ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2286); // radius is 9 in
DJIMotor yaw(4, CANHandler::CANBUS_1, GIMBLY,"Yeah");
DJIMotor pitch(7, CANHandler::CANBUS_2, GIMBLY,"Peach"); // right

DJIMotor indexer(7, CANHandler::CANBUS_2, C610,"Indexer");
DJIMotor RFLYWHEEL(1, CANHandler::CANBUS_2, M3508,"RightFly");
DJIMotor LFLYWHEEL(2, CANHandler::CANBUS_2, M3508,"LeftFly");

//CV STUFF
static BufferedSerial bcJetson(PC_12, PD_2, 115200);  //JETSON PORT
float xRotated, yRotated;

char yaw_angle_char[4];
char yaw_velocity_char[4];
char pitch_angle_char[4];
char pitch_velocity_char[4];

char nucleo_value[30] = {0};
#define JETSON_READ_BUFF_SIZE 500
#define JETSON_READ_MSG_SIZE 11
char jetson_read_buff[JETSON_READ_BUFF_SIZE] = {0};
int jetson_read_buff_pos = 0;
//CV
float CV_pitch_angle_radians = 0.0;
float CV_yaw_angle_radians = 0.0;
char CV_shoot = 0;


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
float jetson_send_feedback(float yaw_degrees) {
    char rotate_x_char[sizeof(float)];
    char rotate_y_char[sizeof(float)];
    getBytesFromFloat(rotate_x_char, xRotated);
    getBytesFromFloat(rotate_y_char, yRotated);

    //GET yaw and pitch data
    //float yaw_angle = ChassisSubsystem::ticksToRadians(yaw.getData(ANGLE)); //Ticks
    float yaw_angle = yaw_degrees * M_PI / 180;
    float yaw_velocity = yaw.getData(VELOCITY)/60.0; //RPM

    float pitch_angle = ChassisSubsystem::ticksToRadians((pitch_zero_offset_ticks - pitch.getData(ANGLE)));
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
//TODO: remove printf's
ssize_t jetson_read_values(float &pitch_move, float & yaw_move, char &shoot_switch) {
    bcJetson.set_blocking(false);

    if(!bcJetson.readable()){
      return -EAGAIN;
    }

    if(jetson_read_buff_pos > (JETSON_READ_BUFF_SIZE - JETSON_READ_MSG_SIZE)){
      printf("WARN: jetson read buffer overflow. Resetting buffer to 0\n");
      jetson_read_buff_pos = 0;
    }

    //TODO: keep a persistent buffer where if no matches are found we keep appending to the buffer until we find a match
    int available_space = JETSON_READ_BUFF_SIZE - jetson_read_buff_pos;
    ssize_t bytes_read = bcJetson.read(jetson_read_buff + jetson_read_buff_pos, available_space);
    if(bytes_read == -EAGAIN){
      return -EAGAIN;
        }

    jetson_read_buff_pos += bytes_read;

    if(jetson_read_buff_pos < JETSON_READ_MSG_SIZE){
      return -1;
        }

        //starting at the very last index of jetson_values (numbytes - 1)
        //i - 10 is the magic byte, and we will use next 9 bytes for checksum which is at i; exclude magic byte in checksum
        //if we meet the match conditions, decode values, clear buffer, return last amount of bytes read
        //if no match found, print no match
        //if buffer empty, print empty
    for(int i = jetson_read_buff_pos - 1 ; i >= 10 ; --i) {
      //calculating checksum without magic header bytes
      //check for magic byte, check checksum != 0, check calculated checksum matches message checksum
      if ((jetson_read_buff[i-10] == MAGICBYTE) &&
          (jetson_read_buff[i] != 0) && 
          (calculateLRC(&jetson_read_buff[i - 9], 9) == jetson_read_buff[i])){

          uint8_t checkSum;
          decode_toSTM32(&jetson_read_buff[i-9], pitch_move, yaw_move, shoot_switch, checkSum);
          //TODO: as an optimization we can clear onto the message we extracted. Leaving any potential partial messages in the buffer
          jetson_read_buff_pos = 0;
          return 1;
            }
        }

  return -1;
}

int main(){

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();

    bcJetson.set_blocking(false);

    /*
    * MOTORS SETUP AND PIDS
    */
    //YAW
    PID yawBeyblade(1.0, 0, 0); //yaw PID is cascading, so there are external position PIDs for yaw control
    // PID yawNonBeyblade(0.15, 0, 550);
    yawBeyblade.setOutputCap(90);
    yaw.setSpeedPID(550, 0, 0);
    yaw.setSpeedIntegralCap(8000);
    yaw.setSpeedOutputCap(32000);
    yaw.outputCap = 16000;
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
    pitch.setPositionPID(26.2644, 0.034926, 1200); //15, 0 1700
    pitch.setPositionOutputCap(32000);
    pitch.setPositionIntegralCap(3000);
    pitch.pidPosition.feedForward = 0;
    pitch.outputCap = 16000;
    pitch.useAbsEncoder = true;

    float pitch_current_angle = 0;
    float pitch_desired_angle = 0;
    float pitch_phase_angle = 33 / 180.0 * PI; // 5.69 theoretical //wtf is this?
    float K = 0.38; // 0.75 //0.85

    //FLYWHEELS
    LFLYWHEEL.setSpeedPID(7.1849, 0.000042634, 0);
    RFLYWHEEL.setSpeedPID(7.1849, 0.000042634, 0);

    //INDEXER
    indexer.setSpeedPID(1, 0, 1);
    indexer.setSpeedIntegralCap(8000);
    //Cascading PID for indexer angle position control. Surely there are better names then "sure"...
    PID sure(0.5,0,0.4);
    sure.setOutputCap(4000);
    //Variables for burst fire
    unsigned long timeSure;
    unsigned long prevTimeSure;
    bool shoot = false;
    int shootTargetPosition = 36*8190 ;
    bool shootReady = false;

    //CHASSIS
    Chassis.setYawReference(&yaw, 6500); //the number of ticks of yaw considered to be robot-front
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

    bool cv_enabled = false;

    ChassisSpeeds cs;

    while(true){
        timeStart = us_ticker_read();

        //CV loop runs every 2ms
        if((timeStart - loopTimerCV) / 1000 > 1) { //1 with sync or 2 without
            loopTimerCV = timeStart;
            //if(remote.rightSwitch() == Remote::SwitchState::UP)
            jetson_send_feedback(imuAngles.yaw + 180); //  __COMENTED OUT LOOLOOKOKOLOOOOKO HERHEHRERHEHRHE
            //basic_bitch_read();
        }

        if ((timeStart - loopTimer) / 1000 > OUTER_LOOP_DT_MS){
            float elapsedms = (timeStart - loopTimer) / 1000;
            loopTimer = timeStart;
            led = !led;
            refLoop++;
            if (refLoop >= 5){
                led2 = referee.readable();
                refereeThread(&referee);
                refLoop = 0;

                //POWER LIMIT OVERRIDE INCASE
                if(robot_status.chassis_power_limit < 10){
                    chassis_power_limit = 49;
                }else{
                    chassis_power_limit = robot_status.chassis_power_limit;
                }
                
                Chassis.power_limit = (float)chassis_power_limit;
                chassis_buffer = power_heat_data.buffer_energy;
            }
            Chassis.periodic();
            cs = Chassis.getChassisSpeeds();
            remoteRead();

            int readResult = jetson_read_values(CV_pitch_angle_radians, CV_yaw_angle_radians, CV_shoot);

            if(cv_enabled){
                if(readResult > 0){
                    yaw_desired_angle = CV_yaw_angle_radians / M_PI * 180;
                    pitch_desired_angle = CV_pitch_angle_radians / M_PI * 180;
                }
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
            if(remote.keyPressed(Remote::Key::V)){
                shot = 'm';
            }else if(remote.keyPressed(Remote::Key::C)){
                shot = 'd';        
            }

            if(remote.keyPressed(Remote::Key::F)){
                cv_enabled = true;
            }else if(remote.keyPressed(Remote::Key::G)){
                cv_enabled = false;
            }


            // Mouse sensitivity initialized
            float MOUSE_SENSITIVITY_YAW_DPS = 10.0;
            float MOUSE_SENSITIVITY_PITCH_DPS = 10.0;
            
            // right click (hold) slows decreases sensitivity
            if (remote.getMouseR()) {
                MOUSE_SENSITIVITY_YAW_DPS = 5.0;
                MOUSE_SENSITIVITY_PITCH_DPS = 5.0;
            }


            //Driving input
            float scalar = 1;
            float jx = remote.leftX() / 660.0 * scalar; // -1 to 1
            float jy = remote.leftY() / 660.0 * scalar; // -1 to 1
            //Pitch, Yaw
            float jpitch = remote.rightY() / 660.0 * scalar; // -1 to 1
            float jyaw = remote.rightX() / 660.0 * scalar; // -1 to 1

            float myaw = remote.getMouseX();
            float mpitch = -remote.getMouseY();

            //joystick tolerance
            float tolerance = 0.05; 
            jx = (abs(jx) < tolerance) ? 0 : jx;
            jy = (abs(jy) < tolerance) ? 0 : jy;
            jpitch = (abs(jpitch) < tolerance) ? 0 : jpitch;
            jyaw = (abs(jyaw) < tolerance) ? 0 : jyaw;
            
            //Keyboard Driving
            float mult = 1;

            // Shift to make robot go slower
            if (remote.keyPressed(Remote::Key::SHIFT)) {
                mult = 0.5;
            }

            jx += mult * ((remote.keyPressed(Remote::Key::D) ? 1 : 0) + (remote.keyPressed(Remote::Key::A) ? -1 : 0));
            jy += mult * ((remote.keyPressed(Remote::Key::W) ? 1 : 0) + (remote.keyPressed(Remote::Key::S) ? -1 : 0));

            //Bounding the four j variables
            jx = max(-1.0F, min(1.0F, jx));
            jy = max(-1.0F, min(1.0F, jy));
            jpitch = max(-1.0F, min(1.0F, jpitch));
            jyaw = max(-1.0F, min(1.0F, jyaw));

            //Chassis Code
            if (drive == 'u' || (drive =='o' && remote.rightSwitch() == Remote::SwitchState::UP)){
                //REGULAR DRIVING CODE
                Chassis.setChassisSpeeds({jx * Chassis.m_OmniKinematicsLimits.max_Vel,
                                          jy * Chassis.m_OmniKinematicsLimits.max_Vel,
                                          0 * Chassis.m_OmniKinematicsLimits.max_vOmega},
                                          ChassisSubsystem::YAW_ORIENTED);
            }else if (drive == 'd' || (drive =='o' && remote.rightSwitch() == Remote::SwitchState::DOWN)){
                //BEYBLADE DRIVING CODE
                Chassis.setChassisSpeeds({jx * Chassis.m_OmniKinematicsLimits.max_Vel,
                                          jy * Chassis.m_OmniKinematicsLimits.max_Vel,
                                          -BEYBLADE_OMEGA},
                                          ChassisSubsystem::YAW_ORIENTED);
            }else{
                //OFF
                Chassis.setWheelPower({0,0,0,0});
            }

            //YAW CODE
            if (drive == 'u' || drive == 'd' || (drive =='o' && (remote.rightSwitch() == Remote::SwitchState::UP || remote.rightSwitch() == Remote::SwitchState::DOWN))){
                float chassis_rotation_radps = cs.vOmega;
                int chassis_rotation_rpm = chassis_rotation_radps * 60 / (2*M_PI) * 4; //I added this 4 but I don't know why.
                
                //Regular Yaw Code
                yaw_desired_angle -= myaw * MOUSE_SENSITIVITY_YAW_DPS * elapsedms / 1000;
                yaw_desired_angle -= jyaw * JOYSTICK_SENSITIVITY_YAW_DPS * elapsedms / 1000;
                //yaw_desired_angle = (yaw_desired_angle + 360) % 360;
                yaw_desired_angle = floatmod(yaw_desired_angle, 360);

                prevTimeSure = timeSure;
                timeSure = us_ticker_read();
                #ifdef USE_IMU
                yawVelo = yawBeyblade.calculatePeriodic(DJIMotor::s_calculateDeltaPhase(yaw_desired_angle, imuAngles.yaw + 180, 360), timeSure - prevTimeSure);
                #else
                yawVelo = -jyaw * JOYSTICK_SENSITIVITY_YAW_DPS / 360.0 * 60;
                #endif
                //yawVelo = 0;
                yawVelo -= chassis_rotation_rpm;

                int dir = 0;
                if(yawVelo > 0){
                    dir = 1;
                }else if(yawVelo < 0){
                    dir = -1;
                }
                yaw.pidSpeed.feedForward = 1221*dir + 97.4 * yawVelo;
                yaw.setSpeed(yawVelo);
            }else{
                //Off
                yaw.setPower(0);
                #ifdef USE_IMU
                yaw_desired_angle = imuAngles.yaw + 180;
                #endif
            }

            //PITCH
            if (drive == 'u' || drive == 'd' || (drive =='o' && (remote.rightSwitch() == Remote::SwitchState::UP || remote.rightSwitch() == Remote::SwitchState::DOWN))){
                //Regular Pitch Code
                pitch_desired_angle += mpitch * MOUSE_SENSITIVITY_PITCH_DPS * elapsedms / 1000;
                pitch_desired_angle += jpitch * JOYSTICK_SENSITIVITY_PITCH_DPS * elapsedms / 1000;

                if (pitch_desired_angle <= LOWERBOUND) {
                    pitch_desired_angle = LOWERBOUND;
                }
                else if (pitch_desired_angle >= UPPERBOUND) {
                    pitch_desired_angle = UPPERBOUND;
                }

                //float FF = K * sin((desiredPitch / 180 * PI) - pitch_phase); // output: [-1,1]
                //float FF = K * cos(pitch_desired_angle / 180 * PI);
                //pitch.pidPosition.feedForward = int((INT16_T_MAX) * FF);
                float pitch_desired_radians = -(pitch_desired_angle / 360) * 2 * M_PI;
                pitch.pidPosition.feedForward = cos(pitch_desired_radians) * -2600;
                pitch.setPosition(-int((pitch_desired_angle / 360) * TICKS_REVOLUTION - pitch_zero_offset_ticks));
            }else{
                //Off
                pitch.setPower(0);
            }
            pitch_current_angle = (pitch_zero_offset_ticks - (pitch>>ANGLE)) / TICKS_REVOLUTION * 360;

            //INDEXER CODE
            if ((remote.leftSwitch() == Remote::SwitchState::UP || remote.getMouseL()) && (abs(RFLYWHEEL>>VELOCITY) > 6000 && abs(LFLYWHEEL>>VELOCITY) > 6000)){
                if (shootReady){
                    shootReady = false;
                    shootTargetPosition = 8192 * 12 + (indexer>>MULTITURNANGLE);

                    //shoot limit
                    if(robot_status.shooter_barrel_heat_limit < 10 || power_heat_data.shooter_17mm_1_barrel_heat < robot_status.shooter_barrel_heat_limit - 40) {
                        shoot = true;
                    }
                    
                }
            } else {
                //SwitchState state set to mid/down/unknown
                shootReady = true;
            }

            // burst fire, turn the indexer to shoot 3-5 balls a time and stop indexer
            // only shoot when left switch changes from down/unknown/mid to up
            // if left switch remains at up state, indexer stops after 3-5 balls
            if (shoot){
                if (indexer>>MULTITURNANGLE >= shootTargetPosition){
                    indexer.setSpeed(0);
                    shoot = false;
                } else {
                    timeSure = us_ticker_read();
                    indexer.setSpeed(sure.calculate(shootTargetPosition, indexer>>MULTITURNANGLE, timeSure - prevTimeSure)); //
                    prevTimeSure = timeSure;
                }
            } else {
                indexer.setSpeed(0);
            }

            //FLYWHEELS
            if (shot == 'm' || (shot == 'o' && remote.leftSwitch() != Remote::SwitchState::DOWN &&
                remote.leftSwitch() != Remote::SwitchState::UNKNOWN)){
                RFLYWHEEL.setSpeed(7000);
                LFLYWHEEL.setSpeed(-7000);
            } else{
                // left SwitchState set to up/mid/unknown
                RFLYWHEEL.setSpeed(0);
                LFLYWHEEL.setSpeed(0);
            }

            printLoop ++;
            if (printLoop >= PRINT_FREQUENCY){
                printLoop = 0;
                //printff("Prints:\n");
                //printff("lX:%.1f lY:%.1f rX:%.1f rY:%.1f lS:%d rS:%d\n", remote.leftX(), remote.leftY(), remote.rightX(), remote.rightY(), remote.leftSwitch(), remote.rightSwitch());
                //printff("jx:%.3f jy:%.3f jpitch:%.3f jyaw:%.3f\n", jx, jy, jpitch, jyaw);
                #ifdef USE_IMU
                //printff("yaw_des_v:%d yaw_act_v:%d", yawVelo, yaw>>VELOCITY);
                //printff("YD:%.3f YA:%.3f CVY:%.3f\n", yaw_desired_angle, imuAngles.yaw + 180, CV_yaw_angle_radians * 180 / M_PI);
                #else
                // printff("yaw_des_v:%d yaw_act_v:%d\n", yawVelo, yaw>>VELOCITY);
                //printff("yaw_des:%.3f yaw_act:%.3f [%d]\n", yaw_desired_angle, yaw_current_angle, yaw>>ANGLE);
                #endif
                //printff("yaw_des_v:%d yaw_act_v:%d", yawVelo, yaw>>VELOCITY);
                //printff("pitch_des:%.3f pitch_act:%.3f [%d]\n", pitch_desired_angle, pitch_current_angle, pitch>>ANGLE);
                //printff("cX%.1f cY%.1f cOmega%.3f cRPM%.1f\n", cs.vX, cs.vY, cs.vOmega, cs.vOmega * 60 / (2*M_PI) * 4);
                // printff("Chassis: LF:%c RF:%c LB:%c RB:%c Yaw:%c Pitch:%c Flywheel_L:%c Flywheel_R:%c Indexer:%c\n", 
                //     Chassis.getMotor(ChassisSubsystem::LEFT_FRONT).isConnected() ? 'y' : 'n', 
                //     Chassis.getMotor(ChassisSubsystem::RIGHT_FRONT).isConnected() ? 'y' : 'n', 
                //     Chassis.getMotor(ChassisSubsystem::LEFT_BACK).isConnected() ? 'y' : 'n', 
                //     Chassis.getMotor(ChassisSubsystem::RIGHT_BACK).isConnected() ? 'y' : 'n',
                //     yaw.isConnected() ? 'y' : 'n', 
                //     pitch.isConnected() ? 'y' : 'n', 
                //     LFLYWHEEL.isConnected() ? 'y' : 'n', 
                //     RFLYWHEEL.isConnected() ? 'y' : 'n',
                //     indexer.isConnected() ? 'y' : 'n');
                #ifdef USE_IMU
                // printff("IMU %.3f %.3f %.3f\n",imuAngles.yaw, imuAngles.pitch, imuAngles.roll);
                #endif
                //printff("pwr:%u max:%d heat:%d\n", chassis_buffer, robot_status.chassis_power_limit, power_heat_data.shooter_17mm_1_barrel_heat);
                //printff("ID:%d LVL:%d HP:%d MAX_HP:%d\n", robot_status.robot_id, robot_status.robot_level, robot_status.current_HP, robot_status.maximum_HP);
                //printff("elap:%.5fms\n", elapsedms);
                //printff("[HEAT] lim:%d buf:%d b1:%d b2:%d sp:%.1f fr:%d\n", robot_status.shooter_barrel_heat_limit, power_heat_data.buffer_energy, power_heat_data.shooter_17mm_1_barrel_heat, power_heat_data.shooter_17mm_2_barrel_heat, shoot_data.initial_speed, shoot_data.launching_frequency);

                // if(nucleo_value[24] != 0 && remote.rightSwitch() == Remote::SwitchState::UP){
                //     printff("[");
                //     for(int i = 0; i < 30; i ++){
                //         printff("%x,", nucleo_value[i]);
                //     }
                //     printff("]");
                // }
                printff("CS: %.1f %.1f %.1f\n", cs.vX, cs.vY, cs.vOmega);
            }

            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}