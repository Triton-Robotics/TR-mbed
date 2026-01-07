#pragma clang diagnostic push
// #pragma ide diagnostic ignored "EndlessLoop"

#include "DJIMotor.h"

int DJIMotor::motorCount = 0;
DJIMotor* DJIMotor::s_allMotors[2][3][4];
bool DJIMotor::s_motorsExist[2][3][4];
CANHandler* DJIMotor::s_canHandlers[2];
bool DJIMotor::initializedWarning = false;


DJIMotor::DJIMotor(bool isErroneousMotor)
{
    canID_0 = -1;
    motorID_0 = -1;
    canBus = CANHandler::NOBUS;
    value = 0;
    mode = OFF;

    if(isErroneousMotor)
        mode = ERR;

    powerOut = 0;
}


DJIMotor::DJIMotor(short motorID, CANHandler::CANBus canBus, motorType type, const std::string& name)
{
    DJIMotor::motorCount++;
    canID_0 = static_cast<short>(motorID - 1);
    motorID_0 = canID_0;
    this -> canBus = canBus;
    this -> type = type;
    this -> name = name;

    value = 0;

    // Adding 4 to move to the next ID line
    if(type == GM6020){
        canID_0 += 4;
        useAbsEncoder = true;
    }

    int can_line = canID_0 / 4; // The can line depends on the motor id and type: M3508 1-4 on line 0, 5-8 on line 1, GM6020 1-4 on line 1, 5-8 on line 2
    int index = canID_0 % 4; // The index within the can line

    if(!s_motorsExist [canBus][can_line][index])
    {
        s_allMotors   [canBus][can_line][index] = this;
        s_motorsExist [canBus][can_line][index] = true;

    }else
    {
        DJIMotor mot(true);
        s_allMotors[canBus][can_line][index] = &mot;
        printf("[ERROR] THERES A CONFLICT ON BUS [%d] MOTOR [%d], \"%s\". YOU WILL HAVE ERRORS.\n", canBus + 1, motorID + 1, name.c_str());
    }
    
    if (motorID > 8 || motorID < 1)
    {
        printf("[ERROR] The canID [%d] not within correct bounds\n", motorID);
    }

    powerOut = 0;
}


DJIMotor::DJIMotor(config cfg)
    : DJIMotor(cfg.motorID, cfg.canBus, cfg.type, cfg.name)
{
    pidSpeed = PID(cfg.vel_cfg);
    pidPosition = PID(cfg.pos_cfg);
}


DJIMotor::~DJIMotor()
{
    mode = OFF;
    s_motorsExist[canBus][canID_0 / 4][canID_0 % 4] = false;
    canID_0 = -1;
    motorID_0 = -1;
    canBus = CANHandler::NOBUS;
}


void DJIMotor::setOutput()
{
    uint32_t time = us_ticker_read();
    int powerOut;

    if(mode == SPD)
        powerOut = calculateSpeedPID(value, getData(VELOCITY), static_cast<double>(time - timeOfLastPID));

    else if(mode == POS)
        if(!useAbsEncoder)
            powerOut = calculatePositionPID(value, getData(MULTITURNANGLE), static_cast<double>(time - timeOfLastPID));

        else
            powerOut = calculatePeriodicPosition(static_cast<float>(calculateDeltaPhase(value, getData(ANGLE))), static_cast<double>(time - timeOfLastPID));

    else if(mode == POW)
        powerOut = value;

    else if(mode == OFF)
        powerOut = 0;

    else
        printf("[ERROR] THIS IS AN ERRONEOUS MOTOR. ON BUS [%d] MOTOR [%d], \"%s\". YOU WILL HAVE ERRORS.\n. DO NOT ATTEMPT TO SEND IT DATA, DO NOT PASS GO, DO NOT COLLECT $200, FIX THIS!\n",
               canBus + 1, motorID_0 + 1, name.c_str());

    if(type == M3508_FLYWHEEL || type == M3508 || type == M2006){
        if(powerOut > outputCap || powerOut > INT15_T_MAX)
            powerOut = min(outputCap, INT15_T_MAX);

        else if(powerOut < -outputCap || powerOut < -INT15_T_MAX)
            powerOut = max(-outputCap, -INT15_T_MAX);

    }else{
        if(powerOut > outputCap || powerOut > INT16_T_MAX)
            powerOut = min(outputCap, INT16_T_MAX);

        else if(powerOut < -outputCap || powerOut < -INT16_T_MAX)
            powerOut = max(-outputCap, -INT16_T_MAX);
    }

    this -> powerOut = static_cast<int16_t>(powerOut);
    timeOfLastPID = time;
}


void DJIMotor::sendValues(bool debug)
{
    for(short canBus = 0; canBus < CAN_HANDLER_NUMBER; canBus++)
        for(short sendIDindex = 0; sendIDindex < 3; sendIDindex++)
            sendOneID((CANHandler::CANBus) canBus, sendIDindex, debug);
}


void DJIMotor::sendOneID(CANHandler::CANBus canBus, short sendIDindex, bool debug)
{
    int8_t bytes[8]  = {0,0,0,0,0,0,0,0};
    bool anyMotors = false;

    if (debug)
        printf("0x%x:\t", s_sendIDs[sendIDindex]);

    for (int i = 0; i < 4; i++)
    {
        if (s_motorsExist[canBus][sendIDindex][i])
        {
            s_allMotors[canBus][sendIDindex][i] -> setOutput();
            int16_t powerOut = s_allMotors[canBus][sendIDindex][i] -> powerOut;

            bytes[2 * i] =      static_cast<int8_t>(powerOut >> 8);
            bytes[2 * i + 1] =  static_cast<int8_t>(powerOut);

            anyMotors = true;

            if(debug)
                printf("%d\t", powerOut);

        }else if (debug)
            printf("NA\t");
    }

    if(anyMotors)
        if(s_canHandlers[canBus] -> exists)
            s_canHandlers[canBus] -> rawSend(s_sendIDs[sendIDindex], bytes);

        else
            printf("\n[ERROR] YOUR CANHANDLERS ARE NOT DEFINED YET. DO THIS BEFORE YOU CALL ANY MOTORS,\n USING [(DJIMotor::s_setCANHandlers(PA_11,PA_12,PB_12,PB_13)], WHERE PA_11, PA_12 ARE TX, RX\n");

    if(debug) printf("\n");
}


// Callback function to populate for CANBus_1
void DJIMotor::getCan1Feedback(const CANMsg * msg) 
{
    int canBus = 0;
    int canID_0 = msg->id - 0x201;

    // Find which motor line we are in (0x200, 0x1FF, 0x2FF)
    int can_line = canID_0 / 4;

    // Find which motor it is
    int motor_id = canID_0 % 4;

    // Update motor specific data
    if(s_motorsExist[canBus][can_line][motor_id]) {
        DJIMotor* motor = s_allMotors[canBus][canID_0 / 4][canID_0 % 4];

        motor -> motorData[ANGLE]       = static_cast<int16_t>(msg->data[0] << 8 | msg->data[1]);
        motor -> motorData[VELOCITY]    = static_cast<int16_t>(msg->data[2] << 8 | msg->data[3]);
        motor -> motorData[TORQUE]      = static_cast<int16_t>(msg->data[4] << 8 | msg->data[5]);
        motor -> motorData[TEMPERATURE] = static_cast<int16_t>(msg->data[6]);
        motor -> timeOfLastFeedback     = us_ticker_read() / 1000;

    if(motor -> motorData[TEMPERATURE] > 80)
        printf("[WARNING] YOU HAVE A MOTOR [0x%x] ATTACHED THAT IS %d DEGREES CELSIUS ON BUS [%d] ID [%d], \"%s\" \n", msg->id, motor -> motorData[TEMPERATURE], canBus + 1, motor -> motorID_0 + 1, motor -> name.c_str());

    }else if(initializedWarning){
        printf("[WARNING] YOU HAVE A MOTOR [0x%x] {%d}{%d}{%d} ATTACHED THAT IS NOT INITIALIZED.. WHY: \n", msg->id, canBus, can_line, motor_id);
    }

    updateOneMultiTurn(canBus, can_line, motor_id);
}


// Callback function to populate for CANBus_2
void DJIMotor::getCan2Feedback(const CANMsg * msg) 
{
    int canBus = 1;
    int canID_0 = msg->id - 0x201;

    // Find which motor line we are in (0x200, 0x1FF, 0x2FF)
    int can_line = canID_0 / 4;

    // Find which motor it is
    int motor_id = canID_0 % 4;

    // Update motor specific data
    if(s_motorsExist[canBus][can_line][motor_id]) {
        DJIMotor* motor = s_allMotors[canBus][canID_0 / 4][canID_0 % 4];

        motor -> motorData[ANGLE]       = static_cast<int16_t>(msg->data[0] << 8 | msg->data[1]);
        motor -> motorData[VELOCITY]    = static_cast<int16_t>(msg->data[2] << 8 | msg->data[3]);
        motor -> motorData[TORQUE]      = static_cast<int16_t>(msg->data[4] << 8 | msg->data[5]);
        motor -> motorData[TEMPERATURE] = static_cast<int16_t>(msg->data[6]);
        motor -> timeOfLastFeedback     = us_ticker_read() / 1000;

    if(motor -> motorData[TEMPERATURE] > 80)
        printf("[WARNING] YOU HAVE A MOTOR [0x%x] ATTACHED THAT IS %d DEGREES CELSIUS ON BUS [%d] ID [%d], \"%s\" \n", msg->id, motor -> motorData[TEMPERATURE], canBus + 1, motor -> motorID_0 + 1, motor -> name.c_str());

    }else if(initializedWarning){
        printf("[WARNING] YOU HAVE A MOTOR [0x%x] {%d}{%d}{%d} ATTACHED THAT IS NOT INITIALIZED.. WHY: \n", msg->id, canBus, can_line, motor_id);
    }

    updateOneMultiTurn(canBus, can_line, motor_id);
}

// TODO: Fix again
void DJIMotor::updateOneMultiTurn(int canBus, int can_line, int motor_id) 
{
    int Threshold = 3000; // From 0 to 8191
    int curAngle, lastAngle, deltaAngle, speed;
    DJIMotor *curMotor;

    unsigned long time = us_ticker_read();

    if (s_motorsExist[canBus][can_line][motor_id]) {
        curMotor = s_allMotors[canBus][can_line][motor_id];

        // Update data from most recent frame
        curAngle = curMotor->getData(ANGLE);
        lastAngle = curMotor->lastMotorAngle;
        speed = curMotor->getData(VELOCITY);
        deltaAngle = curAngle - lastAngle;

        // Update multiturn data using latest data
        double dt = 0;
        if (curMotor->lastIntegrationTime != -1) {
            dt = (double) (time - curMotor->lastIntegrationTime) / 1000.0;
        }
        
        curMotor->integratedAngle += (int) (speed * dt * 19 * 6 / 1000.0);
        int positiveDiff = curAngle - curMotor->integratedAngle % 8192;
        
        if (positiveDiff >= 0 && positiveDiff < 4096) {
            curMotor->integratedAngle += positiveDiff;
        } 
        else if (positiveDiff < 0 && positiveDiff > -4096) {
            curMotor->integratedAngle += positiveDiff;
        } 
        else if (positiveDiff >= 4096) {
            curMotor->integratedAngle += positiveDiff - 8192;
        } 
        else if (positiveDiff <= -4096) {
            curMotor->integratedAngle += positiveDiff + 8192;
        } 
        else {
            //printf("Don't know what to do with this %i\n", positiveDiff);
        }

        if (abs(speed) < 100) {
            if (curAngle > (8191 - Threshold) && lastAngle < Threshold)
                curMotor->multiTurn -= deltaAngle - 8191;
            else if (curAngle < Threshold && lastAngle > (8191 - Threshold))
                curMotor->multiTurn += deltaAngle + 8191;
            else
                curMotor->multiTurn += deltaAngle;
        }
        else {
            if (speed < 0 && deltaAngle > 0)     {
                // neg skip
                // printf("positive overclock: %i\n", (int) speed);
                curMotor->multiTurn += deltaAngle - 8191;
            }
            else if (speed > 0 && deltaAngle < 0)     {
                // pos skip
                // printf("negative overclock: %i\n", (int) speed);
                curMotor->multiTurn += deltaAngle + 8191;
            }
            else
                curMotor->multiTurn += deltaAngle;
        }
        // Update new multiturn
        curMotor->lastMotorAngle = curAngle;
        curMotor->lastIntegrationTime = long(time);
    }
}


int DJIMotor::getData(motorDataType data) 
{
    if(data <= 3)
        return motorData[data];

    else if(data == MULTITURNANGLE)
        return multiTurn;

    else if(data == POWEROUT)
        return powerOut;

    else if(data == VALUE)
        return value;

    return 0;

}


int DJIMotor::calculateDeltaPhase(int target, int current, int max) {
    target %= max;

    int deltaPhase = target - current;

    if(abs(deltaPhase) > max / 2)
    {
        if(deltaPhase > 0)
            deltaPhase -= max;
        else
            deltaPhase += max;
    }
    return deltaPhase;
}


float DJIMotor::calculateDeltaPhase(float target, float current, float max) {
    //target %= max;

    float deltaPhase = target - current;

    if(abs(deltaPhase) > max / 2){
        if(deltaPhase > 0)
            deltaPhase -= max;

        else
            deltaPhase += max;
    }
    return deltaPhase;
}


#pragma clang diagnostic pop