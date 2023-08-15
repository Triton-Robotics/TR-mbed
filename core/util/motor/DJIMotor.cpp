#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

#include "DJIMotor.h"

DJIMotor* DJIMotor::s_allMotors[2][3][4];
bool DJIMotor::s_motorsExist[2][3][4];
CANHandler* DJIMotor::s_canHandlers[2];

DJIMotor::DJIMotor(bool isErroneousMotor){
    canID_0 = -1;
    motorID_0 = -1;
    canBus = CANHandler::NOBUS;
    value = 0;
    mode = OFF;

    if(isErroneousMotor)
        mode = ERR;

    powerOut = 0;
}

DJIMotor::DJIMotor(short motorID, CANHandler::CANBus canBus, motorType type, const std::string& name){

    canID_0 = static_cast<short>(motorID - 1);
    motorID_0 = canID_0;
    this -> canBus = canBus;
    this -> type = type;
    this -> name = name;

    value = 0;

    if(type == GM6020){
        canID_0 += 4;

        setSpeedPID(gimbly.speed.p, gimbly.speed.i, gimbly.speed.d);
        setSpeedOutputCap(gimbly.speed.outputCap);
        setSpeedIntegralCap(gimbly.speed.integralCap);

        setPositionPID(gimbly.pos.p, gimbly.pos.i, gimbly.pos.d);
        setPositionOutputCap(gimbly.pos.outputCap);
        setPositionIntegralCap(gimbly.pos.integralCap);

    }else if(type == M3508){
        setSpeedPID(m3508.speed.p, m3508.speed.i, m3508.speed.d);
        setSpeedOutputCap(m3508.speed.outputCap);
        setSpeedIntegralCap(m3508.speed.integralCap);

        setPositionPID(m3508.pos.p, m3508.pos.i, m3508.pos.d);
        setPositionOutputCap(m3508.pos.outputCap);
        setPositionIntegralCap(m3508.pos.integralCap);

    }else if(type == M3508_FLYWHEEL){
        setSpeedPID(m3508Flywheel.speed.p, m3508Flywheel.speed.i, m3508Flywheel.speed.d);
        setSpeedOutputCap(m3508Flywheel.speed.outputCap);
        setSpeedIntegralCap(m3508Flywheel.speed.integralCap);

        setPositionPID(m3508Flywheel.pos.p, m3508Flywheel.pos.i, m3508Flywheel.pos.d);
        setPositionOutputCap(m3508Flywheel.pos.outputCap);
        setPositionIntegralCap(m3508Flywheel.pos.integralCap);
    }

    if(!s_motorsExist [canBus][canID_0 / 4][canID_0 % 4]){
        s_allMotors   [canBus][canID_0 / 4][canID_0 % 4] = this;
        s_motorsExist [canBus][canID_0 / 4][canID_0 % 4] = true;

    }else{
        DJIMotor mot(true);
        s_allMotors[canBus][canID_0 / 4][canID_0 % 4] = &mot;
        printf("[ERROR] THERES A CONFLICT ON BUS [%d] MOTOR [%d], \"%s\". YOU WILL HAVE ERRORS.\n", canBus + 1, motorID + 1, name.c_str());
    }

    if (motorID > 8 || motorID < 1)
        printf("[ERROR] The canID [%d] not within correct bounds\n", motorID);

    powerOut = 0;
}

DJIMotor::~DJIMotor(){
    mode = OFF;
    s_motorsExist[canBus][canID_0 / 4][canID_0 % 4] = false;
    canID_0 = -1;
    motorID_0 = -1;
    canBus = CANHandler::NOBUS;
}

void DJIMotor::s_setCANHandlers(CANHandler* bus_1, CANHandler* bus_2, bool threadSend, bool threadFeedback){
    s_canHandlers[0] = bus_1;
    s_canHandlers[1] = bus_2;

    if(threadSend)
        s_motorSendThread.start(s_sendThread);

    if(threadFeedback)
        s_motorFeedbackThread.start(s_feedbackThread);
}

void DJIMotor::setOutput(){
    uint32_t time = us_ticker_read();
    int powerOut;

    if(mode == SPD)
        powerOut = calculateSpeedPID(value, getData(VELOCITY), static_cast<double>(time - timeOfLastPID));

    else if(mode == POS)
        if (!useAbsEncoder)
            powerOut = calculatePositionPID(value, getData(MULTITURNANGLE), static_cast<double>(time - timeOfLastPID));

        else
            powerOut = calculatePositionPID(s_calculateDeltaTicks(value, getData(ANGLE)), static_cast<double>(time - timeOfLastPID));

    else if(mode == POW)
        powerOut = value;

    else if(mode == OFF)
        powerOut = 0;

    else
        printf("[ERROR] THIS IS AN ERRONEOUS MOTOR. ON BUS [%d] MOTOR [%d], \"%s\". YOU WILL HAVE ERRORS.\n. DO NOT ATTEMPT TO SEND IT DATA, DO NOT PASS GO, DO NOT COLLECT $200, FIX THIS!\n",
               canBus + 1, motorID_0 + 1, name.c_str());

    if(type == M3508_FLYWHEEL || type == M3508 || type == M2006){
        if(powerOut > outCap || powerOut > INT15_T_MAX)
            powerOut = min(outCap, INT15_T_MAX);

        else if(powerOut < -outCap || powerOut < -INT15_T_MAX)
            powerOut = max(-outCap, -INT15_T_MAX);

    }else{
        if(powerOut > outCap || powerOut > INT16_T_MAX)
            powerOut = min(outCap, INT16_T_MAX);

        else if(powerOut < -outCap || powerOut < -INT16_T_MAX)
            powerOut = max(-outCap, -INT16_T_MAX);
    }

    this -> powerOut = static_cast<int16_t>(powerOut);
    timeOfLastPID = time;
}

void DJIMotor::s_sendValues(bool debug){
    for(short canBus = 0; canBus < CAN_HANDLER_NUMBER; canBus++)
        for(short sendIDindex = 0; sendIDindex < 3; sendIDindex++)
            s_sendOneID((CANHandler::CANBus) canBus, sendIDindex, debug);
}

void DJIMotor::s_sendOneID(CANHandler::CANBus canBus, short sendIDindex, bool debug){
    int8_t bytes[8]  = {0,0,0,0,0,0,0,0};
    bool anyMotors = false;

    if(debug)
        printf("0x%x:\t", s_sendIDs[sendIDindex]);

    for(int i = 0; i < 4; i++){
        if(s_motorsExist[canBus][sendIDindex][i]){
            s_allMotors[canBus][sendIDindex][i] -> setOutput();
            int16_t powerOut = s_allMotors[canBus][sendIDindex][i] -> powerOut;

            bytes[2 * i] =      int8_t(powerOut >> 8);
            bytes[2 * i + 1] =  int8_t(powerOut);

            anyMotors = true;

            if(debug)
                printf("%d\t", powerOut);

        }else if (debug)
            printf("NA\t");
    }

    if(s_canHandlers[canBus] -> exists && anyMotors)
        s_canHandlers[canBus] -> rawSend(s_sendIDs[sendIDindex], bytes);

    else
        printf("\n[ERROR] YOUR CANHANDLERS ARE NOT DEFINED YET. DO THIS BEFORE YOU CALL ANY MOTORS,\n USING [(DJIMotor::s_setCANHandlers(PA_11,PA_12,PB_12,PB_13)], WHERE PA_11, PA_12 ARE TX, RX\n");

    if(debug) printf("\n");

}

void DJIMotor::s_getFeedback(bool debug){

    for(int canBus = 0; canBus < CAN_HANDLER_NUMBER; canBus++){
        uint8_t receivedBytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        int msgID;

        while(s_canHandlers[canBus] -> getFeedback(&msgID, receivedBytes, canBus)) {
            int canID_0 = msgID - 0x201;

            if(s_motorsExist[canBus][canID_0 / 4][canID_0 % 4]){
                DJIMotor* motor = s_allMotors[canBus][canID_0 / 4][canID_0 % 4];

                motor -> motorData[ANGLE]       = (int16_t)(receivedBytes[0] << 8 | receivedBytes[1]);
                motor -> motorData[VELOCITY]    = (int16_t)(receivedBytes[2] << 8 | receivedBytes[3]);
                motor -> motorData[TORQUE]      = (int16_t)(receivedBytes[4] << 8 | receivedBytes[5]);
                motor -> motorData[TEMPERATURE] = (int16_t) receivedBytes[6];
                motor -> timeOfLastFeedback     = us_ticker_read() / 1000;

                if(debug)
                    motor -> printAllMotorData();

                if(motor -> motorData[TEMPERATURE] > 80)
                    printf("[WARNING] YOU HAVE A MOTOR [0x%x] ATTACHED THAT IS %d DEGREES CELSIUS ON BUS [%d] ID [%d], \"%s\" \n", msgID, motor -> motorData[TEMPERATURE], canBus + 1, motor -> motorID_0 + 1, motor -> name.c_str());

            }else
                printf("[WARNING] YOU HAVE A MOTOR [0x%x] {%d}{%d}{%d} ATTACHED THAT IS NOT INITIALIZED.. WHY: \n", msgID, canBus, canID_0 / 4, canID_0 % 4);
        }
    }
    s_updateMultiTurnPosition();
}

__attribute__((unused)) bool DJIMotor::s_allMotorsConnected(bool debug){
    for(int canBus = 0; canBus < CAN_HANDLER_NUMBER; canBus++)
        for(int c = 0; c < 4; c++)
            for(int r = 0; r < 3; r++)
                if(s_motorsExist[canBus][r][c]) {
                    if (us_ticker_read() / 1000 - s_allMotors[canBus][r][c] -> timeOfLastFeedback > TIMEOUT_MS) {
                        if(debug) {
                            printf("Motor ID %d on bus: %d lost connection\n", s_allMotors[canBus][r][c] -> motorID_0 + 1, canBus + 1);
                            printf("Motor [%d][%d][%d]\n", canBus, r, c);
                        }
                        return false;
                    }
                }
    return true;
}

int DJIMotor::getData(motorDataType data) {
    if(data <= 3)
        return motorData[data];

    else if(data == MULTITURNANGLE)
        return multiTurn;

    else
        return powerOut;

}

void DJIMotor::printAllMotorData() {
    printf("TYPE:%d ANGL:%d MLTI:%d VELO:%d TORQ:%d TEMP:%d | PWR:%d canBus:%d ID:%d name:%s\n", type, getData(ANGLE), getData(MULTITURNANGLE), getData(VELOCITY), getData(TORQUE), getData(TEMPERATURE), powerOut, canBus + 1, motorID_0 + 1, name.c_str());
}

int DJIMotor::s_calculateDeltaTicks(int target, int current) {
    int deltaTicks = target - current;

    if(abs(deltaTicks) > TICKS_REVOLUTION / 2){
        if(deltaTicks > 0)
            deltaTicks -= TICKS_REVOLUTION;

        else
            deltaTicks += TICKS_REVOLUTION;
    }
    return deltaTicks;
}

void DJIMotor::s_updateMultiTurnPosition() {
    int Threshold = 3000; // From 0 to 8191
    int curAngle, lastAngle, deltaAngle, speed;
    DJIMotor *curMotor;

    unsigned long time = us_ticker_read();

    for(int x = 0; x < CAN_HANDLER_NUMBER; x++)
        for (int y = 0; y < 3; y++)
            for (int z = 0; z < 4; z++)
                if (s_motorsExist[x][y][z]) {
                    curMotor = s_allMotors[x][y][z];

                    curAngle = curMotor->getData(ANGLE);
                    lastAngle = curMotor->lastMotorAngle;
                    speed = curMotor->getData(VELOCITY);
                    deltaAngle = curAngle - lastAngle;
                    double dt = 0;
                    if (curMotor->lastIntegrationTime != -1) {
                        dt = (double) (time - curMotor->lastIntegrationTime) / 1000.0;
                    }
                    curMotor->integratedAngle += (int) (speed * dt * 19 * 6 / 1000.0);
                    int positiveDiff = curAngle - curMotor->integratedAngle % 8192;
                    if (positiveDiff >= 0 && positiveDiff < 4096) {
                        curMotor->integratedAngle += positiveDiff;
                    } else if (positiveDiff < 0 && positiveDiff > -4096) {
                        curMotor->integratedAngle += positiveDiff;
                    } else if (positiveDiff >= 4096) {
                        curMotor->integratedAngle += positiveDiff - 8192;
                    } else if (positiveDiff <= -4096) {
                        curMotor->integratedAngle += positiveDiff + 8192;
                    } else {
                        //printf("Don't know what to do with this %i\n", positiveDiff);
                    }
                    if (abs(speed) < 100)
                        if (curAngle > (8191 - Threshold) && lastAngle < Threshold)
                            curMotor->multiTurn -= deltaAngle - 8191;
                        else if (curAngle < Threshold && lastAngle > (8191 - Threshold))
                            curMotor->multiTurn += deltaAngle + 8191;
                        else
                            curMotor->multiTurn += deltaAngle;

                    else
                    if (speed < 0 && deltaAngle > 0)     {
                        // neg skip
//                            printf("positive overclock: %i\n", (int) speed);
                        curMotor->multiTurn += deltaAngle - 8191;
                    }
                    else if (speed > 0 && deltaAngle < 0)     {
//                            printf("negative overclock: %i\n", (int) speed);       // pos skip
                        curMotor->multiTurn += deltaAngle + 8191;
                    }
                    else
                        curMotor->multiTurn += deltaAngle;

                    curMotor->lastMotorAngle = curAngle;
                    curMotor->lastIntegrationTime = long(time);

                    if (curMotor->printAngle)
                        printf("Integ multi: %i %i\n", curMotor->integratedAngle, curMotor->multiTurn);

                }
}

/**
* @brief the thread that runs the ever-necessary DJIMotor::s_getFeedback()
*/

void DJIMotor::s_feedbackThread() {
    while (true) {
        s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}

/**
* @brief the thread that runs the ever-necessary DJIMotor::s_sendValues()
*/

void DJIMotor::s_sendThread() {
    while (true) {
        s_sendValues();
        ThisThread::sleep_for(1ms);
    }
}
#pragma clang diagnostic pop