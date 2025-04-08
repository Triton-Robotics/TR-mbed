#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

#include "DJIMotor.h"

int DJIMotor::motorCount = 0;
DJIMotor* DJIMotor::s_allMotors[2][3][4];
bool DJIMotor::s_motorsExist[2][3][4];
CANHandler* DJIMotor::s_canHandlers[2];
bool DJIMotor::initializedWarning = false;

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

    DJIMotor::motorCount++;
    canID_0 = static_cast<short>(motorID - 1);
    motorID_0 = canID_0;
    this -> canBus = canBus;
    this -> type = type;
    this -> name = name;

    value = 0;

    if(type == GM6020){
        canID_0 += 4;

        setSpeedPID(gimbly.speed.p, gimbly.speed.i, gimbly.speed.d);
        setSpeedOutputCap(gimbly.speed.outCap);
        setSpeedIntegralCap(gimbly.speed.integralCap);

        setPositionPID(gimbly.pos.p, gimbly.pos.i, gimbly.pos.d);
        setPositionOutputCap(gimbly.pos.outCap);
        setPositionIntegralCap(gimbly.pos.integralCap);

    }else if(type == M3508){
        setSpeedPID(m3508.speed.p, m3508.speed.i, m3508.speed.d);
        setSpeedOutputCap(m3508.speed.outCap);
        setSpeedIntegralCap(m3508.speed.integralCap);

        setPositionPID(m3508.pos.p, m3508.pos.i, m3508.pos.d);
        setPositionOutputCap(m3508.pos.outCap);
        setPositionIntegralCap(m3508.pos.integralCap);

    }else if(type == M3508_FLYWHEEL){
        setSpeedPID(m3508Flywheel.speed.p, m3508Flywheel.speed.i, m3508Flywheel.speed.d);
        setSpeedOutputCap(m3508Flywheel.speed.outCap);
        setSpeedIntegralCap(m3508Flywheel.speed.integralCap);

        setPositionPID(m3508Flywheel.pos.p, m3508Flywheel.pos.i, m3508Flywheel.pos.d);
        setPositionOutputCap(m3508Flywheel.pos.outCap);
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
        if(!useAbsEncoder)
            powerOut = calculatePositionPID(value, getData(MULTITURNANGLE), static_cast<double>(time - timeOfLastPID));

        else
            powerOut = calculatePeriodicPosition(static_cast<float>(s_calculateDeltaPhase(value, getData(ANGLE))), static_cast<double>(time - timeOfLastPID));

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

void DJIMotor::s_getFeedback(bool debug){

    for(int canBus = 0; canBus < CAN_HANDLER_NUMBER; canBus++){
        uint8_t receivedBytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        int msgID;

        while(s_canHandlers[canBus] -> getFeedback(&msgID, receivedBytes, canBus)) {
            int canID_0 = msgID - 0x201;

            if(s_motorsExist[canBus][canID_0 / 4][canID_0 % 4]){
                DJIMotor* motor = s_allMotors[canBus][canID_0 / 4][canID_0 % 4];

                motor -> motorData[ANGLE]       = static_cast<int16_t>(receivedBytes[0] << 8 | receivedBytes[1]);
                motor -> motorData[VELOCITY]    = static_cast<int16_t>(receivedBytes[2] << 8 | receivedBytes[3]);
                motor -> motorData[TORQUE]      = static_cast<int16_t>(receivedBytes[4] << 8 | receivedBytes[5]);
                motor -> motorData[TEMPERATURE] = static_cast<int16_t>(receivedBytes[6]);
                motor -> timeOfLastFeedback     = us_ticker_read() / 1000;

                if(debug)
                    motor -> printAllMotorData();

                if(motor -> motorData[TEMPERATURE] > 80)
                    printf("[WARNING] YOU HAVE A MOTOR [0x%x] ATTACHED THAT IS %d DEGREES CELSIUS ON BUS [%d] ID [%d], \"%s\" \n", msgID, motor -> motorData[TEMPERATURE], canBus + 1, motor -> motorID_0 + 1, motor -> name.c_str());

            }else if(initializedWarning){
                printf("[WARNING] YOU HAVE A MOTOR [0x%x] {%d}{%d}{%d} ATTACHED THAT IS NOT INITIALIZED.. WHY: \n", msgID, canBus, canID_0 / 4, canID_0 % 4);
            }
        }
    }
    s_updateMultiTurnPosition();
}

bool DJIMotor::s_theseConnected(DJIMotor* motors[], int size, bool debug) {
    if(!debug) {
        return std::all_of(motors, motors + size, [](DJIMotor *motor){
            return motor -> isConnected();
        });

    }else{
        bool allConnected = true;

        for (int i = 0; i < size; i++) {
            if (!motors[i] -> isConnected()) {
                allConnected = false;
                printf("Motor ID_1 %d on canBus_1: %d, \"%s\" lost connection\n", motors[i] -> motorID_0 + 1, motors[i] -> canBus + 1, motors[i] -> name.c_str());
            }
        }
        return allConnected;
    }
}

bool DJIMotor::s_allConnected(bool debug){
    if(!debug){
        return std::all_of(**s_allMotors, **s_allMotors + CAN_HANDLER_NUMBER * 3 * 4, [](DJIMotor *motor){
            return motor -> isConnected();
        });

    }else {
        bool allConnected = true;

        for (int canBus = 0; canBus < CAN_HANDLER_NUMBER; canBus++) {
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 4; c++) {
                    if (s_motorsExist[canBus][r][c] && !s_allMotors[canBus][r][c] -> isConnected()) {
                        allConnected = false;
                        printf("Motor ID_1 %d on canBus_1: %d, \"%s\" lost connection\n", s_allMotors[canBus][r][c] -> motorID_0 + 1, canBus + 1, s_allMotors[canBus][r][c] -> name.c_str());
                        printf("Motor [%d][%d][%d]\n", canBus, r, c);
                    }
                }
            }
        }
        return allConnected;
    }
}

int DJIMotor::getData(motorDataType data) {
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

void DJIMotor::printAllMotorData() {
    printf("TYPE:%d ANGL:%d MLTI:%d VELO:%d TORQ:%d TEMP:%d | PWR:%d canBus:%d ID:%d name:\"%s\"\n", type, getData(ANGLE), getData(MULTITURNANGLE), getData(VELOCITY), getData(TORQUE), getData(TEMPERATURE), powerOut, canBus + 1, motorID_0 + 1, name.c_str());
}

int DJIMotor::s_calculateDeltaPhase(int target, int current, int max) {
    target %= max;

    int deltaPhase = target - current;

    if(abs(deltaPhase) > max / 2){
        if(deltaPhase > 0)
            deltaPhase -= max;

        else
            deltaPhase += max;
    }
    return deltaPhase;
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
