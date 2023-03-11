//
// Created by ankit on 1/31/23.
//

#pragma clang diagnostic push
#pragma ide diagnostic ignored "ConstantConditionsOC"
#pragma ide diagnostic ignored "UnreachableCode"
#pragma ide diagnostic ignored "EndlessLoop"
#pragma ide diagnostic ignored "misc-unconventional-assign-operator"

#include "DJIMotor.h"

DJIMotor* DJIMotor::allMotors[2][3][4];
bool DJIMotor::motorsExist[2][3][4];
long int DJIMotor::lastCalled [CAN_HANDLER_NUMBER][3][4];

CANHandler* DJIMotor::canHandlers[2];
bool DJIMotor::sendDebug = false;
bool DJIMotor::feedbackDebug = false;

DJIMotor::DJIMotor(bool isErroneousMotor){

    motorNumber = -1;
    canBus = CANHandler::NOBUS;
    value = 0;
    mode = OFF;

    conflict = isErroneousMotor;
    if(isErroneousMotor)
        mode = ERR;

    powerOut = 0;
}

DJIMotor::DJIMotor(short canID, CANHandler::CANBus bus, motorType mType){

    motorNumber = (short)(canID - 1);
    canBus = bus;
    type = mType;

    value = 0;

    if(type == GM6020){
        motorNumber += 4;
        pidSpeed.setPID(defautlGimblySpeedSettings[0],defautlGimblySpeedSettings[1],defautlGimblySpeedSettings[2]);
        pidSpeed.setOutputCap(defautlGimblySpeedSettings[3]);
        pidSpeed.setIntegralCap(defautlGimblySpeedSettings[4]);

        pidPosition.setPID(defaultGimblyPosSettings[0],defaultGimblyPosSettings[1],defaultGimblyPosSettings[2]);
        pidPosition.setOutputCap(defaultGimblyPosSettings[3]);
        pidPosition.setIntegralCap(defaultGimblyPosSettings[4]);

    }else if(type == M3508){
        pidSpeed.setPID(defautM3508SpeedSettings[0],defautM3508SpeedSettings[1],defautM3508SpeedSettings[2]);
        pidSpeed.setOutputCap(defautM3508SpeedSettings[3]);
        pidSpeed.setIntegralCap(defautM3508SpeedSettings[4]);

        pidPosition.setPID(defautM3508PosSettings[0],defautM3508PosSettings[1],defautM3508PosSettings[2]);
        pidPosition.setOutputCap(defautM3508PosSettings[3]);
        pidPosition.setIntegralCap(defautM3508PosSettings[4]);
    }

    if(!motorsExist [bus][motorNumber / 4][motorNumber % 4]){
        allMotors   [bus][motorNumber / 4][motorNumber % 4] = this;
        motorsExist [bus][motorNumber / 4][motorNumber % 4] = true;

    }else{
        DJIMotor mot(true);
        allMotors[bus][motorNumber / 4][motorNumber % 4] = &mot;
        printf("[ERROR] THERES A CONFLICT ON BUS [%d] ID [%d]. YOU WILL HAVE ERRORS.\n", motorNumber/4, motorNumber%4);
    }

    if (canID > 8 || canID < 1)
        printf("[ERROR] The canID [%d] not within correct bounds\n", canID);

    powerOut = 0;
}

DJIMotor::~DJIMotor(){
    mode = OFF;
    motorsExist[canBus][motorNumber/4][motorNumber%4] = false;
    motorNumber = -1;
    canBus = CANHandler::NOBUS;
}

void DJIMotor::setCANHandlers(CANHandler* bus_1, CANHandler* bus_2, bool threadSend, bool threadFeedback){
    canHandlers[0] = bus_1;
    canHandlers[1] = bus_2;

    // if(thread){
    //     //motorupdatethread.start(tickThread);
    //     motorSendThread.start(sendThread);
    //     motorFeedbackThread.start(feedbackThread);

    //     // canHandlers[0]->attach(&getFeedback);
    //     // canHandlers[1]->attach(&getFeedback);

    //     // CAN *can1, *can2;
    //     // canHandlers[0]->getCAN(can1);
    //     // canHandlers[1]->getCAN(can2);
    //     // can1->attach(&getFeedback);
    //     // can2->attach(&getFeedback);

    //     // canHandlers[0]->can.attach(&getFeedback);
    //     // canHandlers[1]->can.attach(&getFeedback);
    // }

    if(threadSend)
        motorSendThread.start(sendThread);

    if(threadFeedback)
        motorFeedbackThread.start(feedbackThread);
}

void DJIMotor::setValue(int val){
    value = val;
}

void DJIMotor::setPower(int power){
    setValue(power);
    mode = POW;
    setOutput();
}

void DJIMotor::setSpeed(int speed){
    setValue(speed);
    mode = SPD;
    setOutput();
}

void DJIMotor::setPosition(int position){
    setValue(position);
    mode = POS;
    setOutput();
}

void DJIMotor::operator=(int value){
    setValue(value);
}

void DJIMotor::setOutput(){
    unsigned long time = us_ticker_read() / 1000;

    if(mode == POW)
        powerOut = (int16_t)value;

    else if(mode == SPD)
        powerOut = (int16_t)pidSpeed.calculate((float)value, (float)getData(VELOCITY), float(time - lastTime));

    else if(mode == POS)
        if(!justPosError)
            if(!useAbsEncoder)
                powerOut = (int16_t)pidSpeed.calculate(pidPosition.calculate((float)value, (float)getData(MULTITURNANGLE), (float)(time - lastTime)), (float)getData(VELOCITY), (float)(time - lastTime));
            else
                powerOut = (int16_t)pidSpeed.calculate(pidPosition.calculate((float)value, (float)getData(ANGLE), (float)(time - lastTime)), (float)getData(VELOCITY), (float)(time - lastTime));

        else
            if(!useAbsEncoder)
                powerOut = (int16_t)pidPosition.calculate((float)value, (float)getData(MULTITURNANGLE), (float)(time - lastTime));
            else
                powerOut = (int16_t)pidPosition.calculate((float)value, (float)getData(ANGLE), (float)(time - lastTime));

    else if(mode == OFF)
        powerOut = 0;

    else if(mode == ERR)
        printf("[ERROR] THIS IS AN ERRONEOUS MOTOR. DO NOT ATTEMPT TO SEND IT DATA, DO NOT PASS GO, DO NOT COLLECT $200, FIX THIS!\n");

    if(powerOut > outCap)
        powerOut = (int16_t)outCap;

    else if(powerOut < -outCap)
        powerOut = (int16_t) - outCap;

    lastTime = time;
}

__attribute__((unused)) int DJIMotor::getPowerOut() const {
    return powerOut;
}

int DJIMotor::getData(motorDataType data) {
    if(data == POWEROUT)
        return powerOut;

    else if (data != MULTITURNANGLE)
        return motorData[data];

    else if(data == MULTITURNANGLE)
        return multiTurn;

    return 0;
}

int DJIMotor::operator>>(motorDataType data){
    return getData(data);
}

void DJIMotor::printAllMotorData() {
    printf("ANGL:%d MLTI:%d VELO:%d TORQ:%d TEMP:%d | PWR:%d\n", getData(ANGLE), getData(MULTITURNANGLE), getData(VELOCITY), getData(TORQUE), getData(TEMPERATURE), powerOut);
}

void DJIMotor::updateMultiTurnPosition() {
    int Threshold = 3000; // From 0 to 8191
    int curAngle, lastAngle, deltaAngle, speed;
    DJIMotor *curMotor;

    for(int x = 0; x < CAN_HANDLER_NUMBER; x++)
        for (int y = 0; y < 3; y++)
            for (int z = 0; z < 4; z++)
                if (motorsExist[x][y][z]) {
                    curMotor = allMotors[x][y][z];

                    curAngle = curMotor->getData(ANGLE);
                    lastAngle = curMotor->lastMotorAngle;
                    speed = curMotor->getData(VELOCITY);
                    deltaAngle = curAngle - lastAngle;

                    if (abs(speed) < 100)
                        if (curAngle > (8191 - Threshold) && lastAngle < Threshold)
                            curMotor->multiTurn -= deltaAngle - 8191;
                        else if (curAngle < Threshold && lastAngle > (8191 - Threshold))
                            curMotor->multiTurn += deltaAngle + 8191;
                        else
                            curMotor->multiTurn += deltaAngle;

                    else
                        if (speed < 0 && deltaAngle > 0)                                // neg skip
                            curMotor->multiTurn += deltaAngle - 8191;
                        else if (speed > 0 && deltaAngle < 0)                           // pos skip
                            curMotor->multiTurn += deltaAngle + 8191;
                        else
                            curMotor->multiTurn += deltaAngle;

                    curMotor->lastMotorAngle = curAngle;
                }

}

void DJIMotor::sendOneID(CANHandler::CANBus bus, short sendIDindex, bool debug){
    int8_t bytes[8]  = {0,0,0,0,0,0,0,0};

    if(debug)
        printf("0x%x:\t",sendIDs[sendIDindex]);

    for(int i = 0; i < 4; i++){
        if(motorsExist[bus][sendIDindex][i]){
            allMotors[bus][sendIDindex][i]->setOutput();
            int16_t pO = allMotors[bus][sendIDindex][i]->powerOut;

            if(debug)
                printf("%d\t",pO);

            bytes[2*i] = int8_t(pO >> 8);
            bytes[2*i + 1] = int8_t(pO);

        }else
            if(debug) printf("NA\t");
    }
    //if(debug) printf("\n");
    //printf("0x%x:\t",sendIDs[sendIDindex]);
    //printArray(bytes, 8);
    //printf("meh1%d, meh2%d\n",canHandlers[0]->exists,canHandlers[1]->exists);
    //printf("canhandler id is%d\n",canHandlers[bus]);

    if(canHandlers[bus]->exists)
        canHandlers[bus]->rawSend(sendIDs[sendIDindex], bytes);

    else
        printf("[ERROR] YOUR CANHANDLERS ARE NOT DEFINED YET. DO THIS BEFORE YOU CALL ANY MOTORS,\n USING [(DJIMotor::setCANHandlers(PA_11,PA_12,PB_12,PB_13)], WHERE PA_11, PA_12 ARE TX, RX\n");

}

void DJIMotor::getFeedback(){

    for(int i = 0; i < CAN_HANDLER_NUMBER; i ++){
        uint8_t recievedBytes[8] = {0,0,0,0,0,0,0,0};
        int msgID;

        if(canHandlers[i]->getFeedback(&msgID,recievedBytes)) {
            int mNum = msgID - 0x201;

            // Debugging motorsExist array
            // for(int o = 0; o < 3; o++){
            //     for(int p = 0; p < 4; p++){
            //         printf("%d ",motorsExist[i][o][p]);
            //     }
            //     printf("\n");(int16_t)
            // }

            if(motorsExist[i][mNum/4][mNum%4]){
                allMotors[i][mNum/4][mNum%4] -> motorData[ANGLE]        = (int16_t)(recievedBytes[0] << 8 | recievedBytes[1]);
                allMotors[i][mNum/4][mNum%4] -> motorData[VELOCITY]     = (int16_t)(recievedBytes[2] << 8 | recievedBytes[3]);
                allMotors[i][mNum/4][mNum%4] -> motorData[TORQUE]       = (int16_t)(recievedBytes[4] << 8 | recievedBytes[5]);
                allMotors[i][mNum/4][mNum%4] -> motorData[TEMPERATURE]  = (int16_t) recievedBytes[6];

                if(feedbackDebug)
                    allMotors[i][mNum/4][mNum%4] -> printAllMotorData();

                allMotors [i][mNum/4][mNum%4] -> timeSinceLastFeedback = us_ticker_read() / 1000 - allMotors[i][mNum/4][mNum%4]->timeOfLastFeedback;
                allMotors [i][mNum/4][mNum%4] -> timeOfLastFeedback = us_ticker_read() / 1000;
                lastCalled[i][mNum/4][mNum%4] = us_ticker_read() / 1000;

                if(allMotors[i][mNum/4][mNum%4]->motorData[TEMPERATURE] > 40)
                    printf("[WARNING] YOU HAVE A MOTOR [0x%x] ATTACHED THAT IS %d DEGREES CELSIUS\n",msgID,allMotors[i][mNum/4][mNum%4]->motorData[TEMPERATURE]);

            }else{
                // printf("[WARNING] YOU HAVE A MOTOR [0x%x] {%d}{%d} ATTACHED THAT IS NOT INITIALIZED.. WHY: \n",msgID,mNum/4,mNum%4);
            }
        }
    }
    updateMultiTurnPosition();
}

__attribute__((unused)) bool DJIMotor::checkConnection(bool debug){
    for(int bus = 0; bus < CAN_HANDLER_NUMBER; bus++)
        for(int c = 0; c < 4; c++)
            for(int r = 0; r < 3; r++)
                if(motorsExist[bus][r][c]) {
                    if (us_ticker_read() / 1000 - lastCalled[bus][r][c] > TIMEOUT_MS) {
                        if(debug) {
                            printf("Motor %d on bus: %d lost connection\n", c + 4 * r + 1, bus + 1);
                            printf("Motor[%d][%d][%d]\n", bus, r, c);
                        }
                        return false;
                    }
                }
    return true;
}

__attribute__((unused)) void DJIMotor::zeroPos(){
    multiTurn = 0;
}

/**
* @brief the thread that runs the ever-necessary Motor::tick()
*/

__attribute__((unused)) void DJIMotor::tickThread() {
    while (true) {
        tick();
        ThisThread::sleep_for(1ms);
    }
}

/**
* @brief the thread that runs the ever-necessary Motor::tick()
*/

void DJIMotor::feedbackThread() {
    while (true) {
        getFeedback();
        ThisThread::sleep_for(1ms);
    }
}

/**
* @brief the thread that runs the ever-necessary Motor::tick()
*/

void DJIMotor::sendThread() {
    while (true) {
        sendValues();
        ThisThread::sleep_for(1ms);
    }
}

//TODO: MAKE THIS MORE EFFICIENT BY LIMITING ADDRESSES TO ONLY THOSE THAT HOLD MOTORS
void DJIMotor::sendValues() {
    for(short i = 0; i < 3; i ++)
        sendOneID(CANHandler::CANBUS_1, i, sendDebug);
    if(sendDebug) printf("\n");
    for(short i = 0; i < 3; i ++)
        sendOneID(CANHandler::CANBUS_2, i, sendDebug);
    if(sendDebug) printf("\n");
}

void DJIMotor::tick(){
    getFeedback();
    //updateMultiTurnPosition();
    sendValues();
}

#pragma clang diagnostic pop

//void DJIMotor::printChunk(CANHandler::CANBus bus, short sendID, motorDataType data){
//    printf("Bus:");
//    if(bus == CANHandler::CANBUS_1)
//        printf("BUS_1 |");
//    else if(bus == CANHandler::CANBUS_2)
//        printf("BUS_2 |");
//    printf(" sendID:0x%x ",sendIDs[sendID]);
//    for(int i = 0; i < 4; i ++){
//        if(motorsExist[bus][sendID][i])
//            if(data == POWEROUT)
//                printf("%d ",allMotors[bus][sendID][i]->powerOut);
//            else
//                printf("%d ",allMotors[bus][sendID][i]->getData(data));
//        else
//            printf("NA ");
//    }
//    printf("\n");
//}

// static void DJIMotor::setCANHandlers(PinName can1Tx, PinName can1Rx, PinName can2Tx, PinName can2Rx){
//     canHandlers[0].updateCANs(PinName canRx, PinName canTx);
//     canHandlers[1]* = &can2;
//     for(int i = 0; i < 3; i ++){
//         for(int j = 0; j < 4; j++){
//             for(int k = 0; k < 2; k++){
//                 CANMotor m;
//                 allMotors[k][i][j] = &m;
//             }
//         }
//     }
// }
