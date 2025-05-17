// Test yaw or pitch motor
#include "main.h"
#include <cmath>
#include <functional>

// Testing Parameters
bool pitch = false, yaw = true;
bool position = false, velocity = true;

// Sentry turret constraints
#define PITCH_MIN 2400 // TODO
#define PITCH_MAX 300 // TODO
#define YAW_MIN 0
#define YAW_MAX 8191
#define MAX_POWER 32767 // GM6020

DigitalOut led(L26);
DigitalOut led2(L27);
DigitalOut led3(L25);

I2C i2c(I2C_SDA, I2C_SCL);

//int pitchPos = PITCH_MIN, yawPos = YAW_MIN;
int ry = 0, lx = 0, ly = 0, rx = 0, yawPower = 0;
bool converged, steadyState;
int pitchRange, pitchCenter, nextStartingPitch;
std::vector<unsigned long> convVec_u;

int main(){
    DJIMotor pitchMot(5, CANHandler::CANBUS_2, GM6020, "pitch_motor");
    DJIMotor yawMot1 (6, CANHandler::CANBUS_1, GM6020, "yaw_motor_1");
    DJIMotor yawMot2 (7, CANHandler::CANBUS_1, GM6020, "yaw_motor_2");

    yawMot1.setSpeedPID(922.9095, 0.51424, 0);
    yawMot1.setSpeedIntegralCap(1000);
    // TODO pitch pos PID and yaw1 pos PID, yaw2 should mirror yaw1

    std::reference_wrapper<DJIMotor> mot { pitchMot };
    std::reference_wrapper<DJIMotor> mot2{ pitchMot };

    if (pitch) {
        mot = pitchMot;
        mot2 = pitchMot; // unused
    }
    else if (yaw) {
        mot  = yawMot1;
        mot2 = yawMot2;
    }
    DJIMotor& testMot  = mot;
    DJIMotor& testMot2 = mot2;

    //assigning can handler objects to motor class.
    DJIMotor::s_setCANHandlers(&canHandler1,&canHandler2, false, false);

    //getting initial feedback.
    DJIMotor::s_getFeedback();

    unsigned long loopTimer_u = us_ticker_read();
    unsigned long timeEnd_u, timeStart_u, timeTest_u = 0;

    int refLoop = 0, desiredOutput = 0;

    while(true) { //main loop
        timeStart_u = us_ticker_read();

        //inner loop runs every 25ms
        if ((timeStart_u - loopTimer_u) / 1000 > 15) {
            loopTimer_u = timeStart_u;
            led = !led; //led blink tells us how fast the inner loop is running

            remoteRead(); //reading data from remote
            bool switL = (remote.leftSwitch() == Remote::SwitchState::UP);
            bool switLDown = (remote.leftSwitch() == Remote::SwitchState::DOWN);

            lx = 0;
            ly = 0;
            rx = 0;
            ry = 0;

            if (abs(remote.rightX()) > 35) {
                rx = remote.rightX();
            }
            if (abs(remote.rightY()) > 35) {
                ry = remote.rightY();
            }
            if (abs(remote.leftX()) > 35) {
                lx = remote.leftX();
            }
            if (abs(remote.leftY()) > 35) {
                ly = remote.leftY();
            }

            if (switL) { // manual control
                desiredOutput = ry / 10;
                if (position)
                    testMot.setPosition(desiredOutput);
                else if (velocity)
                    testMot.setSpeed(desiredOutput);
                int powerOut = testMot.powerOut; // for sentry double motor
                testMot2.setPower(powerOut);
            }

            else if (switLDown) { // PID convergence testing
                if (!timeTest_u) { // initialize test vars
                    timeTest_u = timeStart_u;
                    convVec_u = {};
                    converged = true;
                }
                if (converged && steadyState) { // set next speed once system settles
                    converged = false;
                    steadyState = false;
                    timeTest_u = timeStart_u;
                    if (yaw && velocity) {
                        switch (convVec_u.size()) {
                            case 0: // test 1a: small +delta
                                desiredOutput = 5;
                                break;
                            case 1: // test 1b: small -delta
                                desiredOutput = -5;
                                break;
                            case 2: // test 2a: medium +delta
                                desiredOutput = 15;
                                break;
                            case 3: // test 2b: medium -delta
                                desiredOutput = -15;
                                break;
                            case 4: // test 3a: large +delta
                                desiredOutput = 50;
                                break;
                            case 5: // test 3b: large -delta
                                desiredOutput = -50;
                                break;
                            default: // print test results
                                for (int i = 0; i < convVec_u.size(); i++) {
                                    printff("%lu\t", convVec_u[i]);
                                }
                                printff("\n");
                        }
                    }
                    else if (yaw && position) {
                        int curPos = testMot >> ANGLE;
                        switch (convVec_u.size()) {
                            case 0: // test 1a: small +delta
                                desiredOutput = (curPos + YAW_MAX / 10) % YAW_MAX;
                                break;
                            case 1: // test 1b: small -delta
                                desiredOutput = (curPos - YAW_MAX / 10) % YAW_MAX;
                                break;
                            case 2: // test 2a: medium +delta
                                desiredOutput = (curPos + YAW_MAX / 5) % YAW_MAX;
                                break;
                            case 3: // test 2b: medium -delta
                                desiredOutput = (curPos - YAW_MAX / 5) % YAW_MAX;
                                break;
                            case 4: // test 3a: large +delta
                                desiredOutput = (curPos + YAW_MAX / 2) % YAW_MAX;
                                break;
                            case 5: // test 3b: large -delta
                                desiredOutput = (curPos - YAW_MAX / 2) % YAW_MAX;
                                break;
                            default: // print test results
                                for (int i = 0; i < convVec_u.size(); i++) {
                                    printff("%lu\t", convVec_u[i]);
                                }
                                printff("\n");
                        }
                    }
                    else if (pitch && position) {
                        pitchRange = PITCH_MAX - PITCH_MIN;
                        pitchCenter = PITCH_MIN + pitchRange / 2;
                        nextStartingPitch = pitchCenter;
                        switch (convVec_u.size()) {
                            case 0: // test 1a: small +delta
                                desiredOutput = pitchCenter + pitchRange / 8;
                                break;
                            case 1: // test 1b: small -delta
                                desiredOutput = pitchCenter - pitchRange / 8;
                                break;
                            case 2: // test 2a: medium +delta
                                desiredOutput = pitchCenter + pitchRange / 3;
                                break;
                            case 3: // test 2b: medium -delta
                                desiredOutput = pitchCenter - pitchRange / 3;
                                nextStartingPitch = PITCH_MIN;
                                break;
                            case 4: // test 3a: large +delta, starting at lowest turret point
                                desiredOutput = pitchCenter + pitchRange * 0.4;
                                nextStartingPitch = PITCH_MAX;
                                break;
                            case 5: // test 3b: large -delta, starting at highest turret point
                                desiredOutput = pitchCenter - pitchRange * 0.4;
                                break;
                            default: // print test results
                                for (int i = 0; i < convVec_u.size(); i++) {
                                    printff("%lu\t", convVec_u[i]);
                                }
                                printff("\n");
                        }
                    }
                    testMot.setSpeed(desiredOutput);
                    int powerOut = testMot.powerOut;
                    testMot2.setPower(powerOut); // match velocities on both
                }

                else if (converged) { // get to steady state before next test
                    if (velocity) { // ss: velocity = 0
                        testMot.setPower(0);
                        if (!(testMot >> VELOCITY)) {
                            steadyState = true;
                        }
                    } else if (position) { // ss: velocity = 0, position = desired
                        if (pitch) {
                            testMot.setPosition(nextStartingPitch);
                            float limit = 0.02;
                            float error = ((testMot >> ANGLE) - nextStartingPitch) / (testMot >> ANGLE);
                            if (!(testMot >> VELOCITY) && (abs(error) < limit)) {
                                steadyState = true;
                            }
                        }
                    }

                }
                else { // check if convergence reached (+-2% error)
                    float limit = 0.02;
                    if (yaw && velocity) {
                        float error = ((testMot >> VELOCITY) - desiredOutput) / (testMot >> VELOCITY);
                        if (abs(error) < limit) {
                            converged = true;
                            convVec_u.push_back(timeStart_u - timeTest_u); // store time to converge
                        }
                    }
                    else if (yaw && position) {
                        float error = ((testMot >> ANGLE) - desiredOutput) / (testMot >> ANGLE);
                        /*
                         * Velocity is in RPM, but position goes from 0-8191, so translate that into a variable
                         * tthat calculates the next position in 15 ms. If that error is also within the tolerance,
                         * consider that successful convergence.
                         */
                        float velTicks = ((float) (testMot>>VELOCITY)) * 60 * 1000/15; // velTicks: delta pos ticks per 15 ms
                        float nextPos = ((float) (testMot>>ANGLE)) + velTicks;
                        float nextError = (nextPos - desiredOutput) / nextPos; //estimated error on next read
                        if (abs(error) < limit && abs(nextError) < limit) {
                            converged = true;
                            convVec_u.push_back(timeStart_u - timeTest_u); // store time to converge
                        }
                    }
                    else if (pitch && position) {
                        float error = ((testMot >> ANGLE) - desiredOutput) / (testMot >> ANGLE);
                        /*
                         * Velocity is in RPM, but position goes from 0-8191, so translate that into a variable
                         * tthat calculates the next position in 15 ms. If that error is also within the tolerance,
                         * consider that successful convergence.
                         */
                        float velTicks = ((float) (testMot>>VELOCITY)) * 60 * 1000/15; // velTicks: delta pos ticks per 15 ms
                        float nextPos = ((float) (testMot>>ANGLE)) + velTicks;
                        float nextError = (nextPos - desiredOutput) / nextPos; //estimated error on next read
                        if (abs(error) < limit && abs(nextError) < limit) {
                            converged = true;
                            convVec_u.push_back(timeStart_u - timeTest_u); // store time to converge
                        }
                    }
                }
            }

            if (switL || switLDown) { // run PID calcs
                int dir = 0;
                if (desiredOutput > 0) {
                    dir = 1;
                } else if (desiredOutput < 0) {
                    dir = -1;
                }
                testMot.pidSpeed.feedForward = (-0.431808886065578 * pow(desiredOutput, 2)) * dir
                                               + 167.091229447692 * desiredOutput + 1643.17393278750 * dir;
            }
            else {
                testMot.setPower(0);
                testMot2.setPower(0);
                timeTest_u = 0;
            }

            timeEnd_u = us_ticker_read();

            DJIMotor::s_sendValues();

            if (refLoop >= 5) { // printing every 5 cycles
                refLoop = 0;
                refereeThread(&referee);

                int curAngle = testMot >> ANGLE;
                int curSpeed = testMot >> VELOCITY;
                int posError = abs(desiredOutput - curAngle);
                int speedError = abs(desiredOutput - curSpeed);
                int iCPos = testMot.pidPosition.iC;
                int iCSpd = testMot.pidSpeed.iC;

                if (switL || velocity)
                    printff("%d\t%d\t%d\t%d\t%d\t%d\n", speedError, desiredOutput, curSpeed, iCSpd, testMot.powerOut);
                else if (switL || position)
                    printff("%d\t%d\t%d\t%d\t%d\t%d\n", posError, desiredOutput, curAngle, iCPos, testMot.powerOut);
            }
            refLoop++;

            //FEEDBACK CODE DOES NEED TO RUN FASTER THAN 1MS
            //OTHER QUICK AND URGENT TASKS GO HERE

            DJIMotor::s_getFeedback();
            ThisThread::sleep_for(1ms);
        }
    }
}