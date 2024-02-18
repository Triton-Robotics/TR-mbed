#include "PIDTuner.h"

PIDTuner::PIDTuner(DJIMotor &motor_) : motor{motor_}{
    motor.useAbsEncoder = true;
}

void PIDTuner::tune(PIDDataSet dataSet[], int len, unsigned long time_s, int iterations){
    kP = 1;
    kI = 0;
    kD = 0;

    printf("tuning %s...\n", motor.name.c_str());

    for(int i = 0; i < len; i++){
        motor.pidPosition.resetPID(kP, kI, kD);

        dataSet[i].kP = kP;
        dataSet[i].kI = kI;
        dataSet[i].kD = kD;

        DJIMotor::s_sendValues();
        DJIMotor::s_getFeedback();

        dataSet[i].startError = DJIMotor::s_calculateDeltaPhase(0, motor.getData(ANGLE));
        dataSet[i].maxError = dataSet[i].startError;
        dataSet[i].minError = dataSet[i].startError;

        double lastError_ = dataSet[i].startError;
        double integralAbsErrorSum_ = 0;
        double integralSqErrorSum_ = 0;

        for(int j = 0; j < iterations; j++) {

            unsigned long timeStart;
            unsigned long loopTimer = us_ticker_read();
            unsigned long start_u = us_ticker_read();
            unsigned long current_u = us_ticker_read();

            while ((current_u - start_u) / 1000 < (time_s) * 1000) {
                timeStart = us_ticker_read();

                if ((timeStart - loopTimer) / 1000 > 25) {

                    // -----------------------------------------
                    motor.setPosition(0);

                    double error = DJIMotor::s_calculateDeltaPhase(0, motor.getData(ANGLE));
                    double dt_m = static_cast<double>(us_ticker_read() - current_u) / 1000;


                    integralAbsErrorSum_ += 0.01 * abs(dt_m * (error + lastError_) / 2);
                    integralSqErrorSum_ += 0.01 * dt_m * pow(((error + lastError_) / 2), 2);

                    lastError_ = error;

                    current_u = us_ticker_read();
                    // -----------------------------------------

                    loopTimer = timeStart;
                    DJIMotor::s_sendValues();
                }
                DJIMotor::s_getFeedback();
                ThisThread::sleep_for(1ms);
            }

            int power = 2 * static_cast<int>((static_cast<float>(rand() - RAND_MAX / 2) / RAND_MAX) * static_cast<float>(motor.type == GIMBLY ? INT16_T_MAX : INT15_T_MAX));

            printf("power: %d %d\n", power, RAND_MAX);
            motor.setPower(power);
            DJIMotor::s_sendValues();
            ThisThread::sleep_for(250ms);

            motor.setPower(0);
            DJIMotor::s_sendValues();
            ThisThread::sleep_for(700ms);
        }

        dataSet[i].meanIntegralAbsError = integralAbsErrorSum_ / iterations;
        dataSet[i].meanIntegralSqError = integralSqErrorSum_ / iterations;

        kP += 10;
        kI += 0.1;
        kD += 10;
    }
}