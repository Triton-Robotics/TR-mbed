#include "PIDTuner.h"

PIDTuner::PIDTuner(DJIMotor &motor_, PIDDataSet population[], size_t len, PIDRanges ranges) : motor{motor_}{
    motor.useAbsEncoder = true;

    for(int i; i < len; i++){
        population[i].kP = PIDTuner::getRandomNumber(ranges.kPMin, ranges.kPMax);
        population[i].kI = PIDTuner::getRandomNumber(ranges.kIMin, ranges.kIMax);
        population[i].kD = PIDTuner::getRandomNumber(ranges.kDMin, ranges.kDMax);
    }
}

void PIDTuner::tune(PIDDataSet population[], size_t len, unsigned long time_s, size_t iterations, size_t generations){
    printf("tuning %s...\n", motor.name.c_str());

    for(int generation = 0; generation < generations; generation++) {
        for (int i = 0; i < len; i++) {
            motor.pidPosition.resetPID(population[i].kP, population[i].kI, population[i].kD);

            DJIMotor::s_sendValues();
            DJIMotor::s_getFeedback();

            double lastError_ = DJIMotor::s_calculateDeltaPhase(0, motor.getData(ANGLE));
            double integralAbsErrorSum_ = 0;
            double integralSqErrorSum_ = 0;

            for (int j = 0; j < iterations; j++) {

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
                randomizePosition();
                motor.pidPosition.resetErrorIntegral();
            }
            population[i].meanIntegralAbsError = integralAbsErrorSum_ / iterations;
            population[i].meanIntegralSqError = integralSqErrorSum_ / iterations;
        }
        // mating
    }
}

float PIDTuner::getRandomNumber(float min, float max) {
    static std::random_device device;
    static std::mt19937 mt(device());
    static std::uniform_real_distribution<float> dist(min, max);

    return dist(mt);
}

void PIDTuner::randomizePosition() {
    float max = static_cast<float>(motor.type == GIMBLY ? INT16_T_MAX : INT15_T_MAX);
    int power = static_cast<int>(PIDTuner::getRandomNumber(-max, max));

    printf("power: %d %d\n", power, RAND_MAX);
    motor.setPower(power);
    DJIMotor::s_sendValues();
    ThisThread::sleep_for(250ms);

    motor.setPower(0);
    DJIMotor::s_sendValues();
    ThisThread::sleep_for(700ms);
}

void PIDTuner::crossover(bool sexual) {

}
