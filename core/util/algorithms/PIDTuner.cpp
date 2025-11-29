#include "PIDTuner.h"

PIDTuner::PIDTuner(DJIMotor &motor_, PIDDataSet population_[], size_t populationSize_, PIDRanges ranges)
 : motor{motor_}, populationSize {populationSize_}, population {population_}, mutationRate{0.01}, mt(static_cast<unsigned>(std::chrono::system_clock::now().time_since_epoch().count()) + device())
{
    motor.useAbsEncoder = true;

    for(int i = 0; i < populationSize; i++){
        population[i].kP = getRandomNumber(ranges.kPMin, ranges.kPMax);
        population[i].kI = getRandomNumber(ranges.kIMin, ranges.kIMax);
        population[i].kD = getRandomNumber(ranges.kDMin, ranges.kDMax);
    }

    for(int i = 0; i < populationSize; i++)
        population[i].dumpPIDs();
}

void PIDTuner::tune(unsigned long time_s, size_t iterations, size_t generations, std::vector<PIDDataSet> &best, double mutationRate, fitnessFunction getFitness){
    printf("tuning %s...\n", motor.name.c_str());
    this -> getFitness = std::move(getFitness);
    this -> mutationRate = mutationRate;

    for(int generation = 0; generation < generations; generation++) {
        for (int i = 0; i < populationSize; i++) {
            printf("%d: ", i);
            population[i].dumpPIDs();
            motor.pidPosition.resetPID(static_cast<float>(population[i].kP), static_cast<float>(population[i].kI), static_cast<float>(population[i].kD));

            DJIMotor::s_sendValues();
            DJIMotor::DEPRECATEDs_getFeedback();

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
                    DJIMotor::DEPRECATEDs_getFeedback();
                    ThisThread::sleep_for(1ms);
                }
                randomizePosition();
                motor.pidPosition.resetErrorIntegral();
            }
            population[i].meanIntegralAbsError = integralAbsErrorSum_ / iterations;
            population[i].meanIntegralSqError = integralSqErrorSum_ / iterations;
            population[i].dumpIntegrals();

            if(population[i].meanIntegralAbsError < best.back().meanIntegralAbsError)
                best.emplace_back(population[i]);
        }
        for(auto e: best)
            e.dumpData();

        nextGeneration();
    }
}

double PIDTuner::getRandomNumber(double min, double max){
    std::uniform_real_distribution<double> dist(min, max);
    double num = dist(mt);

//    printf("%lf, %lf, %lf\n", num, min, max);
    return num;
}

void PIDTuner::randomizePosition() {
    float max = static_cast<float>(motor.type == GIMBLY ? INT16_T_MAX : INT15_T_MAX);
    int power = static_cast<int>(PIDTuner::getRandomNumber(-max, max));

    motor.setPower(power);
    DJIMotor::s_sendValues();
    ThisThread::sleep_for(250ms);

    motor.setPower(0);
    DJIMotor::s_sendValues();
    ThisThread::sleep_for(700ms);
}

void PIDTuner::getProbabilities(double probabilities[]) {
    double sum = 0;

    for(int i = 0; i < populationSize; i++){
        probabilities[i] = getFitness(population[i].meanIntegralAbsError);
        sum += probabilities[i];
    }

    for(int i = 0; i < populationSize; i++) {
        probabilities[i] /= sum;
        printf("%lf\n", probabilities[i]);
    }

}

PIDDataSet PIDTuner::crossover(PIDDataSet &parent1, PIDDataSet &parent2) {
    PIDDataSet child;
    double weight1 = getRandomNumber(0, 1);
    double weight2 = 1 - weight1;

    child.kP = (parent1.kP * weight1) + (parent2.kP * weight2);
    child.kI = (parent1.kI * weight1) + (parent2.kI * weight2);
    child.kD = (parent1.kD * weight1) + (parent2.kD * weight2);

    return child;
}

PIDDataSet PIDTuner::select(const double probabilities[]) {
    double r = getRandomNumber(0, 1);
    double cumulativeProbability = 0.0;

    for (int i = 0; i < populationSize; ++i) {
        cumulativeProbability += probabilities[i];
        if (r <= cumulativeProbability) {
            return population[i];
        }
    }

    return population[populationSize - 1];
}

void PIDTuner::mutate(PIDDataSet &child){

    double r = getRandomNumber(0.0, 1.0);
    if(r <= mutationRate)
        child.kP += getRandomNumber(-3, 3);

    r = getRandomNumber(0.0, 1.0);
    if(r <= mutationRate)
        child.kI += getRandomNumber(-0.0001, 0.0001);

    r = getRandomNumber(0.0, 1.0);
    if(r <= mutationRate)
        child.kD += getRandomNumber(-10, 10);
}

void PIDTuner::nextGeneration() {
    double probabilities[populationSize];
    PIDDataSet children[populationSize];
    getProbabilities(probabilities);

    for(int i = 0; i < populationSize; i++){
        PIDDataSet parent1 = select(probabilities);
        PIDDataSet parent2 = select(probabilities);

        PIDDataSet child = crossover(parent1, parent2);
        mutate(child);

        children[i] = child;
    }

    for(int i = 0; i < populationSize; i++)
        population[i] = children[i];
}
