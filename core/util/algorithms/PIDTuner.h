#ifndef TR_EMBEDDED_PIDTUNER_H
#define TR_EMBEDDED_PIDTUNER_H
#include "../motor/DJIMotor.h"
#include <random>
#include <chrono>
#include <vector>

using fitnessFunction = std::function<double(double)>;

struct PIDDataSet{
    double kP = 0;
    double kI = 0;
    double kD = 0;

    double meanIntegralAbsError = 0;
    double meanIntegralSqError = 0;

    void dumpPIDs() const{
        printf("kP: %lf kI: %lf kD: %lf\n", kP, kI, kD);
    }

    void dumpIntegrals() const{
        printf("IAE: %lf ISE: %lf\n", meanIntegralAbsError, meanIntegralSqError);
    }

    void dumpData() const{
        printf("kP: %lf kI: %lf kD: %lf\n", kP, kI, kD);
        printf("IAE: %lf ISE: %lf\n\n", meanIntegralAbsError, meanIntegralSqError);
    }
};

struct PIDRanges{
    double kPMin;
    double kPMax;
    double kIMin;
    double kIMax;
    double kDMin;
    double kDMax;
};

class PIDTuner {
public:
    PIDTuner(DJIMotor &motor_, PIDDataSet population[], size_t populationSize, PIDRanges ranges = {0, 30, 0, 10, 0, 1000});

    /*
     * getFitness can be any function that converts the integralAbsError to fitness
     * e.g., a/x, a^-x, a - x
     */
    void tune(unsigned long time_s, size_t iterations, size_t generations, std::vector<PIDDataSet> &best, double mutationRate = 0.01, fitnessFunction getFitness = [](double integral){return 1 / integral;});

    ~PIDTuner() = default;

    double getRandomNumber(double min, double max);

private:
    DJIMotor &motor;
    fitnessFunction getFitness;

    size_t populationSize;
    PIDDataSet *population;

    double mutationRate;

    std::random_device device;
    std::mt19937 mt;

    void randomizePosition();
    void getProbabilities(double probabilities[]);
    PIDDataSet crossover(PIDDataSet &parent1, PIDDataSet &parent2);
    PIDDataSet select(const double probabilities[]);
    void mutate(PIDDataSet &child);
    void nextGeneration();
};



#endif //TR_EMBEDDED_PIDTUNER_H
