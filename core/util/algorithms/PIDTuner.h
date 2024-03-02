#ifndef TR_EMBEDDED_PIDTUNER_H
#define TR_EMBEDDED_PIDTUNER_H
#include "../motor/DJIMotor.h"
#include <random>

using fitnessFunction = std::function<double(double)>;

struct PIDDataSet{
    float kP = 0;
    float kI = 0;
    float kD = 0;

    double meanIntegralAbsError = 0;
    double meanIntegralSqError = 0;

    void dumpData() const{
        printf("kP: %f kI: %f kD: %f\n", kP, kI, kD);
        printf("IAE: %lf ISE: %lf\n\n", meanIntegralAbsError, meanIntegralSqError);
    }
};

struct PIDRanges{
    float kPMin;
    float kPMax;
    float kIMin;
    float kIMax;
    float kDMin;
    float kDMax;
};

class PIDTuner {
public:
    PIDTuner(DJIMotor &motor_, PIDDataSet population[], size_t len, PIDRanges ranges = {0, 30, 0, 10, 0, 1000});
    void tune(PIDDataSet population[], size_t len, unsigned long time_s, size_t iterations, size_t generations);

    ~PIDTuner() = default;

private:
    DJIMotor &motor;

    static float getRandomNumber(float min, float max);
    void randomizePosition();
    void crossover(bool sexual = true);

};



#endif //TR_EMBEDDED_PIDTUNER_H
