#ifndef TR_EMBEDDED_PIDTUNER_H
#define TR_EMBEDDED_PIDTUNER_H
#include "../motor/DJIMotor.h"
#include <random>

struct PIDDataSet{
    float kP = 0;
    float kI = 0;
    float kD = 0;

    int startError = 0;
    int maxError = 0;
    int minError = 0;

    double meanIntegralAbsError = 0;
    double meanIntegralSqError = 0;

    void dumpData(){
        printf("kP: %f kI: %f kD: %f\n", kP, kI, kD);
        printf("Error start: %d max: %d min: %d\n", startError, maxError, minError);
        printf("IAE: %lf ISE: %lf\n\n", meanIntegralAbsError, meanIntegralSqError);
    }
};

class PIDTuner {
public:
    PIDTuner(DJIMotor &motor_);
    void tune(PIDDataSet dataSet[], int len, unsigned long time_s, int iterations);

    ~PIDTuner() = default;

private:
    DJIMotor &motor;

    float kP;
    float kI;
    float kD;

};



#endif //TR_EMBEDDED_PIDTUNER_H
