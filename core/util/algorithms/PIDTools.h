//
// Created by moksh on 12/24/25.
//

#ifndef TR_EMBEDDED_PIDTOOLS_H
#define TR_EMBEDDED_PIDTOOLS_H
#include "motor/DJIMotor.h"

enum class TYPE
{
    VELOCITY,
    POSITION
};

class PIDTools
{
    public:
    // constructor
    PIDTools(DJIMotor *motor, int des_val, TYPE type = TYPE::VELOCITY);

    void test_velocity();
    void test_position();
    std::string run();

    private:
    DJIMotor *motor;
    int des_val;
    TYPE type;

    float calculateOvershootPosition(int des_val);
    unsigned long calculateRiseTimePosition(int des_val);
    unsigned long calculateSettlingTimePosition(int des_val);

    float calculateOvershootVelocity(int des_val);
    unsigned long calculateRiseTimeVelocity(int des_val);
    unsigned long calculateSettlingTimeVelocity(int des_val);

};

#endif //TR_EMBEDDED_PIDTOOLS_H