//
// Created by moksh on 12/24/25.
//

#ifndef TR_EMBEDDED_PIDTOOLS_H
#define TR_EMBEDDED_PIDTOOLS_H
#include "motor/DJIMotor.h"
#include "util/motor/DJIMotor.h"

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

    void init()
    {
        if (type == TYPE::VELOCITY)
        {
            motor->setSpeed(des_val);
            current_value = motor->getData(VELOCITY);
            initial_value = motor->getData(VELOCITY);
            max_value = current_value;
            start_settle_time = us_ticker_read();
            time_in_range = us_ticker_read();
            start_time = us_ticker_read();
        }
        else
        {
            motor->setPosition(des_val);
            current_value = motor->getData(ANGLE);
            initial_value = motor->getData(ANGLE);
            max_value = current_value;
            start_settle_time = us_ticker_read();
            time_in_range = us_ticker_read();
            start_time = us_ticker_read();
        }
    }
    void test_velocity();
    void test_position();
    std::string run();


    private:
    DJIMotor *motor;
    int des_val;
    TYPE type;
    float current_value;
    float max_value;
    static float initial_value;
    bool print_overshoot = false;
    bool print_settling_time = false;

    float overshoot;
    unsigned long rise_time;
    unsigned long settling_time;

    bool running = true;
    bool calc_overshoot = true;
    bool calc_rise_time = true;
    bool calc_settling_time = true;
    bool passed_des = false;
    static uint32_t start_time;
    uint32_t end_time;
    static uint32_t start_settle_time;
    int lower_bound = 0.98 * (des_val - initial_value);
    int upper_bound = 1.02 * (des_val - initial_value);
    static uint32_t time_in_range;

    void calculateOvershootVelocity();
    void calculateOvershootPosition();
    void calculateSettlingTimeVelocity();
    void calculateSettlingTimePosition();

  //  float calculateOvershootPosition(int des_val);
  //  unsigned long calculateRiseTimePosition(int des_val);
  //  unsigned long calculateSettlingTimePosition(int des_val);

  //  float calculateOvershootVelocity(int des_val);
  //  unsigned long calculateRiseTimeVelocity(int des_val);
  //  unsigned long calculateSettlingTimeVelocity(int des_val);

};

#endif //TR_EMBEDDED_PIDTOOLS_H