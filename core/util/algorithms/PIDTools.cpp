//
// Created by moksh on 12/24/25.
//

#include "PIDTools.h"

#include <iostream>
#include <ostream>

#include "util/motor/DJIMotor.h"

PIDTools::PIDTools(DJIMotor* motor, int des_val, TYPE type)
    :motor(motor), des_val(des_val), type(type) {}

void PIDTools::test_velocity()
{
    motor->setSpeed(des_val);
    // continue
    float overshoot = calculateOvershootVelocity(des_val);
    unsigned long rise_time = calculateRiseTimeVelocity(des_val);
    unsigned long settling_time = calculateSettlingTimeVelocity(des_val);

    std::cout << "Results: " << std::endl;
    std::cout << "Overshoot: " << overshoot << "degrees" << std::endl;
    std::cout << "Rise Time: " << rise_time << "seconds" << std::endl;
    std::cout << "Settling Time: " << settling_time << "seconds" << std::endl;

}

void PIDTools::test_position()
{
    motor->setPosition(des_val);
    // continue
  //  float overshoot = calculateOvershoot(des_val);
    unsigned long rise_time = calculateRiseTimePosition(des_val);
    unsigned long settling_time = calculateSettlingTimePosition(des_val);

    std::cout << "Results: " << std::endl;
  //  std::cout << "Overshoot: " << overshoot << "degrees" << std::endl;
    std::cout << "Rise Time: " << rise_time << "seconds" << std::endl;
    std::cout << "Settling Time: " << settling_time << "seconds" << std::endl;

}

void printResults(float overshoot, float rise_time, float settling_time)
{

}

std::string PIDTools::run()
{
    if (type == TYPE::VELOCITY)
    {
        test_velocity();
    }
    else
    {
        test_position();
    }
    return "Testing complete";
}

float PIDTools::calculateOvershootVelocity(int des_val)
{
    float current_value = motor->getData(VELOCITY);
    float max_value = current_value;
    bool passed_des = false;

    while (1)
    {

        current_value = motor->getData(VELOCITY);
        if (abs(current_value) >= abs(max_value))
        {
            max_value = current_value;
        }

        if (current_value >= des_val)
        {
            passed_des = true;
        }

        if (abs(current_value) < abs(max_value) && passed_des)
        {
            break;
        }

    }

    float overshoot = abs(max_value - des_val);
    return overshoot;
}

float PIDTools::calculateOvershootPosition(int des_val)
{
    float current_value = motor->getData(ANGLE);
    float initial_value = motor->getData(ANGLE);
    float max_value = current_value;
    bool passed_des = false;

    while (1)
    {

        current_value = motor->getData(ANGLE);
        current_value -= initial_value;

        if ((current_value) > 8192)
        {
            current_value -= 8192;
        }
        else if (current_value < 0)
        {
            current_value += 8192;
        }

        if (abs(current_value) >= abs(max_value))
        {
            max_value = current_value;
        }


        if (current_value >= des_val)
        {
            passed_des = true;
        }

        if (abs(current_value) < abs(max_value) && passed_des)
        {
            break;
        }

    }
    float overshoot = abs(max_value - des_val);
    return overshoot;
}

unsigned long PIDTools::calculateRiseTimeVelocity(int des_val)
{
    auto start_time = us_ticker_read();

    while (1)
    {
        float current_value = motor->getData(VELOCITY);
        if (current_value >= des_val)
        {
            auto end_time = us_ticker_read();
            auto rise_time = end_time - start_time;
            return rise_time;
        }
    }
}

unsigned long PIDTools::calculateRiseTimePosition(int des_val)
{
    auto start_time = us_ticker_read();

    while (1)
    {
        float current_value = motor->getData(ANGLE);
        if (current_value >= des_val)
        {
            auto end_time = us_ticker_read();
            auto rise_time = end_time - start_time;
            return rise_time;
        }
    }
}


unsigned long PIDTools::calculateSettlingTimeVelocity(int des_val)
{
    int initial_value = motor->getData(VELOCITY);
    int lower_bound = 0.98 * (des_val - initial_value);
    int upper_bound = 1.02 * (des_val - initial_value);
    auto start_time = us_ticker_read();
    auto time_in_range = us_ticker_read();

    while (1)
    {
        float current_value = motor->getData(VELOCITY);
        auto now = us_ticker_read();
        if (!(current_value >= lower_bound && current_value <= upper_bound))
        {
            time_in_range = us_ticker_read();
        }

        if (now - time_in_range >= 1000000)
        {
            auto settling_time = now-start_time;
            return settling_time;
        }
    }
}

unsigned long PIDTools::calculateSettlingTimePosition(int des_val)
{
    int initial_value = motor->getData(ANGLE);
    float lower_bound = 0.98 * (des_val);
    float upper_bound = 1.02 * (des_val);
    auto start_time = us_ticker_read();
    auto time_in_range = us_ticker_read();

    while (1)
    {
        float current_value = motor->getData(ANGLE) - initial_value;
        auto now = us_ticker_read();
        if (!(current_value >= lower_bound && current_value <= upper_bound))
        {
            time_in_range = us_ticker_read();
        }

        if (now - time_in_range >= 1000000)
        {
            auto settling_time = now-start_time;
            return settling_time;
        }
    }
}


