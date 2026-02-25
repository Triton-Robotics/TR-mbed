//
// Created by moksh on 12/24/25.
//

#include "PIDTools.h"

#include <iostream>
#include <ostream>

#include "util/motor/DJIMotor.h"

PIDTools::PIDTools(DJIMotor* motor, int des_val, TYPE type)
    :motor(motor), des_val(des_val), type(type) {}

float     PIDTools::initial_value = 0.0f;
uint32_t  PIDTools::start_time = 0;
uint32_t  PIDTools::start_settle_time = 0;
uint32_t  PIDTools::time_in_range = 0;


void PIDTools::test_velocity()
{
    if (running)
    {
        if (calc_overshoot)
        {
            motor->setSpeed(des_val);
            calculateOvershootVelocity();
        }
        if (calc_settling_time)
        {
            motor->setSpeed(des_val);
            calculateSettlingTimeVelocity();
        }
        if (print_settling_time && print_overshoot)
        {
           printf("Results: \n");
            printf("Overshoot: %.2f degrees\n", overshoot);
            printf("Rise Time: %lu seconds\n", rise_time);
            printf("Settling Time: %lu seconds\n", settling_time);
            running = false;
        }
    }
}

void PIDTools::test_position()
{
    if (running)
    {
        if (calc_overshoot)
        {
            motor->setPosition(des_val);
            calculateOvershootVelocity();
        }
        if (calc_settling_time)
        {
            motor->setPosition(des_val);
            calculateSettlingTimeVelocity();
        }
        if (print_settling_time && print_overshoot)
        {
            printf("Results: \n");
            printf("Overshoot: %.2f degrees\n", overshoot);
            printf("Rise Time: %lu seconds\n", rise_time);
            printf("Settling Time: %lu seconds\n", settling_time);
            running = false;
        }
    }
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
    if(!running){
        motor->setPower(0);
    }
    return "Testing complete";
}


void PIDTools::calculateOvershootVelocity()
{
    current_value = motor->getData(VELOCITY);
    if (abs(current_value) >= abs(max_value))
    {
        max_value = current_value;
        // overshoot = abs(max_value - des_val);
        // printf("Overshoot: %.2f degrees\n", overshoot); 
    }

    if (current_value >= des_val && calc_rise_time)
    {
        passed_des = true;
        end_time = us_ticker_read();
        rise_time = end_time - start_time;
        calc_rise_time = false;

    }

    if (abs(current_value) < abs(max_value) && passed_des)
    {
        overshoot = abs(max_value - des_val);
        calc_overshoot = false;
        print_overshoot = true;
    }
}

void PIDTools::calculateOvershootPosition()
{
    current_value = motor->getData(VELOCITY);
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

    if (abs(current_value) >= abs(max_value))
    {
        max_value = current_value;
        // overshoot = abs(max_value - des_val);
        // printf("Overshoot: %.2f degrees\n", overshoot);
    }

    if (current_value >= des_val && calc_rise_time)
    {
        passed_des = true;
        end_time = us_ticker_read();
        rise_time = end_time - start_time;
        calc_rise_time = false;
    }

    if (abs(current_value) < abs(max_value) && passed_des)
    {
        calc_overshoot = false;
        overshoot = abs(max_value - des_val);
        print_overshoot = true;
    }
}

void PIDTools::calculateSettlingTimeVelocity()
{

    float current_value = motor->getData(VELOCITY);
    auto now = us_ticker_read();
    if (!(current_value >= lower_bound && current_value <= upper_bound))
    { 
        time_in_range = us_ticker_read();
    }

    if (now - time_in_range >= 1000000 && calc_settling_time)
    {
        settling_time = now-start_time;
        calc_settling_time = false;
        print_settling_time = true;
    }

}

void PIDTools::calculateSettlingTimePosition()
{
    float current_value = motor->getData(ANGLE) - initial_value;
    auto now = us_ticker_read();
    if (!(current_value >= lower_bound && current_value <= upper_bound))
    {
        time_in_range = us_ticker_read();
    }

    if (now - time_in_range >= 1000000 && calc_settling_time)
    {
        settling_time = now-start_time;
        calc_settling_time = false;
        print_settling_time = true;
    }
}
