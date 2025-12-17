#include "base_robot/BaseRobot.h"
#include "mbed.h"

void BaseRobot::main_loop()
{
    unsigned long loop_clock_us = us_ticker_read();
    unsigned long prev_loop_time_us = loop_clock_us;

    unsigned long main_loop_dt_ms = 1; // idk how to call main_loop_dt_ms() without erroring for some reason

    init();

    while (true)
    {
        loop_clock_us = us_ticker_read();
        if ((loop_clock_us - prev_loop_time_us) / 1000 >= main_loop_dt_ms)
        {
            periodic(loop_clock_us - prev_loop_time_us);
            prev_loop_time_us = us_ticker_read();
        }
        end_of_loop();
    }
}