#include "base_robot/BaseRobot.h"
#include "mbed.h"

void BaseRobot::main_loop()
{
    unsigned long loop_clock_us = us_ticker_read();
    unsigned long prev_loop_time_us = loop_clock_us;

    unsigned long main_loop_dt_ms = 1; // idk how to call main_loop_dt_ms() without erroring for some reason

    // Init all constants, subsystems, sensors, IO, etc.
    init();

    while (true)
    {
        // TODO: StmIO comms (Ref and Jetson)

        loop_clock_us = us_ticker_read();
        if ((loop_clock_us - prev_loop_time_us) / 1000 >= main_loop_dt_ms)
        {
            // Add subsystems in periodic
            periodic(loop_clock_us - prev_loop_time_us);
            prev_loop_time_us = us_ticker_read();
        }
        // Add sensors updates in your end of loop
        end_of_loop();

        // TODO: Motor updates
    }
}