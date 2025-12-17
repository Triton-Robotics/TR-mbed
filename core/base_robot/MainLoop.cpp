#include "base_robot/MainLoop.h"

void main_loop(BaseRobot &base_robot) {
    unsigned long loop_clock_us = us_ticker_read();
    unsigned long prev_loop_time_us = loop_clock_us;
    unsigned long last_periodic_call_us = 0;

    unsigned long main_loop_dt_ms = base_robot.main_loop_dt_ms();

    base_robot.init();

    while (true) {
        // TODO > vs >=
        if ((loop_clock_us - prev_loop_time_us) / 1000 >= main_loop_dt_ms) {

            base_robot.periodic(loop_clock_us - last_periodic_call_us);
            // TODO make sure this periodic dt logic is correct
            last_periodic_call_us = us_ticker_read();
        }
    }
}