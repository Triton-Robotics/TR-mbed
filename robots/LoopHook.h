#pragma once

#include <functional>

namespace TR
{
    struct look_hooks
    {
        std::function<void()> init;

        // Instead of having a callback for each subsystem we should just have
        // 1. init
        // 2. post IO (guarantees ref and comms/jetson reads are completed)
        // 3. end of loop (for printing/debugging etc. )
        std::function<void()> turret_executor;
        std::function<void()> chassis_executor;
        std::function<void()> shooter_executor;
        std::function<void()> end_of_loop;
        std::function<void()> print_rate_limited;
    };
}