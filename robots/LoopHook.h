#pragma once

#include <functional>

namespace TR
{
    struct look_hooks
    {
        std::function<void()> init;
        std::function<void()> turret_executor;
        std::function<void()> chassis_executor;
        std::function<void()> shooter_executor;
        std::function<void()> end_of_loop;
        std::function<void()> print_rate_limited;
    };
}