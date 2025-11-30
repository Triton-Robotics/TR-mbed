#pragma once

#include <functional>

namespace TR
{
    struct look_hooks
    {
        std::function<void()> init;

        std::function<void()> periodic;
        std::function<void()> end_of_loop;
        std::function<void()> print_rate_limited;
    };
}