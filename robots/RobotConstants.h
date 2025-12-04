#pragma once

namespace TR
{
    // TODO should we delete these and move into remote.h, since this is remote specific?
    constexpr float JOYSTICK_SENSITIVITY_YAW_DPS = 180.0;
    constexpr float JOYSTICK_SENSITIVITY_PITCH_DPS = 180.0;

    // Mouse sensitivity initialized
    constexpr float MOUSE_SENSITIVITY_YAW_DPS = 10.0;
    constexpr float MOUSE_SENSITIVITY_PITCH_DPS = 10.0;

    constexpr int OUTER_LOOP_DT_MS = 1;

// ROBOT SPECIFIC CONSTANTS BELOW
#ifdef INFANTRY
    // TODO
    // Motor IDs

    // General constants
    constexpr float CHASSIS_FF_KICK = 0.065;

    // Pitch bounds
    constexpr float LOWERBOUND = -35.0;
    constexpr float UPPERBOUND = 40.0;

    // Initialization ticks (may not be needed with M3508)
    constexpr int pitch_zero_offset_ticks = 1500;
#endif

#ifdef SENTRY

#endif

#ifdef HERO

#endif
}