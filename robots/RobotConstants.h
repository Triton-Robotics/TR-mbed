#pragma once

namespace TR
{

    constexpr float JOYSTICK_SENSITIVITY_YAW_DPS = 180.0;
    constexpr float JOYSTICK_SENSITIVITY_PITCH_DPS = 180.0;

    // Mouse sensitivity initialized
    constexpr float MOUSE_SENSITIVITY_YAW_DPS = 10.0;
    constexpr float MOUSE_SENSITIVITY_PITCH_DPS = 10.0;

    constexpr int OUTER_LOOP_DT_MS = 1;

// ROBOT SPECIFIC CONSTANTS BELOW
#ifdef INFANTRY
    constexpr int pitch_zero_offset_ticks = 1500;
#endif

#ifdef SENTRY

#endif

#ifdef HERO

#endif
}