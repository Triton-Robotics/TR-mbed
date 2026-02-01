# Suppress warnings for all mbed-os sources
file(GLOB_RECURSE MBED_C_SOURCES "${CMAKE_SOURCE_DIR}/mbed-os/*.c")
file(GLOB_RECURSE MBED_CXX_SOURCES "${CMAKE_SOURCE_DIR}/mbed-os/*.cpp")

set_source_files_properties(
    ${MBED_C_SOURCES}
    ${MBED_CXX_SOURCES}
    PROPERTIES
    COMPILE_FLAGS "-w"
)

# --- INFANTRY -- 
add_executable(TR-Infantry)
mbed_configure_app_target(TR-Infantry)
target_sources(TR-Infantry
    PRIVATE 
    Infantry/Infantry.cpp
)
set_target_properties(TR-Infantry PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/Infantry"
)
mbed_set_post_build(TR-Infantry)


# --- SENTRY ---
add_executable(TR-Sentry)
mbed_configure_app_target(TR-Sentry)
target_sources(TR-Sentry
    PRIVATE
    Sentry/Sentry.cpp
)
set_target_properties(TR-Sentry PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/Sentry"
)
mbed_set_post_build(TR-Sentry)

# --- HERO ---
add_executable(TR-Hero)
mbed_configure_app_target(TR-Hero)
target_sources(TR-Hero
    PRIVATE
    Hero/Hero.cpp
)
set_target_properties(TR-Hero PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/Hero"
)
mbed_set_post_build(TR-Hero)

# --- TestBench ---
add_executable(TR-TestBench)
mbed_configure_app_target(TR-TestBench)
target_sources(TR-TestBench
    PRIVATE
    TestBench/TestBench.cpp
)
set_target_properties(TR-TestBench PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/TestBench"
)
mbed_set_post_build(TR-TestBench)