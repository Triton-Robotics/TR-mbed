add_library(robots_common)

target_include_directories(robots_common PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# include_directories(${CMAKE_CURRENT_SOURCE_DIR})



add_subdirectory(Infantry EXCLUDE_FROM_ALL)
# TODO FIX ME
# add_subdirectory(TestBench EXCLUDE_FROM_ALL)
# add_subdirectory(Engineer EXCLUDE_FROM_ALL)
# add_subdirectory(Sentry EXCLUDE_FROM_ALL)
# add_subdirectory(Hero EXCLUDE_FROM_ALL)
