
file(GLOB_RECURSE COMMON_SOURCES ${CMAKE_CURRENT_LIST_DIR}/common/*.cc)

include(add_target.cmake)

include_directories(${CMAKE_CURRENT_LIST_DIR})
include_directories(${CMAKE_CURRENT_LIST_DIR}/targets)

file(GLOB_RECURSE TEST_TARGET_SOURCES ${CMAKE_CURRENT_LIST_DIR}/targets/test/*.cc)
add_exe_target(test_target "${TEST_TARGET_SOURCES};${COMMON_SOURCES}")

file(GLOB_RECURSE WHEEL_LEGGED_SOURCES ${CMAKE_CURRENT_LIST_DIR}/targets/wheel_legged/*.cc)
add_exe_target(wheel_legged "${WHEEL_LEGGED_SOURCES};${COMMON_SOURCES}")
set(WHEEL_LEGGED_ROBOT_VARIANT "2" CACHE STRING "Wheel-legged robot parameter set: 1=hero, 2=infantry3(default), 3=infantry4")
set_property(CACHE WHEEL_LEGGED_ROBOT_VARIANT PROPERTY STRINGS 1 2 3)
target_compile_definitions(wheel_legged PRIVATE WHEEL_LEGGED_ROBOT_VARIANT=${WHEEL_LEGGED_ROBOT_VARIANT})

## add targets here...
