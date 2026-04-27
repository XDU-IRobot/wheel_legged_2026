
file(GLOB_RECURSE COMMON_SOURCES ${CMAKE_CURRENT_LIST_DIR}/common/*.cc)

include(add_target.cmake)

include_directories(${CMAKE_CURRENT_LIST_DIR})
include_directories(${CMAKE_CURRENT_LIST_DIR}/targets)

file(GLOB_RECURSE TEST_TARGET_SOURCES ${CMAKE_CURRENT_LIST_DIR}/targets/test/*.cc)
add_exe_target(test_target "${TEST_TARGET_SOURCES};${COMMON_SOURCES}")

file(GLOB_RECURSE WHEEL_LEGGED_SOURCES ${CMAKE_CURRENT_LIST_DIR}/targets/wheel_legged/*.cc)
add_exe_target(wheel_legged "${WHEEL_LEGGED_SOURCES};${COMMON_SOURCES}")

## add targets here...
