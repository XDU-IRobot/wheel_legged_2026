
file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_LIST_DIR}/*.c
        ${CMAKE_CURRENT_LIST_DIR}/*.cc
        ${CMAKE_CURRENT_LIST_DIR}/*.cpp
        ${CMAKE_CURRENT_LIST_DIR}/*.s
        ${CMAKE_CURRENT_LIST_DIR}/*.S
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${USER_SOURCES}
)

target_include_directories(stm32cubemx INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/fsm
        ${CMAKE_CURRENT_LIST_DIR}/communication
        ${CMAKE_CURRENT_LIST_DIR}/chassis
        ${CMAKE_CURRENT_LIST_DIR}/chassis/wbr
        ${CMAKE_CURRENT_LIST_DIR}/chassis/state
        ${CMAKE_CURRENT_LIST_DIR}/utils
)
