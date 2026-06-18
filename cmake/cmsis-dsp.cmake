get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

add_library(cmsis_dsp INTERFACE)

target_compile_definitions(cmsis_dsp INTERFACE
    ARM_MATH_CM7
)

target_include_directories(cmsis_dsp INTERFACE
    ${PROJECT_ROOT}/Middlewares/ST/ARM/DSP/Inc
)

set(CMSIS_DSP_LIBRARY "" CACHE FILEPATH "Path to the CMSIS-DSP prebuilt math library")

if(CMSIS_DSP_LIBRARY STREQUAL "")
    foreach(_cmsis_dsp_lib_name
        libarm_cortexM7lfdp_math.a
        libarm_cortexM7lfsp_math.a
    )
        file(GLOB _cmsis_dsp_candidates
            "${PROJECT_ROOT}/Drivers/CMSIS/DSP/Lib/GCC/${_cmsis_dsp_lib_name}"
            "${PROJECT_ROOT}/Middlewares/ST/ARM/DSP/Lib/GCC/${_cmsis_dsp_lib_name}"
            "${PROJECT_ROOT}/Middlewares/Third_Party/ARM/DSP/Lib/GCC/${_cmsis_dsp_lib_name}"
            "${PROJECT_ROOT}/docs/dm-mc02/*/CtrBoard-H7_ALL/Drivers/CMSIS/DSP/Lib/GCC/${_cmsis_dsp_lib_name}"
            "${PROJECT_ROOT}/docs/dm-mc02/*/CtrBoard-H7_USB/Drivers/CMSIS/DSP/Lib/GCC/${_cmsis_dsp_lib_name}"
        )
        if(_cmsis_dsp_candidates)
            list(SORT _cmsis_dsp_candidates)
            list(GET _cmsis_dsp_candidates 0 CMSIS_DSP_LIBRARY)
            break()
        endif()
    endforeach()
endif()

if(CMSIS_DSP_LIBRARY STREQUAL "" OR NOT EXISTS "${CMSIS_DSP_LIBRARY}")
    message(FATAL_ERROR "CMSIS-DSP library (libarm_cortexM7lfdp_math.a or libarm_cortexM7lfsp_math.a) not found")
endif()

# Keep the linker input in an ASCII build path; Windows linkers can fail on
# non-ASCII source paths such as the vendor example directory.
set(CMSIS_DSP_STAGING_DIR ${CMAKE_BINARY_DIR}/cmsis_dsp_lib)
file(MAKE_DIRECTORY ${CMSIS_DSP_STAGING_DIR})
file(COPY ${CMSIS_DSP_LIBRARY} DESTINATION ${CMSIS_DSP_STAGING_DIR})
get_filename_component(CMSIS_DSP_LIBRARY_NAME ${CMSIS_DSP_LIBRARY} NAME)
set(CMSIS_DSP_LIBRARY_LOCAL ${CMSIS_DSP_STAGING_DIR}/${CMSIS_DSP_LIBRARY_NAME})

message(STATUS "Using CMSIS-DSP library: ${CMSIS_DSP_LIBRARY}")

target_link_libraries(cmsis_dsp INTERFACE
    ${CMSIS_DSP_LIBRARY_LOCAL}
)
