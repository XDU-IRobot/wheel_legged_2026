set(BUILD_TESTS OFF)

add_compile_definitions(LIBRM_BYPASS_HAL_ASSERT)

add_subdirectory(librm)

add_library(tinympcstatic STATIC
        tinympc/src/tinympc/admm.cpp
        tinympc/src/tinympc/tiny_api.cpp
)
target_include_directories(tinympcstatic PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/tinympc/src
        ${CMAKE_CURRENT_LIST_DIR}/tinympc/include/Eigen
)
target_compile_definitions(tinympcstatic
        PUBLIC EIGEN_NO_IO TINY_MPC_NO_IO TINY_MPC_DISABLE_ADAPTIVE_RHO TINY_MPC_USE_FLOAT
)
target_compile_options(tinympcstatic PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:-Os>
)

add_library(${PROJECT_NAME}_libs INTERFACE)

target_link_libraries(${PROJECT_NAME}_libs INTERFACE
        rm
        tinympcstatic
)
