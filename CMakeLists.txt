cmake_minimum_required(VERSION 3.16)

project(hang_dwa VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(hang_dwa STATIC
    src/dwa_core.cpp
    src/dwa_costs.cpp
    src/dwa_map.cpp
)

add_library(hang_dwa::hang_dwa ALIAS hang_dwa)

target_include_directories(hang_dwa
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_compile_features(hang_dwa PUBLIC cxx_std_14)

set_target_properties(hang_dwa PROPERTIES CXX_EXTENSIONS OFF)

option(HANG_DWA_BUILD_TESTS "Build standalone DWA regression checks" OFF)
if(HANG_DWA_BUILD_TESTS)
    enable_testing()
    add_executable(dwa_regression tests/dwa_regression.cpp)
    target_link_libraries(dwa_regression PRIVATE hang_dwa::hang_dwa)
    add_test(NAME dwa_regression COMMAND dwa_regression)
endif()
