cmake_minimum_required(VERSION 3.20.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CONFIG_LIB_CPLUSPLUS ON)
set(CONFIG_NEWLIB_LIBC ON)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(posey-firmware)

add_subdirectory(drivers/sensor/bno08x)

file(GLOB_RECURSE SRCS_CPP ${CMAKE_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE SRCS_C ${CMAKE_SOURCE_DIR}/src/*.c)
target_sources(app PRIVATE ${SRCS_CPP} ${SRCS_C})
target_include_directories(app PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/posey-lib
    ${CMAKE_SOURCE_DIR}/drivers/sensor/bno08x)

zephyr_library_include_directories(.)
