# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/Users/james/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Kitronik-Robotics-Board C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(Kitronik-Robotics-Board Kitronik-Robotics-Board.c )

pico_set_program_name(Kitronik-Robotics-Board "Kitronik-Robotics-Board")
pico_set_program_version(Kitronik-Robotics-Board "0.1")

# Add any user requested libraries
target_link_libraries(Kitronik-Robotics-Board
        pico_stdlib
        hardware_i2c
        )

pico_add_extra_outputs(Kitronik-Robotics-Board)

# Enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
