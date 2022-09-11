cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# # Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake 
         ${PROJECT_SOURCE_DIR}
)

project(Kitronik-Robotics-Board C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(main main.c regwrite.c Kitronik-Robotics-Board.c)

# # pico_set_program_name(Kitronik-Robotics-Board "Kitronik-Robotics-Board")
# # pico_set_program_version(Kitronik-Robotics-Board "0.1")

pico_add_extra_outputs(main)

# Add any user requested libraries
target_link_libraries(main
        pico_stdlib
        hardware_i2c
        )


# Enable usb output, disable uart output
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)
