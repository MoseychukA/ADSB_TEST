
# Pico /2040 waveform generator 

cmake_minimum_required(VERSION 3.12)
include(pico_sdk_import.cmake)

project(picowave C CXX ASM)

pico_sdk_init()

add_compile_options(-Wall -Wextra -Wno-unused-parameter)

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.c ${PROJECT_NAME}.pio)

target_link_libraries(${PROJECT_NAME} pico_stdlib hardware_spi pico_rand hardware_pio 
    hardware_dma hardware_pwm hardware_flash)

pico_add_extra_outputs(${PROJECT_NAME})

pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/${PROJECT_NAME}.pio)

# Console output on UART1
add_definitions(-DPICO_DEFAULT_UART=1)

# disable uart output, enable usb output
pico_enable_stdio_usb( ${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)