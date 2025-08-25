# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.12)

# Import the Pico SDK (ensure the PICO_SDK_PATH environment variable is set)
include(pico_sdk_import.cmake)

# Set the project name and language
project(function_gen C CXX ASM)

# Initialize the Pico SDK
pico_sdk_init()

# Add the executable target from the stub main file
add_executable(function_gen
    src/main.c
)
pico_enable_stdio_usb(function_gen 1)
pico_enable_stdio_uart(function_gen 0)
# Link the Pico standard library
target_link_libraries(function_gen pico_stdlib hardware_pwm hardware_pio hardware_dma hardware_adc)

pico_generate_pio_header(function_gen ${CMAKE_CURRENT_LIST_DIR}/src/dac_high.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)

# Create additional outputs such as UF2 for flashing to the Pico
pico_add_extra_outputs(function_gen)
