# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Select the board before importing the SDK and choosing its toolchain.
set(PICO_BOARD pico2_w CACHE STRING "Target Pico board")

if(NOT PICO_SDK_PATH AND DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}")
endif()

if(NOT EXISTS "${PICO_SDK_PATH}/pico_sdk_init.cmake")
    message(FATAL_ERROR "Set PICO_SDK_PATH to an installed Pico SDK 2.x directory.")
endif()

include("${PICO_SDK_PATH}/pico_sdk_init.cmake")
project(geophone C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
pico_sdk_init()

add_executable(geophone main.c)
target_sources(geophone PRIVATE
    third_party/kissfft/kiss_fft.c
    third_party/kissfft/kiss_fftr.c
)

target_include_directories(geophone PRIVATE
    third_party/kissfft
)

target_link_libraries(geophone PRIVATE m)
target_link_libraries(geophone PRIVATE pico_stdlib hardware_adc hardware_dma hardware_i2c)
target_compile_options(geophone PRIVATE -Wall -Wextra)

# printf/puts go to the computer over USB CDC, not the UART pins.
pico_enable_stdio_usb(geophone 1)
pico_enable_stdio_uart(geophone 0)
pico_add_extra_outputs(geophone)
