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
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

set(PICO_BOARD pico_w CACHE STRING "Board type")
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/FreeRTOS-Kernel)

include(pico_sdk_import.cmake)
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)
project(main C CXX ASM)

pico_sdk_init()

file(GLOB SRC_FILES "src/*.c")
file(GLOB SRC_LIB_FILES "lib/mqtt/src/*.c")
add_executable(main ${SRC_FILES} ${SRC_LIB_FILES})

pico_set_program_name(main "main")
pico_set_program_version(main "0.1")

pico_enable_stdio_uart(main 0)
pico_enable_stdio_usb(main 1)

target_link_libraries(main
    pico_stdlib
    hardware_adc
    FreeRTOS-Kernel 
    FreeRTOS-Kernel-Heap4
    hardware_i2c
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_mqtt
    pico_lwip
    pico_cyw43_driver
)

target_include_directories(main PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/lib/mqtt/include
)

pico_add_extra_outputs(main)
