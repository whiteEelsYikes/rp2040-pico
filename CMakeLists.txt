# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Specify path to own LVGL config header
set(LV_CONF_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/lv_conf.h
    CACHE STRING "" FORCE)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(demo C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()


# Add subdirectory containing oled_driver
# add_subdirectory(oled_iic)

# Add executable. Default name is the project name, version 0.1
# add_executable(demo demo.c oled_iic/bsp_iic_oled.c)  # oled test 1
add_executable(demo demo.c oled/OLED.c oled/OLED_Data.c)


# 添加LVGL库
add_subdirectory(lvgl)

# add_subdirectory(ui)


pico_set_program_name(demo "demo")
pico_set_program_version(demo "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(demo 0)
pico_enable_stdio_usb(demo 1)

# Add the standard library to the build
target_link_libraries(demo
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(demo PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/lvgl
        ${CMAKE_CURRENT_LIST_DIR}/oled_iic
        ${CMAKE_CURRENT_LIST_DIR}/oled
        ${PICO_SDK_PATH}/include
)

# Add any user requested libraries
target_link_libraries(
        demo 
        # oled_iic
        lvgl::lvgl
        # ui
        hardware_i2c
        hardware_spi
        hardware_dma
        hardware_timer
        )

pico_add_extra_outputs(demo)

