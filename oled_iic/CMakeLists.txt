cmake_minimum_required(VERSION 3.13)

add_library(oled_iic STATIC
    bsp_iic_oled.c
)

target_include_directories(oled_iic PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${PICO_SDK_PATH}/include
)
