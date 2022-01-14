cmake_minimum_required(VERSION 3.12)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# include_directories(/Users/saypaul/git_test/arduino-pico/libraries/Wire/src)
# include_directories(/Users/saypaul/git_test/arduino-pico/cores/rp2040)
# include_directories(/Users/saypaul/git_test/arduino-pico/variants/rpipico)
# include_directories(/Users/saypaul/git_test/arduino-pico/pico-sdk/src/rp2_common/cmsis/stub/CMSIS/Device/RaspberryPi/RP2040/Include)
# include_directories(/Users/saypaul/git_test/arduino-pico/pico-sdk/src/rp2_common/cmsis/stub/CMSIS/Core/Include)
# include_directories(/Users/saypaul/git_test/arduino-pico/pico-sdk/src/rp2_common/hardware_pio/include)
project(fly2sky C CXX ASM)
# set(CMAKE)

pico_sdk_init()

add_executable(${PROJECT_NAME}
    main.cpp
    Wire.cpp
)
pico_add_extra_outputs(${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_i2c
    cmsis
    hardware_pio
)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)