cmake_minimum_required(VERSION 2.8)

option(EMBEDDED_TARGET "Target" ON)

if(EMBEDDED_TARGET)
    # Arduino Target
    message("Embedded target")
    #set(ARDUINO_SDK_PATH /opt/arduino)
    set(CMAKE_TOOLCHAIN_FILE cmake/ArduinoToolchain.cmake) # Arduino Toolchain

    project(arduino_example C CXX)
    print_board_list()
    print_programmer_list()
    add_subdirectory(dxl_slave)
else()
    # PC Target
    message("PC target")
    add_subdirectory(odict)
endif()