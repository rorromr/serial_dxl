set(ARDUINO_DEFAULT_BOARD mega2560) # Default Board ID, when not specified
set(ARDUINO_DEFAULT_PORT /dev/ttyUSB0) # Default Port, when not specified

generate_arduino_firmware(serial_test
        SKETCH SerialTest
        HDRS  SerialTest/avr_dxl_serial.h
        PORT  /dev/ttyACM0
        SERIAL minicom -b 9600 -o -D @SERIAL_PORT@
        BOARD mega2560)
