cmake_minimum_required(VERSION 3.16)
project(damiao_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(motor_test
    src/motor_test.cpp
    src/communication/serial_port.cpp
    src/drivers/motor_control.cpp
)

target_include_directories(motor_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# LibSerial
find_library(
    LIBSERIAL_LIBRARY 
    NAMES serial 
    PATHS /usr/lib /usr/lib/x86_64-linux-gnu 
    NO_DEFAULT_PATH
    REQUIRED)
find_path(LIBSERIAL_INCLUDE_DIR NAMES libserial/SerialPort.h PATHS /usr/include REQUIRED)
if(NOT LIBSERIAL_LIBRARY)
    message(FATAL_ERROR "LibSerial not found. Install with: sudo apt install libserial-dev")
endif()

target_link_libraries(motor_test PRIVATE
    ${LIBSERIAL_LIBRARY}
    pthread
)
