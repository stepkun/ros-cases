cmake_minimum_required(VERSION 3.12)

# avoid error message regarding symlink-install
set(ignoreMe "${AMENT_CMAKE_SYMLINK_INSTALL}")
set(PICO_PLATFORM "rp2040")
set(PICO_BOARD "pico")
set(PICO_COMPILER "pico_arm_gcc")

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(uros_pkg C CXX ASM)

pico_sdk_init()

# find dependencies
#find_package(ament_cmake REQUIRED)
#find_package(rclc REQUIRED)
#find_package(test_interfaces REQUIRED)

link_directories($ENV{MICRO_ROS_PICO_SDK_PATH}/libmicroros)

add_executable(${PROJECT_NAME}
    src/main.c
    src/pico_uart_transport.c
)

target_include_directories(${PROJECT_NAME} PUBLIC
    include
    $ENV{MICRO_ROS_PICO_SDK_PATH}/libmicroros/include
)

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    microros
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    C_STANDARD 11
    C_EXTENSIONS YES
    CXX_STANDARD 17
    CXX_EXTENSIONS NO
)

# Configure Pico
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

target_compile_definitions(${PROJECT_NAME}
    PUBLIC PICO_UART_ENABLE_CRLF_SUPPORT=0
    PUBLIC PICO_STDIO_ENABLE_CRLF_SUPPORT=0
    PUBLIC PICO_STDIO_DEFAULT_CRLF=0
)

# Generate UF2
pico_add_extra_outputs(${PROJECT_NAME})

#ament_package()
