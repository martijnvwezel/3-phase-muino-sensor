cmake_minimum_required(VERSION 3.13)

# Initialize pico-sdk from submodule
# note: this must happen before project()
set(PICO_SDK_FETCH_FROM_GIT on)
include(pico_sdk_import.cmake)

project(watermeter C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# Add -O0 to remove optimizations when using gcc
# IF(CMAKE_COMPILER_IS_GNUCC)
#     set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
#     set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
# ENDIF(CMAKE_COMPILER_IS_GNUCC)

if (TARGET tinyusb_device)

    # Rest of your project
    add_executable(watermeter # projectname
        src/main.cpp # main file
    )

    add_definitions(-DPICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=1)
    add_definitions(-DPICO_DEFAULT_UART_BAUD_RATE=115200)

    # pull in common dependencies and additional i2c hardware support
    target_link_libraries(watermeter pico_stdlib hardware_i2c )

    # Enable usb output, enable uart output
    pico_enable_stdio_usb(watermeter 1)
    pico_enable_stdio_uart(watermeter 1)

    # Create map/bin/hex/uf2 file in addition to ELF.)
    pico_add_extra_outputs(watermeter)

    

elseif(PICO_ON_DEVICE)
    message(WARNING "not building main because TinyUSB submodule is not initialized in the SDK")
endif()





