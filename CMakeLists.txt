# cmake version
cmake_minimum_required(VERSION 3.13)

# include the sdk.cmake file
include(pico_sdk_import.cmake)

# give the project a name (anything you want)
project(1D_Helicopter C CXX ASM)

# initialize the sdk
pico_sdk_init()

add_executable(1D_Helicopter)

# must match with pio filename and executable name from above
pico_generate_pio_header(1D_Helicopter ${CMAKE_CURRENT_LIST_DIR}/hsync.pio)
pico_generate_pio_header(1D_Helicopter ${CMAKE_CURRENT_LIST_DIR}/vsync.pio)
pico_generate_pio_header(1D_Helicopter ${CMAKE_CURRENT_LIST_DIR}/rgb.pio)

# must match with executable name and source file names
target_sources(1D_Helicopter PRIVATE 1d-helicopter.c vga16_graphics_v2.c mpu6050.c)

# Add pico_multicore which is required for multicore functionality
target_link_libraries(1D_Helicopter PRIVATE pico_stdlib pico_bootsel_via_double_reset pico_multicore hardware_pwm hardware_dma hardware_irq hardware_adc hardware_pio hardware_i2c hardware_clocks)

pico_enable_stdio_usb(1D_Helicopter 1)
pico_enable_stdio_uart(1D_Helicopter 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(1D_Helicopter)