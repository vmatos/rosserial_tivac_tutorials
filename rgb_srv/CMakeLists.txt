cmake_minimum_required(VERSION 2.8.3)
project(rgb_srv)

# Include rosserial libraries for TivaC
include_directories(${ROS_LIB_DIR})

# Per project based definitions and options
add_definitions(-DTX_BUFFER_SIZE=256)
add_definitions(-DRX_BUFFER_SIZE=256)

# Generate target for TivaC
generate_tivac_firmware(
  USB
  SRCS rgb_led.cpp ${TIVA_WARE_PATH}/examples/boards/ek-tm4c123gxl/drivers/rgb.c 
  INCS ${TIVA_WARE_PATH}/examples/boards/ek-tm4c123gxl/drivers
  BOARD tm4c123gxl
)
