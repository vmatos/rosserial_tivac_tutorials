cmake_minimum_required(VERSION 2.8.3)
project(buttons)

# Include rosserial libraries for TivaC
include_directories(${ROS_LIB_DIR})

# Per project based definitions and options
add_definitions(-DLED_HEARTBEAT)
add_definitions(-DLED_COMM)
add_definitions(-DTX_BUFFER_SIZE=256)
add_definitions(-DRX_BUFFER_SIZE=256)

# Generate target for TivaC
generate_tivac_firmware(
  USB
  SRCS buttons.cpp buttons.c
  INCS .
  BOARD tm4c123gxl
)
