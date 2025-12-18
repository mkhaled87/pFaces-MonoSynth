cmake_minimum_required(VERSION 3.10)

# Name of the project
project(pFaces-MonoSynth)

# Include CMake configs for pFaces
if("$ENV{PFACES_SDK_ROOT}" STREQUAL "")
  message(FATAL_ERROR "Please make sure pFaces is installed correctly.")
else()
  include($ENV{PFACES_SDK_ROOT}/pfaces_kernel.cmake)
endif()
set(CMAKE_CXX_STANDARD 14)


# Kernel driver name and its sources
set(PFACES_KERNEL mono_synth)
set(PFACES_KERNEL_SOURCES 
    ${PROJECT_SOURCE_DIR}/kernel-driver/configReader.cpp
    ${PROJECT_SOURCE_DIR}/kernel-driver/pfacesKernel_mono_synth.cpp
)
set(PFACES_KERNEL_OUT_PATH "${PROJECT_SOURCE_DIR}/kernel-pack")

# Add the kernel driver as shared lib
ADD_LIBRARY(${PFACES_KERNEL} SHARED
    ${PFACES_KERNEL_SOURCES}
)
target_include_directories(${PFACES_KERNEL} PUBLIC
    ${PROJECT_SOURCE_DIR}/kernel
    ${OPENSSL_INCLUDE_DIR}
)

# Include link configs for the kernel.
include($ENV{PFACES_SDK_ROOT}/pfaces_link.cmake)
