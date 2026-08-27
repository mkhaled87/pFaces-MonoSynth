cmake_minimum_required(VERSION 3.12)

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
    ${PROJECT_SOURCE_DIR}/kernel-driver/basis_store.h
)
set(PFACES_KERNEL_OUT_PATH "${PROJECT_SOURCE_DIR}/kernel-pack")

# Add the kernel driver as shared lib
ADD_LIBRARY(${PFACES_KERNEL} SHARED
    ${PFACES_KERNEL_SOURCES}
)
target_include_directories(${PFACES_KERNEL} PUBLIC
    ${PROJECT_SOURCE_DIR}/kernel-driver
    ${OPENSSL_INCLUDE_DIR}
)

# Include link configs for the kernel.
include($ENV{PFACES_SDK_ROOT}/pfaces_link.cmake)

include(CTest)
if(BUILD_TESTING)
  add_executable(test_basis_store tests/test_basis_store.cpp)
  target_include_directories(test_basis_store PRIVATE
      ${PROJECT_SOURCE_DIR}/kernel-driver)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(test_basis_store PRIVATE -UNDEBUG)
  endif()
  add_test(NAME basis_store COMMAND test_basis_store)

  find_package(Python3 COMPONENTS Interpreter)
  if(Python3_Interpreter_FOUND)
    add_test(NAME four_method_oracle
             COMMAND ${Python3_EXECUTABLE}
                     ${PROJECT_SOURCE_DIR}/tests/check_four_methods.py)
  endif()
endif()
