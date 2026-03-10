cmake_minimum_required(VERSION 3.20)
project(rt_controller
    VERSION 1.0.0
    DESCRIPTION "MonoSafe Real-Time Safe Controller Framework"
    LANGUAGES CXX
)

# ===========================================================================
# C++ Standard
# ===========================================================================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# ===========================================================================
# FetchContent dependencies (header-only)
# ===========================================================================
include(FetchContent)

# --- nlohmann/json ---
FetchContent_Declare(json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
    GIT_SHALLOW    TRUE
)
set(JSON_BuildTests OFF CACHE INTERNAL "")
FetchContent_MakeAvailable(json)

# --- Eigen3 ---
FetchContent_Declare(eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
    GIT_SHALLOW    TRUE
)
set(BUILD_TESTING OFF CACHE INTERNAL "")
set(EIGEN_BUILD_DOC OFF CACHE INTERNAL "")
set(EIGEN_BUILD_PKGCONFIG OFF CACHE INTERNAL "")
FetchContent_MakeAvailable(eigen)

# --- NLopt (system-installed, required by libmpc++) ---
find_package(NLopt QUIET)
if(NOT NLopt_FOUND)
    find_library(NLOPT_LIB NAMES nlopt PATHS /opt/homebrew/lib /usr/local/lib)
    find_path(NLOPT_INCLUDE NAMES nlopt.hpp PATHS /opt/homebrew/include /usr/local/include)
    if(NOT NLOPT_LIB)
        message(FATAL_ERROR "NLopt not found. Install with: brew install nlopt")
    endif()
endif()

# --- libmpc++ (header-only, fetched from GitHub) ---
FetchContent_Declare(libmpc
    GIT_REPOSITORY https://github.com/nicolapiccinelli/libmpc.git
    GIT_TAG        master
    GIT_SHALLOW    TRUE
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
)
# Don't run its CMakeLists (it requires OSQP etc.); just fetch the headers.
FetchContent_GetProperties(libmpc)
if(NOT libmpc_POPULATED)
    FetchContent_Populate(libmpc)
endif()
set(LIBMPC_INCLUDE_DIR "${libmpc_SOURCE_DIR}/include")

# ===========================================================================
# Optional: pFaces SDK (for direct synthesis backend)
# ===========================================================================
option(USE_PFACES_SDK "Enable direct pFaces SDK integration" OFF)
if(USE_PFACES_SDK)
    if(DEFINED ENV{PFACES_SDK_ROOT})
        set(PFACES_SDK_ROOT "$ENV{PFACES_SDK_ROOT}")
        set(PFACES_SDK_INCLUDE "${PFACES_SDK_ROOT}/include")
        find_library(PFACES_SDK_LIB NAMES pfaces-sdk PATHS "${PFACES_SDK_ROOT}/lib")
        find_library(PFACES_SDK_IFACE_LIB NAMES pfaces-sdk-interface PATHS "${PFACES_SDK_ROOT}/lib")
    else()
        message(WARNING "USE_PFACES_SDK=ON but PFACES_SDK_ROOT not set. Set the environment variable.")
    endif()
endif()

# ===========================================================================
# Main Target
# ===========================================================================
set(RT_CONTROLLER_SOURCES src/main.cpp)

# Add kernel driver sources when using pFaces SDK
if(USE_PFACES_SDK AND PFACES_SDK_LIB)
    set(KERNEL_DRIVER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../kernel-driver")
    list(APPEND RT_CONTROLLER_SOURCES
        ${KERNEL_DRIVER_DIR}/configReader.cpp
        ${KERNEL_DRIVER_DIR}/pfacesKernel_mono_synth.cpp
    )
endif()

add_executable(rt_controller ${RT_CONTROLLER_SOURCES})

target_include_directories(rt_controller PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(rt_controller PRIVATE
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
)

# libmpc++ (header-only)
target_include_directories(rt_controller PRIVATE ${LIBMPC_INCLUDE_DIR})

# NLopt
if(NLopt_FOUND)
    target_link_libraries(rt_controller PRIVATE NLopt::nlopt)
else()
    target_link_libraries(rt_controller PRIVATE ${NLOPT_LIB})
    target_include_directories(rt_controller PRIVATE ${NLOPT_INCLUDE})
endif()

# pFaces SDK (optional)
if(USE_PFACES_SDK AND PFACES_SDK_LIB)
    target_compile_definitions(rt_controller PRIVATE HAS_PFACES_SDK=1)
    target_include_directories(rt_controller PRIVATE ${PFACES_SDK_INCLUDE})
    target_include_directories(rt_controller PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../kernel-driver
    )
    # OpenCL C++ headers (from parent project's FetchContent)
    # Search common locations for the OpenCL-CLHPP headers
    set(OPENCL_HPP_CANDIDATES
        "${CMAKE_CURRENT_SOURCE_DIR}/../../build/openclhpp-src/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../pFaces-MonoSynth/build/openclhpp-src/include"
    )
    foreach(candidate ${OPENCL_HPP_CANDIDATES})
        if(EXISTS "${candidate}/CL/opencl.hpp")
            set(OPENCL_HPP_INCLUDE "${candidate}")
            break()
        endif()
    endforeach()
    if(NOT OPENCL_HPP_INCLUDE)
        # Fetch OpenCL-CLHPP if not found
        FetchContent_Declare(openclhpp
            GIT_REPOSITORY https://github.com/KhronosGroup/OpenCL-CLHPP.git
            GIT_TAG        main
            GIT_SHALLOW    TRUE
        )
        FetchContent_GetProperties(openclhpp)
        if(NOT openclhpp_POPULATED)
            FetchContent_Populate(openclhpp)
        endif()
        set(OPENCL_HPP_INCLUDE "${openclhpp_SOURCE_DIR}/include")
    endif()
    target_include_directories(rt_controller PRIVATE ${OPENCL_HPP_INCLUDE})
    # Link both SDK libraries
    target_link_libraries(rt_controller PRIVATE ${PFACES_SDK_LIB})
    if(PFACES_SDK_IFACE_LIB)
        target_link_libraries(rt_controller PRIVATE ${PFACES_SDK_IFACE_LIB})
    endif()
    # OpenCL framework (macOS)
    if(APPLE)
        target_link_libraries(rt_controller PRIVATE "-framework OpenCL")
    else()
        find_package(OpenCL REQUIRED)
        target_link_libraries(rt_controller PRIVATE OpenCL::OpenCL)
    endif()
endif()

# Filesystem (for older GCC)
target_link_libraries(rt_controller PRIVATE
    $<$<AND:$<CXX_COMPILER_ID:GNU>,$<VERSION_LESS:$<CXX_COMPILER_VERSION>,9.1>>:stdc++fs>
)

# ===========================================================================
# pFaces SDK: symlink runtime-required 'kernels/' directory next to binary
# The pFaces SDK's loadFromFile() expects kernels/func_mem.template and
# kernels/func_mem.defaults to be in the same directory as the executable.
# ===========================================================================
if(USE_PFACES_SDK)
    # Derive pFaces bin/kernels from SDK root (SDK root is pfaces-sdk/, kernels is in ../bin/kernels/)
    get_filename_component(_PFACES_PARENT "${PFACES_SDK_ROOT}" DIRECTORY)
    set(_PFACES_KERNELS "${_PFACES_PARENT}/bin/kernels")
    if(EXISTS "${_PFACES_KERNELS}")
        add_custom_command(TARGET rt_controller POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E create_symlink
                "${_PFACES_KERNELS}"
                "$<TARGET_FILE_DIR:rt_controller>/kernels"
            COMMENT "Symlinking pFaces kernels/ runtime support"
        )
    else()
        message(WARNING "pFaces kernels dir not found at ${_PFACES_KERNELS} — .mem rendering may fail at runtime")
    endif()
endif()

# ===========================================================================
# Install
# ===========================================================================
install(TARGETS rt_controller RUNTIME DESTINATION bin)

# ===========================================================================
# Config summary
# ===========================================================================
message(STATUS "")
message(STATUS "=== MonoSafe RT Controller ===")
message(STATUS "  C++ Standard:  ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Type:    ${CMAKE_BUILD_TYPE}")
message(STATUS "  pFaces SDK:    ${USE_PFACES_SDK}")
message(STATUS "==============================")
