@ECHO OFF

rem This is supposed be run from the Developer Command Prompt for Visual Studio
rem because we depend on the variable VCPKG_ROOT set by MSVC Developer Command Prompt.
rem You may first install the needed packages with vcpkg, e.g.:
rem $ vcpkg install
rem from the root of this proejct so that the vcpkg.json is found. CMake should detect
rem and vcpkg.json and install the needed packages automatically if you do not do it manually.

rem CMake settings for using Visual Studio. You may need to chnage the 
rem VS version with one from the list in 'cmake --help'.
set BUILD_TYPE=Release
set VS_VERSION="Visual Studio 17 2022"
set BUILD_DEF=-DCMAKE_BUILD_TYPE=%BUILD_TYPE%
set VCPKG_TRIPLET=-DVCPKG_TARGET_TRIPLET=x64-windows

rem In case you need to use a custom OpenCL implementation, update this (OFF, or ON and fill values)
set CUSTOM_OPENCL=OFF
set CUSTOM_OPENCL_INC_DIR=%OCL_ROOT%\include
set CUSTOM_OPENCL_LIB_PATH=%OCL_ROOT%\lib\x86_64\OpenCL.lib

rem Remove any old build
IF NOT EXIST .\build GOTO BUILDING
  rmdir /S/Q .\build

rem Building ....
:BUILDING
set vcpkg=-DCMAKE_TOOLCHAIN_FILE="%VCPKG_ROOT%/scripts/buildsystems/vcpkg.cmake"
mkdir build
cd build
cmake .. -Wno-dev -Wno-deprecated %BUILD_DEF% %vcpkg% %VCPKG_TRIPLET% -G %VS_VERSION% -DCUSTOM_OPENCL:BOOL=%CUSTOM_OPENCL% -DCUSTOM_OPENCL_INC_DIR="%CUSTOM_OPENCL_INC_DIR%" -DCUSTOM_OPENCL_LIB_PATH="%CUSTOM_OPENCL_LIB_PATH%"
cmake --build . --config %BUILD_TYPE%
cd ..
