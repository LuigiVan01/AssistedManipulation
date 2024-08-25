@echo off

:: Initialise the MSVC environment. Sets path for cmake, cl.
call "C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Auxiliary/Build/vcvars64" || goto :error

:: %1 is "Release" or "Debug"

:: Register vcpkg on path.
set WORKSPACE=%cd%
set VCPKG_ROOT=%WORKSPACE%\vcpkg
set RAISIM_ROOT=%WORKSPACE%\lib\raisimlib\raisim\win32
set PATH=%VCPKG_ROOT%;%RAISIM_ROOT%;%PATH%

:: Create build files.
(rd /S /Q build && mkdir build) || (mkdir build)
cd build

cmake ^
    -DCMAKE_TOOLCHAIN_FILE="%VCPKG_ROOT%/scripts/buildsystems/vcpkg.cmake" ^
    -DCMAKE_PREFIX_PATH="%RAISIM_ROOT%" ^
    -DCMAKE_BUILD_TYPE=%1 ^
    -Wno-dev ^
    ../src ^
    || goto :error

:: Success!
echo Success!
exit /b 0

:error
echo Failed!
exit /b %errorlevel%
