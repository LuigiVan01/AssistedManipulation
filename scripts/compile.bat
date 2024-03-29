@echo off

:: Initialise the MSVC environment. Sets path for cmake, cl.
call "C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Auxiliary/Build/vcvarsx86_amd64" || goto :error

:: Register vcpkg on path.
set VCPKG_ROOT="%cd%\tools\vcpkg"
set PATH=%VCPKG_ROOT%;%PATH%

:: Create build files.
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE="%VCPKG_ROOT%/scripts/buildsystems/vcpkg.cmake" ../src || goto :error

:: Compile the application.
cmake --build .

:: Success!
echo Success!
exit /b 0

:error
echo Failed!
exit /b %errorlevel%
