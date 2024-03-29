(rd /S /Q bin && mkdir bin) || (mkdir bin)
(rd /S /Q lib && mkdir lib) || (mkdir lib)
(rd /S /Q tools && mkdir tools) || (mkdir tools)

:: Install cmake locally.
powershell -c Invoke-WebRequest -Uri 'https://github.com/Kitware/CMake/releases/download/v3.29.0-rc1/cmake-3.29.0-rc1-windows-x86_64.zip' -OutFile 'cmake.zip'
powershell -c Expand-Archive cmake.zip lib
powershell -c Rename-Item lib/cmake-3.29.0-rc1-windows-x86_64 cmake
powershell -c Remove-Item cmake.zip

:: Install vcpkg locally for dependencies.
git clone https://github.com/microsoft/vcpkg.git
powershell Move-Item vcpkg tools/vcpkg/
cd tools/vcpkg
bootstrap-vcpkg.bat
cd ../..

:: Set the root location for vcpkg
set VCPKG_ROOT="%cd%\tools\vcpkg"
set PATH=%VCPKG_ROOT%;%PATH%

:: Add dependencies.
vcpkg add port eigen3
vcpkg add port yaml-cpp

@REM cd lib
@REM git clone https://github.com/raisimTech/raisimlib
@REM git clone https://github.com/ethz-asl/sampling_based_control
