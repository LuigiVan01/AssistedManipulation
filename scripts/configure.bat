(rd /S /Q bin && mkdir bin) || (mkdir bin)
(rd /S /Q lib && mkdir lib) || (mkdir lib)
(rd /S /Q tools && mkdir tools) || (mkdir tools)

:: Install vcpkg locally for dependencies.
git clone https://github.com/microsoft/vcpkg.git
powershell Move-Item vcpkg tools/vcpkg/
cd tools/vcpkg
bootstrap-vcpkg.bat
cd ../..

:: Set the root location for vcpkg
set VCPKG_ROOT="%cd%\tools\vcpkg"
set PATH=%VCPKG_ROOT%;%PATH%

:: Install dependancies.
vcpkg install eigen3

cd lib
git clone https://github.com/raisimTech/raisimlib
git clone https://github.com/ethz-asl/sampling_based_control
