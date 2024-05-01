:: Clear directories.
(rd /S /Q bin && mkdir bin) || (mkdir bin)
(rd /S /Q lib && mkdir lib) || (mkdir lib)
(rd /S /Q build && mkdir build) || (mkdir build)
(rd /S /Q vcpkg && mkdir vcpkg) || (mkdir vcpkg)

:: Install vcpkg locally for dependencies.
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
call bootstrap-vcpkg.bat
cd ..

:: Set the root location for vcpkg.
set VCPKG_ROOT=%cd%\vcpkg
set PATH=%VCPKG_ROOT%;%PATH%

:: Install dependancies.
vcpkg install eigen3 yaml-cpp

cd lib
git clone https://github.com/raisimTech/raisimlib
git clone https://github.com/ethz-asl/sampling_based_control
