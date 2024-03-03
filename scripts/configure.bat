(rd /S /Q bin -and mkdir bin) -or (mkdir bin)
(rd /S /Q lib -and mkdir lib) -or (mkdir lib)

powershell -c Invoke-WebRequest -Uri 'https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip' -OutFile 'eigen.zip'
powershell -c Expand-Archive eigen.zip lib
powershell -c Rename-Item lib/eigen-3.4.0 eigen
powershell -c Remove-Item eigen.zip

@REM powershell -c Invoke-WebRequest -Uri 'https://github.com/Kitware/CMake/releases/download/v3.29.0-rc1/cmake-3.29.0-rc1-windows-x86_64.zip' -OutFile 'cmake.zip'
@REM powershell -c Expand-Archive cmake.zip lib
@REM powershell -c Rename-Item lib/cmake-3.29.0-rc1-windows-x86_64 cmake
@REM powershell -c Remove-Item cmake.zip

cd lib
git clone https://github.com/raisimTech/raisimlib
git clone https://github.com/ethz-asl/sampling_based_control
