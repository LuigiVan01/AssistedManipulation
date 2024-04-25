@echo off

:: Initialise the MSVC environment.
call "C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Auxiliary/Build/vcvars64" || goto :error

cd build
cmake --build . || goto :error

:: Success!
echo Success!
exit /b 0

:error
echo Failed!
exit /b %errorlevel%
