@echo off

:: Initialise the MSVC environment.
call "C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Auxiliary/Build/vcvars64" || goto :error

:: %1 is the release or debug mode.
if %1==true (
    set DEBUG=Release
) else (
    set DEBUG=Debug
)

cd build
cmake --build . --config %DEBUG% || goto :error

cd ..
cmake --install build --config %DEBUG% --prefix install || goto :error

:: Success!
echo Success!
exit /b 0

:error
echo Failed!
exit /b %errorlevel%
