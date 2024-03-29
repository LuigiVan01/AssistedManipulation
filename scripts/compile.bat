:: Initialise the MSVC environment.
call "C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Auxiliary/Build"
where "cl"

set compiler = where cl
set VCPKG_ROOT="%cd%\tools\vcpkg"
set PATH=%VCPKG_ROOT%;%PATH%

:: Compile the application.
"tools/cmake/bin/cmake.exe" ^
    -A Win32 src ^
    -D CMAKE_CXX_COMPILER="C:\\Program Files (x86)\\Microsoft Visual Studio\\2022\\BuildTools\\VC\\Tools\\MSVC\\14.39.33519\\bin\\Hostx86\\x86\\cl.exe"
    -D CMAKE_TOOLCHAIN_FILE="$env{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
