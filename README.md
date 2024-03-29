## Development Environment

### Windows Prerequisites

Compiling windows C++ applications requires the microsoft visual studio compiler
(mvsc) that is available as part of the [build tools for visual
studio](https://visualstudio.microsoft.com/visual-cpp-build-tools/).
This should be installed on your local machine. Make sure the following are
selected (the default selection)
- Desktop development with C++
    - MSVC v143 - VS 2022 C++ 
    - Windows 11 SDK
    - C++ CMake tools for Windows
    - Testing tools core features - Build Tools
    - C++ AddressSanitzer

Once installed, **reboot**.

Note that a typical terminal will not have the build tools or compiler set on
the path. The provided `Developer Command Prompt for VS 2022` script /
application that comes with BuildTools can be used to get a terminal with the
correct environment variables. Running `code .` in this terminal will provide
every vscode terminal with the build tools. Alternatively, scripts are provided
that initialise the environment variables for a terminal already running. These
are used by the compilation script [compile.bat](scripts/compile.bat) to set the
environment before compiling. For more details see [Configure VS Code for
Microsoft
C++](https://code.visualstudio.com/docs/cpp/config-msvc#_prerequisites).

### Linux & Windows

Running the configuration script will download all required dependancies locally
without installing them on the system. Press `ctrl+shift+p` and type `Tasks: Run
Task` and select `Configure`. This task is defined in
[tasks.json](.vscode/tasks.json) which selects the correct script to run
depending on the operating system.

The dependencies are:
- CMake (assumed to be installed locally)
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [Raisim](https://raisim.com/)

Raisim is proprietary and requires license keys to use. Information on getting
a license can be found [here](https://raisim.com/sections/License.html).
Licenses are free for academic use.

## Building

#### Linux

Different programs can be built using the [compile.bash](scripts/compile.bash)
script that takes the source directory as an argument. The VSCode task `Compile`
uses this script to build the directory specified by `user_build_target` located
in the workspace settings file [`.vscode/settings.json`](.vscode/settings.json).
Change this variable to build different programs.
