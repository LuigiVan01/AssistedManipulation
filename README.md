## Development Environment

### Windows Development

Compiling windows C++ applications requires the microsoft visual studio compiler
(mvsc) that is available as part of the [Build Tools for Visual Studio 2022](https://visualstudio.microsoft.com/downloads/).
This should be installed on your local machine. Select C++ Development for
Desktop and ensure the following are selected:
- Desktop development with C++
    - MSVC v143 - VS 2022 C++ 
    - Windows 11 SDK
    - C++ CMake tools for Windows
    - Testing tools core features - Build Tools
    - C++ AddressSanitzer

Once installed, **reboot**.

To verify that the windows SDK was properly installed, check that the file
`C:\Program Files (x86)\Windows Kits\10\Lib\<version>\um\x64\kernel32.Lib`
exists. If not, go to `Settings`, `Apps`, `Installed Apps` and search for
`Windows Software Development Kit`. Click the three dots, click `Modify`, then
click `Repair` on the installer.

The MSVC toolchain will not be added to path. The provided `Developer Command
Prompt for VS 2022`  application that comes with `BuildTools` can be used to
start a terminal with the correct environment variables. Running `code .` in
this terminal will ensure every vscode terminal has access to the toolchain
tools. Alternatively, scripts are provided by `BuildTools` that initialise the
environment for a terminal already running. These are used by the compilation
script [compile.bat](scripts/compile.bat) to set the environment before
compiling. For more details see [Configure VS Code for Microsoft
C++](https://code.visualstudio.com/docs/cpp/config-msvc#_prerequisites).

Running the configuration script will download all required dependencies locally
without installing them on the system. Press `ctrl+shift+p` and type `Tasks: Run
Task` and select `Setup`. This task is defined in
[tasks.json](.vscode/tasks.json).

### Linux Development

Running the configuration script will download all required dependencies locally without installing them on the system. Press `ctrl+shift+p` and type `Tasks: Run
Task` and select `Setup`. This task is defined in
[tasks.json](.vscode/tasks.json).

### VSCode Development Container

A ROS development container is provided in the [`.devcontainer`](.devcontainer)
folder. Both Linux and WSL2 configurations are available with different file
mounts an commans to support graphics forwarding to the local OS. Copy and
rename the relevant configuration file to `.devcontainer.json` to use that
configuration, then press ctrl+shift+p and select `Dev Containers: Rebuild and
Reopen in Container` from the drop-down.

### Linux & Windows

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
