## Development Environment

Running the configuration script will initialise the development environment on
windows. This can be done with the `Configure` VSCode task that configures the
environment for the current operating system. It creates the required folders
and downloads dependancies (except the windows SDK) to the local folder without
installing them on the system.

Dependencies:
- CMake (assumed to be installed locally)
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [Raisim](https://raisim.com/)

Raisim is proprietary and requires license keys to use. Information on getting
a license can be found [here](https://raisim.com/sections/License.html).
Licenses are free for academic use.

### Windows specific

Compiling windows C++ applications requires the microsoft visual studio compiler
(mvsc) that is available as part of the [build tools for visual
studio](https://visualstudio.microsoft.com/visual-cpp-build-tools/).
This should be installed on your local machine. Use the default selection that
includes cmake, the microsoft visual studio compiler (MVSC) and Windows SDK.

Once installed, **reboot**. The development environment is initialised by the
`Developer Command Prompt for VS 2022` script / application, and therefore
VSCode must be started by running `code .` in the prompt. This sets the numerous
environment variables used by the visual studio build tools, that prevent it
from being used free standing. See [Configure VS Code for Microsoft
C++](https://code.visualstudio.com/docs/cpp/config-msvc#_prerequisites).

## Building

#### Linux

Different programs can be built using the [compile.bash](scripts/compile.bash)
script that takes the source directory as an argument. The VSCode task `Compile`
uses this script to build the directory specified by `user_build_target` located
in the workspace settings file [`.vscode/settings.json`](.vscode/settings.json).
Change this variable to build different programs.
