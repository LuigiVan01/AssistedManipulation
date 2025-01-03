# Development Environment

## Windows Setup

### `msvc`

Compiling windows C++ applications requires the microsoft visual studio compiler
(mvsc) which is available as part of the [Build Tools for Visual Studio 2022](https://visualstudio.microsoft.com/downloads/).
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
start a terminal with the correct environment variables. This can run by searching for `"Developer Command Prompt VS 2022"` in the windows search bar. Running `code .` in
this terminal will ensure every vscode terminal has access to the toolchain
tools. Alternatively, scripts are provided by `BuildTools` that initialise the
environment for a terminal already running. These are used by the compilation
script [compile.bat](scripts/compile.bat) to set the environment before
compiling. For more details see [Configure VS Code for Microsoft
C++](https://code.visualstudio.com/docs/cpp/config-msvc#_prerequisites).

Press `ctrl+shift+p` and type `Tasks: Run
Task` and select `Setup`. Running the configuration script will download all required dependencies locally
without installing them on the system. This task is defined in
[tasks.json](.vscode/tasks.json).

## Linux Setup

Running the configuration script will download all required dependencies locally without installing them on the system. Press `ctrl+shift+p` and type `Tasks: Run
Task` and select `Setup`. This task is defined in
[tasks.json](.vscode/tasks.json).

## RaiSim License

If being used, the physics simulator requires a license file to be included in the root folder (i.e. next to `src`). Information on getting
a license can be found [here](https://raisim.com/sections/License.html).
Licenses are free for academic use.

## Dependency Overview

- [RaiSim](https://raisim.com/)
  - A closed source and licensed high fidelity physics simulator for   robotics. Used to simulate controlled systems including the [Franka Research 3](https://franka.de/products) and [Clearpath Ridgeback](https://clearpathrobotics.com/ridgeback-indoor-robot-platform/).
- [vcpkg](https://learn.microsoft.com/en-gb/vcpkg/get_started/overview)
  - For windows, package dependancies and management are done with
    [vcpkg](https://vcpkg.io/en/index.html) installed in the root folder. This is
    installed and all dependancies downloaded wit. the
    [setup.bat](scripts/windows/setup.bat) folder,
- [Eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html)
  - A common C++ linear algebra library.
- [OSQP](https://osqp.org/docs/release-0.6.3/)
  - A C++ quadratic program solving library. Currently unused.

### ROS Development Container

A ROS development container is provided in the [`.devcontainer`](.devcontainer)
folder. Press `ctrl+shift+p` and select `Dev Containers:
Rebuild and Reopen in Container` from the drop-down to reopen the development environment in a ROS environment.

# Running

Before running tests, the build should be configured as `Debug`. Press `ctrl+shift+p`, select `Tasks: Run Task` and select `Configure Debug`. This task runs the appropriate configuration script on either linux or windows, defined under [scripts](/scripts). Another task `Configure Release` is used to produce release builds.

Next, ensure the correct debug configuration is selected in VSCode. Click the debug symbol, and ensure the dropdown debug configuration is appropriate for the development environment, either windows or linux.

Finally, press `F5` and select a test to run. The RaiSim visualiser will automatically be started and stopped during the duration of the test.]

# File Structure

- [src](/src/)
  - [controller](/src/controller/) - Core functionality.
    - [concurrency.hpp](/src/controller/concurrency.hpp) - Multithreading task support.
    - [cost.hpp](/src/controller/cost.hpp) - Various cost / objective function helpers such as quadratic functions, logarithmic and inverse barrier functions.
    - [eigen.hpp](/src/controller/eigen.hpp) - Entrypoint for the eigen library, with common typedefs and extended functionality.
    - [energy.hpp](/src/controller/energy.hpp) - Simple energy tank formulation.
    - [filter.hpp](/src/controller/filter.hpp) / [filter.cpp](/src/controller/filter.cpp) - Smoothing filters and timeseries windows.
    - [forecast.hpp](/src/controller/forecast.hpp) / [forecast.hpp](/src/controller/forecast.cpp) - Forecasting strategies from sampled inputs.
    - [json.hpp](/src/controller/json.hpp) - Entrypoint for the nlohmann json library with eigen matrix serialisation.
    - [kalman.hpp](/src/controller/kalman.hpp) / [kalman.cpp](/src/controller/kalman.hpp) - Kalman filter implementation using eigen.
    - [mppi.hpp](/src/controller/mppi.hpp) / [mppi.cpp](/src/controller/mppi.cpp) - Model predictive path integral controller implementation.
    - [pid.hpp](/src/controller/pid.hpp) / [pid.cpp](/src/controller/pid.cpp) - PID controller.
    - [qp.hpp](/src/controller/qp.hpp) / [qp.cpp](/src/controller/qp.cpp) - Quadratic program solver abstraction (incomplete).
    - [trajectory.hpp](/src/controller/trajectory.hpp) / [trajectory.cpp](/src/controller/trajectory.cpp) - Various spatial and angular trajectories over time.
  - [frankaridgeback](/src/frankaridgeback/) - Implementation of the franka research 3 mounted to the clearpath ridgeback.
    - [model](/src/frankaridgeback/model) - URDF and object files defining system joints, links and geometry.
    - [objective](/src/frankaridgeback/objective) - Objective functions.
      - [assisted_manipulation.hpp](/src/frankaridgeback/objective/assisted_manipulation.hpp) / [assisted_manipulation.cpp](/src/frankaridgeback/objective/assisted_manipulation.cpp) - The assisted manipulation objective function implementation.
      - [track_point.hpp](/src/frankaridgeback/objective/track_point.hpp) / [track_point.cpp](/src/frankaridgeback/objective/track_point.cpp) - Simple end-effector to point tracking objective function.
    - [control.hpp](/src/frankaridgeback/control.hpp) - Definition of the variables required as control inputs to the robot.
    - [dof.hpp](/src/frankaridgeback/dof.hpp) - Defintions of degrees of freedom in the robot.
    - [dynamics.hpp](/src/frankaridgeback/dynamics.hpp) - An abstract declaration of the franka ridgeback MPPI dynamics. Implemented with RaiSim and pinocchio.
    - [pinocchio_dynamics.hpp](/src/frankaridgeback/pinocchio_dynamics.hpp) / [pinocchio_dynamics.cpp](/src/frankaridgeback/pinocchio_dynamics.cpp) - Implementation of the robot dynamics using pinocchio. Currently broken due to the forward integration step failing / no floor.
    - [safety.hpp](/src/frankaridgeback/safety.hpp) / [safety.cpp](/src/frankaridgeback/safety.cpp) - Unfinished safety filter constraints on the robot.
    - [state.hpp](/src/frankaridgeback/state.hpp) / [state.cpp](src/frankaridgeback/state.cpp) - Definition of the variables defining the robot state.
  - [logging](/src/logging) - Logging utilities, independent of other code.
    - [assisted_manipulation.hpp](/src/logging/assisted_manipulation.hpp) / [assisted_manipulation.cpp](/src/logging/assisted_manipulation.cpp) - Logging of the assisted manipulation algorithm costs.
    - [csv.hpp](/src/logging/csv.hpp) - CSV file logging implementation.
    - [file.hpp](/src/logging/file.hpp) - Generic file creation and writing.
    - [frankaridgeback.hpp](/src/logging/frankaridgeback.hpp) / [frankaridgeback.cpp](/src/logging/frankaridgeback.cpp) - Frankaridgeback state logging.
    - [mppi.hpp](/src/logging/mppi.hpp) / [mppi.cpp](/src/logging/mppi.cpp) - Logging of rollout costs, weights, update duration, optimal rollouts of MPPI algorithm.
    - [pid.hpp](/src/logging/pid.hpp) / [pid.cpp](/src/logging/pid.cpp) - Logging of pid reference, error, cumulative error, saturation and output control.
  - [simulation](/src/simulation) - All code referencing the RaiSim simulator.
    - [frankaridgeback](/src/frankaridgeback) - RaiSim implementation of the Frankaridgeback.
      - [actor_dynamics.hpp](/src/simulation/frankaridgeback/actor_dynamics.hpp) / [actor_dynamics.cpp](/src/simulation/frankaridgeback/actor_dynamics.cpp) - The dynamics of the actual Frankaridgeback actor in the world. Not the same as the dynamics used by the MPPI controller. Includes an abstract class `ActorDynamics` which is implemented by `RaisimActorDynamics` for simulating the robot with raisim directly, and `PinocchioActorDynamics` for simulating the robot physics with pinocchio while using RaiSim visualisation.
      - [actor.hpp](/src/simulation/frankaridgeback/actor.hpp) / [actor.cpp](/src/simulation/frankaridgeback/actor.cpp) - The FrankaRidgeback actor in the world simulation. The composition of all Frankaridgeback components into a single entity used in the simulation. Includes MPPI controller, forecast of the dynamics for the objective function, and `ActorDynamics` for world simulation.
      - [dynamics.hpp](/src/simulation/frankaridgeback/dynamics.hpp) / [dynamics.cpp](/src/simulation/frankaridgeback/dynamics.cpp) - Used to instantiate `RaisimDynamics` or `PinocchioDynamics` from json configuration.
      - [raisim_dynamics.hpp](/src/simulation/frankaridgeback/raisim_dynamics.hpp) / [raisim_dynamics.cpp](/src/simulation/frankaridgeback/raisim_dynamics.cpp) - Implementation of the Frankaridgeback dynamics using the RaiSim simulator.
    - [simulator.hpp](/src/simulation/simulator.hpp) / [simulator.cpp](/src/simulation/simulator.cpp) - Wrapper around the RaiSim simulator with additional utilities and actor abstraction.
  - [test](/src/test) - The implementation of the test simulations.
    - [case] - Different cases.
      - [angles.hpp](/src/test/case/angles.hpp) - Unit test checking angular transformation code is correct. Unused.
      - [base.hpp](src/test/case/base.hpp) / [base.cpp](src/test/case/base.cpp) - The base test case which containing the primary test program logic. Has instances of the `Simulator`, `FrankaRidgeback::Actor` and loggers. The main loop of the test program is in [`BaseTest::run()`](/src/test/case/base.cpp#L150) calling [`BaseTest::step()`](src/test/case/base.cpp#L128). Also contains the default configurations.
      - [circle.hpp](/src/test/case/circle.hpp) - Externally applied wrench in a circular trajectory.
      - [external_wrench.hpp](/src/test/case/external_wrench.hpp) - 
 