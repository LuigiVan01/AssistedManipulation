cmake_minimum_required(VERSION 3.10)

# set(CMAKE_FIND_DEBUG_MODE 1)
project(assistedmanipulation)

set(CMAKE_CXX_STANDARD 20) #Tells the compiler to use C++20 features and syntax
set(CMAKE_CXX_STANDARD_REQUIRED ON) #If the compiler doesn't support C++20, the build will fail
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall") #Enables all standard warning messages during compilation

# Display search paths for libraries.

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/openrobots")

# Find the required OSQP packages
include_directories(${OSQP_FOLDER}/include)
link_directories(${OSQP_FOLDER}/lib)

# Find the required Eigen3, Pinocchio, Raisim, and nlohmann_json packages
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(raisim CONFIG REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR} ${PINOCCHIO_INCLUDE_DIRS})


# Print initial compiler flags
message(STATUS "Initial CXX flags: ${CMAKE_CXX_FLAGS}")


# Specify the executable and all the source files that should be compiled and linked top it
# Order doesn't matter
add_executable(
    test #Executable name
    test/main.cpp

    controller/filter.cpp
    controller/forecast.cpp
    controller/gram_savitzky_golay/gram_savitzky_golay.cpp
    controller/kalman.cpp
    controller/mppi.cpp
    controller/pid.cpp
    controller/trajectory.cpp
    controller/qp.cpp
    controller/filterqp.cpp

    # test/case/base/reach.cpp
    
    test/case/base.cpp
    test/case/external_wrench.cpp
    test/case/forecast.cpp
    test/case/trajectory.cpp
    # test/case/pinocchio.cpp

    simulation/frankaridgeback/raisim_dynamics.cpp
    simulation/frankaridgeback/dynamics.cpp
    simulation/frankaridgeback/actor.cpp
    simulation/frankaridgeback/actor_dynamics.cpp
    simulation/simulator.cpp

    frankaridgeback/state.cpp
    frankaridgeback/objective/track_point.cpp
    frankaridgeback/objective/assisted_manipulation.cpp
    frankaridgeback/pinocchio_dynamics.cpp
    frankaridgeback/dynamics.cpp

    logging/assisted_manipulation.cpp
    logging/frankaridgeback.cpp
    logging/mppi.cpp
    logging/pid.cpp
)

target_include_directories(test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if (UNIX)
    message(STATUS "Configuring for Unix/Linux with GCC")

    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 10)
        message("gcc version <= 10, currently using version ${CMAKE_CXX_COMPILER_VERSION}")
    endif()


    # Create a list of MSVC compatibility flags
    set(MSVC_COMPAT_FLAGS
       -fms-extensions            # Enable Microsoft extensions
       -fms-compatibility        # Enable Microsoft compatibility
       -fms-compatibility-version=19.29  # Match VS 2019 behavior
       -fno-threadsafe-statics   # MSVC-like static initialization
       -fno-implicit-templates   # MSVC-like template handling
       -fno-implicit-inline-templates  # More MSVC-like template behavior
   )

   # Add the flags to the compile options
   add_compile_options(${MSVC_COMPAT_FLAGS})
   
   # Print the flags to verify they're set
   message(STATUS "Added MSVC compatibility flags: ${MSVC_COMPAT_FLAGS}")


    # Add threading flags
    add_compile_definitions(
        _REENTRANT 
        FORCE_SEQUENTIAL_INIT
    )
    message(STATUS "Added threading definitions")

    target_link_libraries(
        test PUBLIC 
        Eigen3::Eigen
        raisim::raisim
        pinocchio::pinocchio
        ${OSQP_FOLDER}/lib/libosqp.so 
        nlohmann_json::nlohmann_json
        pthread
    )
    
elseif (WIN32)
    message(STATUS "Configuring for Windows with MSVC")
    target_compile_options(test PUBLIC /W3) # /Wall WX

    target_link_libraries(
        test PUBLIC 
        raisim::raisim
        pinocchio::pinocchio
        ${OSQP_FOLDER}/lib/libosqp.so
        nlohmann_json::nlohmann_json
        Ws2_32
    )

    install(FILES $<TARGET_RUNTIME_DLLS:test> DESTINATION bin)
endif()

# Install instructions
install(TARGETS test DESTINATION bin)
install(DIRECTORY frankaridgeback/model DESTINATION bin)


set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)