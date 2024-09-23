cmake_minimum_required(VERSION 3.10)

# set(CMAKE_FIND_DEBUG_MODE 1)
project(assistedmanipulation)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# Display search paths for libraries.

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
# find_package(osqp REQUIRED)
find_package(raisim CONFIG REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS} ${yaml_cpp_INCLUDE_DIRS} ${pinnocchio_INCLUDE_DIRS})

add_executable(
    test
    test/main.cpp
    # test/case/base/reach.cpp
    test/case/base.cpp
    # test/case/base/no_manipulation.cpp
    # test/case/power_minimisation.cpp
    # test/case/manipulability.cpp
    # test/case/pinocchio.cpp

    simulation/simulator.cpp
    simulation/actors/frankaridgeback.cpp
    simulation/raisim_dynamics.cpp

    controller/gram_savitzky_golay/gram_savitzky_golay.cpp
    controller/filter.cpp
    controller/mppi.cpp
    controller/kalman.cpp
    controller/forecast.cpp
    # controller/qp.cpp

    # frankaridgeback/dynamics.cpp
    # frankaridgeback/objective/track_point.cpp
    frankaridgeback/objective/assisted_manipulation.cpp
    frankaridgeback/pinocchio_dynamics.cpp

    logging/mppi.cpp
    logging/pid.cpp
)

target_include_directories(test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if (UNIX)
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 10)
        message("gcc version <= 10, currently using version ${CMAKE_CXX_COMPILER_VERSION}")
    endif()

    target_link_libraries(
        test PUBLIC 
        raisim::raisim
        pinocchio::pinocchio
        # osqp::osqp
        nlohmann_json::nlohmann_json
        pthread
        -static
    )
elseif (WIN32)
    target_compile_options(test PUBLIC /W3)

    target_link_libraries(
        test PUBLIC 
        raisim::raisim
        pinocchio::pinocchio
        # osqp::osqp
        nlohmann_json::nlohmann_json
        Ws2_32
    )

    install(FILES $<TARGET_RUNTIME_DLLS:test> DESTINATION bin)
endif()

# Install instructions
install(TARGETS test DESTINATION bin)
install(DIRECTORY frankaridgeback/model DESTINATION bin)
