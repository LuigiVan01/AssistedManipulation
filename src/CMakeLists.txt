cmake_minimum_required(VERSION 3.10)

# set(CMAKE_FIND_DEBUG_MODE 1)
project(assistedmanipulation)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# Display search paths for libraries.

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(raisim CONFIG REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS} ${yaml_cpp_INCLUDE_DIRS} ${pinnocchio_INCLUDE_DIRS})

add_executable(
    test
    test/main.cpp
    test/case/reach.cpp
    # test/case/circle.cpp

    simulation/simulator.cpp
    simulation/actors/frankaridgeback.cpp

    controller/gram_savitzky_golay/gram_savitzky_golay.cpp
    controller/filter.cpp
    controller/mppi.cpp

    frankaridgeback/dynamics.cpp
    frankaridgeback/model.cpp
    frankaridgeback/objective/point.cpp
    frankaridgeback/objective/assisted_manipulation.cpp

    logging/mppi.cpp
    logging/pid.cpp
)

target_include_directories(test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(test PUBLIC raisim::raisim pinocchio::pinocchio)

# Additional logic depending on the operating system.
if (UNIX)
    target_link_libraries(test PUBLIC pthread)
elseif (WIN32)
    target_compile_options(test PUBLIC /W3)
    target_link_libraries(test PUBLIC Ws2_32)
endif ()

# Install instructions

install(TARGETS test DESTINATION bin)
install(DIRECTORY frankaridgeback/model DESTINATION bin)

# Copy runtime dlls on windows.
if (WIN32)
    install(FILES $<TARGET_RUNTIME_DLLS:test> DESTINATION bin)
endif ()
