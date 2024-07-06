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
    assistedmanipulation
    # tests/model.cpp
    main.cpp
    mppi.cpp
    dynamics.cpp
    cost.cpp
    controller.cpp
    simulator.cpp
    filter/filter.cpp
    filter/gram_savitzky_golay.cpp
)

target_include_directories(assistedmanipulation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(assistedmanipulation PUBLIC raisim::raisim pinocchio::pinocchio)

# Additional logic depending on the operating system.
if (UNIX)
    target_link_libraries(assistedmanipulation PUBLIC pthread)
elseif (WIN32)
    target_compile_options(assistedmanipulation PUBLIC /W3)
    target_link_libraries(assistedmanipulation PUBLIC Ws2_32)
endif ()

# Install instructions

install(TARGETS assistedmanipulation DESTINATION bin)
install(DIRECTORY model DESTINATION bin)

# Copy runtime dlls on windows.
if (WIN32)
    install(FILES $<TARGET_RUNTIME_DLLS:assistedmanipulation> DESTINATION bin)
endif ()
