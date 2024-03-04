cmake_minimum_required(VERSION 3.10)
project(assistedmanipulation)

# Display search paths for libraries.
# set(CMAKE_FIND_DEBUG_MODE 1)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(Eigen3 REQUIRED)
find_package(raisim CONFIG REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(assistedmanipulation hello.cpp)
target_link_libraries(assistedmanipulation PUBLIC raisim::raisim pthread)
target_include_directories(assistedmanipulation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
