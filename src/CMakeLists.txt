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
find_package(raisim CONFIG REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS} ${yaml_cpp_INCLUDE_DIRS})

add_executable(assistedmanipulation main.cpp)
target_include_directories(assistedmanipulation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Additional logic depending on the operating system.
if (UNIX)
    target_link_libraries(assistedmanipulation PUBLIC raisim::raisim pthread)
elseif (WIN32)
    target_link_libraries(assistedmanipulation PUBLIC raisim::raisim Ws2_32)
    set(CMAKE_FIND_LIBRARY_SUFFIXES ".lib")

    target_compile_options(assistedmanipulation PRIVATE /MT "/wd5045")
    # target_link_options(assistedmanipulation PRIVATE /INCREMENTAL:NO /NODEFAULTLIB:MSVCRT)
endif ()

