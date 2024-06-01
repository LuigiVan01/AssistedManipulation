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
    application/main.cpp
    application/dynamics.cpp
    application/cost.cpp
)

target_include_directories(assistedmanipulation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(assistedmanipulation PUBLIC raisim::raisim pinocchio::pinocchio)

# Additional logic depending on the operating system.
if (UNIX)
    target_link_libraries(assistedmanipulation PUBLIC pthread)
elseif (WIN32)
    target_compile_options(
        assistedmanipulation
        PUBLIC
        /wd5045 # Compiler will insert Spectre mitigation for memory load if /Qspectre switch specified
        /wd4711
        /wd4710 # 'function': function not inlined
        /wd5031 # #pragma warning(pop): likely mismatch, popping warning state pushed in different file 
        /wd4464 # 'bytes' bytes padding added after construct 'member_name'
        /wd4820 # bytes padded after base class
        /wd4625 # copy constructor implicitly deleted
        /wd4626 # assignment operator implicitly deleted
        /wd4996 # getenv function or variable may be unsafe
    )
    target_link_libraries(assistedmanipulation PUBLIC Ws2_32)
endif ()
