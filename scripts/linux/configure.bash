#!/bin/bash

WORKSPACE=$(pwd)


echo "Build type is: $1"

# Add library directories to cmake path.
CMAKE_PREFIX_PATH+=";$WORKSPACE/lib/raisimlib/raisim/linux"
export CMAKE_PREFIX_PATH

# Add raisim linker directories to path.
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$WORKSPACE/raisim/linux/lib
export PYTHONPATH=$PYTHONPATH:$WORKSPACE/raisim/linux/lib

mkdir -p build
mkdir -p install  

cmake -DCMAKE_BUILD_TYPE=$1 -DCMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH -DCMAKE_FIND_USE_CMAKE_ENVIRONMENT_PATH=y -DCMAKE_INSTALL_PREFIX=${WORKSPACE}/install -B build -S src