#!/bin/bash

set -e

if [ $# -ne 1 ]; then
    echo "The source directory is required."
    exit 1
fi

WORKSPACE=`pwd`
SOURCE=$1

# Add library directories to cmake path.
CMAKE_PREFIX_PATH+=";$WORKSPACE/lib/raisimlib/raisim/linux"
export CMAKE_PREFIX_PATH

# Add raisim linker directories to path.
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$WORKSPACE/raisim/linux/lib
export PYTHONPATH=$PYTHONPATH:$WORKSPACE/raisim/linux/lib

cmake \
    -DCMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH \
    -DCMAKE_FIND_USE_CMAKE_ENVIRONMENT_PATH=y \
    -B build \
    -S $SOURCE

cmake --build build
