WORKSPACE=`pwd`
export CMAKE_PREFIX_PATH="$WORKSPACE/lib/eigen;$WORKSPACE/lib/raisimlib/raisim/linux"

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$WORKSPACE/raisim/linux/lib
export PYTHONPATH=$PYTHONPATH:$WORKSPACE/raisim/linux/lib

cmake \
    -DCMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH \
    -DCMAKE_FIND_USE_CMAKE_ENVIRONMENT_PATH=y \
    -B build \
    -S src

cmake --build build
