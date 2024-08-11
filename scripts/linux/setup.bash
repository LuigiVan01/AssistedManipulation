set -e

# rm -rf bin lib
mkdir -p bin lib

ROOT=`pwd`

# Download and make yaml-cpp.
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp && mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX="$ROOT/lib/yaml-cpp"
mkdir -p "$ROOT/lib/yaml-cpp"
make && make install
cd $ROOT && rm -rf yaml-cpp
sudo apt install libeigen3-dev

# cd lib
# git clone https://github.com/raisimTech/raisimlib
# git clone https://github.com/ethz-asl/sampling_based_control
