set -e

# rm -rf bin lib
mkdir -p bin lib

ROOT=`pwd`

sudo apt install libeigen3-dev

cd lib
git clone https://github.com/raisimTech/raisimlib
git clone https://github.com/ethz-asl/sampling_based_control
#TODO: ADD source OSQP library here and modify CMAKEfile to handle that 
# currently I downloaded the binaries (version 0.6.3) for linux from here https://github.com/osqp/osqp/releases
# but for future usage if the repo is better to clone the source files and build locally
cd ..
