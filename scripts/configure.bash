rm -rf bin lib
mkdir -p bin lib

ROOT=`pwd`

# Download and make eigen.
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip -O eigen.zip
unzip eigen.zip
rm eigen.zip
mv eigen-3.4.0 eigen
cd eigen && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX="$ROOT/lib/eigen"
make install
cd $ROOT
rm -rf eigen

# cd lib
git clone https://github.com/raisimTech/raisimlib
git clone https://github.com/ethz-asl/sampling_based_control
