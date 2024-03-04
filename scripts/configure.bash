rm -rf bin lib
mkdir -p bin lib

wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip -O eigen.zip
unzip eigen.zip -d lib/eigen
rm -rf eigen.zip

cd lib
git clone https://github.com/raisimTech/raisimlib
git clone https://github.com/ethz-asl/sampling_based_control
