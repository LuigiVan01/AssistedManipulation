set -e

rm -rf catkin
mkdir -p catkin/src

cd catkin
catkin_init

cd src
git clone --recursive https://github.com/ethz-asl/sampling_based_control.git

cd ..
rosdep init
rosdep install --from-paths src/sampling_based_control --ignore-src -y
