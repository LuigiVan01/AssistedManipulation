set -e

ROOT=`pwd`
SOURCE=$ROOT/lib/sampling_based_control/mppi_examples
DESTINATION=$ROOT/src/model

rm -rf $DESTINATION/*

cp $SOURCE/mppi_manipulation/data/panda_mobile_fixed.urdf $DESTINATION/robot.urdf

# Copy required urdf build definition files.
# cp -t $DESTINATION \
#   $SOURCE/mppi_manipulation/data/panda_mobile_fixed.urdf.xacro \
#         $SOURCE/mppi_panda/resources/panda/panda.urdf.xacro \
#         $SOURCE/mppi_manipulation/data/planar_base.urdf.xacro \
#         $SOURCE/mppi_manipulation/data/ridgeback.urdf.xacro \
#             $SOURCE/mppi_manipulation/data/ridgeback.xacro \

# Copy dependancies.
mkdir -p $DESTINATION/mppi_panda_resources/resources/panda
mkdir -p $DESTINATION/mppi_manipulation_resources/data

cp -R $SOURCE/mppi_panda/resources/panda/* -t $DESTINATION/mppi_panda_resources/resources/panda
cp -R $SOURCE/mppi_manipulation/data/* -t $DESTINATION/mppi_manipulation_resources/data

cd $DESTINATION

# Update resource locations.
sed -i "s|filename=\".*/mppi_panda\(.*\)\"|filename=\"mppi_panda_resources\1\"|g" robot.urdf
sed -i "s|filename=\".*/mppi_manipulation\(.*\)\"|filename=\"mppi_manipulation_resources\1\"|g" robot.urdf
sed -i "s|filename=\".*/ridgeback_description\(.*\)\"|filename=\"mppi_manipulation_resources/data/meshes/ridgeback\1\"|g" robot.urdf

# xacro panda_mobile.urdf.xacro -o panda_mobile.urdf
