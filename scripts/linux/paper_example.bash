set -e

function usage()
{
    printf "usage: $0 [-b] [-r <string>]\n"
    exit 1
}

BUILD=
RUN=

while getopts "br" o; do
    case "${o}" in
        b)
            BUILD=true
            ;;
        r)
            RUN=${OPTARG}
            ;;
        *)
            usage
            ;;
    esac
done

if [ ! $BUILD ] && [ ! $RUN ]; then
    usage
    exit 1
fi

if [ $BUILD ]; then

    # Regenerate the catkin workspace.
    rm -rf reference_paper_example
    mkdir -p reference_paper_example/src

    cd reference_paper_example
    catkin build

    source devel/setup.bash

    WORKSPACE=`pwd`
    SOURCE=$WORKSPACE/src

    # Builds a cmake directory.
    function build
    {
        cd $SOURCE/$1 && mkdir build && cd build
        cmake -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic ..
        cmake --build . --target install
        source devel/setup.bash
        cd $SOURCE
    }

    cd $SOURCE

    # Download dependancies.
    git clone git@github.com:ANYbotics/kindr_ros.git
    git clone git@github.com:ANYbotics/kindr.git
    git clone git@github.com:ANYbotics/message_logger.git
    git clone git@github.com:ANYbotics/signal_logger.git

    # Build and install dependancies
    build kindr
    build kindr_ros/kindr_msgs
    build kindr_ros/kindr_ros
    build message_logger
    build signal_logger/signal_logger_core
    build signal_logger/signal_logger_msgs
    build signal_logger/signal_logger_std
    build signal_logger/signal_logger_ros

    # git clone --recursive https://github.com/ethz-asl/sampling_based_control.git
fi

if [ $RUN ]; then
    echo running
fi
