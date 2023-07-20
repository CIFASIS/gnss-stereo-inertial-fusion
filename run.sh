#!/bin/bash
# TODO doc

set -eE # Any subsequent commands which fail will cause the shell script to exit immediately

# Get full directory name of the script no matter where it is being called from
CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

OUTPUT_DIR=$CURRENT_DIR/output/$(date '+%Y%m%d_%H%M%S')

function echoUsage()
{
    echo -e "Usage: $0 [FLAG] ROSBAG\n\
            \t -o path to output folder \n\
            \t -s path to ORB-SLAM3 yaml configuration file \n\
            \t -l launchfile
            \t -h help" >&2
}

SETTINGS_FILE=$CURRENT_DIR/Examples/Stereo-Inertial/rosario_dataset/Rosario_3_0.yaml
LAUNCH_FILE=$CURRENT_DIR/Examples/ROS/ORB_SLAM3/launch/rosario.launch
while getopts "l:s:ho:" opt; do
    case "$opt" in
        s)  case $OPTARG in
                -*) echo "ERROR: a path to settings-file must be provided"; echoUsage; exit 1 ;;
                *) SETTINGS_FILE=$OPTARG ;;
            esac
            ;;
        l)  case $OPTARG in
                -*) echo "ERROR: a path to settings-file must be provided"; echoUsage; exit 1 ;;
                *) LAUNCH_FILE=$OPTARG ;;
            esac
            ;;            
        h)  echoUsage
            exit 0
            ;;
        o)  case $OPTARG in
                -*) echo "ERROR: a path to output directory must be provided"; echoUsage; exit 1 ;;
                *) OUTPUT_DIR=$OPTARG ;;
            esac
            ;;
        *)
            echoUsage
            exit 1
            ;;
    esac
done

shift $((OPTIND -1))
BAG=$1

mkdir -p $OUTPUT_DIR
echo "Create $OUTPUT_DIR"
echo "Using settings: $SETTINGS_FILE"
echo "Using launchfile: $LAUNCH_FILE"


roslaunch $CURRENT_DIR/launch/play_bag_and_run.launch bagfile:=$1 settings:=$SETTINGS_FILE launchfile:=$LAUNCH_FILE &
P1=$!

# Wait some seconds until roscore is running...
sleep 3
LOG_OUTPUT_DIR=$(roslaunch-logs)

# Wait for roslaunch
wait $P1

# Save trajectory file
TRAJECTORY_FILE=$HOME/.ros/CameraTrajectoryGPSOpt.txt
mv $TRAJECTORY_FILE $OUTPUT_DIR

# Save log file
cp $LOG_OUTPUT_DIR/orbslam3*.log $OUTPUT_DIR

echo "Results saved to $OUTPUT_DIR"
