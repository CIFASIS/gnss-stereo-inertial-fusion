#!/bin/bash
# TODO doc

set -eE # Any subsequent commands which fail will cause the shell script to exit immediately

# Get full directory name of the script no matter where it is being called from
CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

DATASET_DIR=$1

dt=$(date '+%Y%m%d_%H%M%S')
OUTPUT_DIR=$CURRENT_DIR/output/zavalla_${dt}
mkdir -p $OUTPUT_DIR

SETTINGS_FILE=$CURRENT_DIR/Examples/Stereo-Inertial/zavalla_2021/zavalla.yaml
LAUNCH_FILE=$CURRENT_DIR/Examples/ROS/ORB_SLAM3/launch/zavalla.launch

trap "exit 1" INT

for bag in $DATASET_DIR/sequence*.bag ; do
  echo "Running on: $bag"
  filename=$(basename -- "$bag")
  extension="${filename##*.}"
  filename="${filename%.*}"
  $CURRENT_DIR/run.sh -o $OUTPUT_DIR/$filename/ -l $LAUNCH_FILE -s $SETTINGS_FILE $bag
done

for d in $OUTPUT_DIR/*/ ; do 
  cd $d  
  evo_traj tum CameraTrajectoryGPSOpt.txt --transform_right $CURRENT_DIR/Examples/Stereo-Inertial/zavalla_2021/t_b_g.json --save_as_tum
done


