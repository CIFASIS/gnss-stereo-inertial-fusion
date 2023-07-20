#!/bin/bash
#
# Run Rosario dataset (from sequence 01 to 06)
# Parameter:
#   -Path of folder containing rosbags (files must be named sequence0*.bag)

# Get full directory name of the script no matter where it is being called from
CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

if [ -z "${SEQUENCES}" ] ; then
  SEQUENCES=($(seq 1 6)) 
fi

DATASET_DIR=$1

dt=$(date '+%Y%m%d_%H%M%S')
OUTPUT_DIR=$CURRENT_DIR/output/rosario_${dt}
mkdir -p $OUTPUT_DIR

trap "exit 1" INT

for i in ${SEQUENCES[@]} ; do
  echo "Running on: $DATASET_DIR/sequence0$i.bag"
  $CURRENT_DIR/run.sh -o $OUTPUT_DIR/sequence0$i/ $DATASET_DIR/sequence0$i.bag 
done

for d in $OUTPUT_DIR/*/ ; do 
  cd $d  
  evo_traj tum CameraTrajectoryGPSOpt.txt --transform_right $CURRENT_DIR/Examples/Stereo-Inertial/rosario_dataset/t_b_g.json --save_as_tum
done

EVAL_SCRIPT=$OUTPUT_DIR/eval_sequences.sh
touch $EVAL_SCRIPT
chmod +x $EVAL_SCRIPT
echo "CURRENT_DIR=\"\$( cd \"\$( dirname \"\${BASH_SOURCE[0]}\" )\" >/dev/null 2>&1 && pwd )\"" >> $EVAL_SCRIPT
for i in ${SEQUENCES[@]} ; do
  echo "echo \"sequence0$i\"" >> $EVAL_SCRIPT
  echo "evo_ape tum $DATASET_DIR/sequence0${i}_gt.txt \$CURRENT_DIR/sequence0${i}/CameraTrajectoryGPSOpt.tum -a --t_max 0.03" >> $EVAL_SCRIPT
done

