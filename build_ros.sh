echo "Building ROS nodes"

cd Examples/ROS/GNSS_SI
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release #-DGPS_DROPOUT=ON
make -j
