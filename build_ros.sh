echo "Building ROS nodes"

cd Examples_added/ROS/ORB_SLAM3_Dev_ROS
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j50
