echo "Building ROS nodes"

cd Examples_added/ROS2_ws/
rm -rf build install log
#colcon build --symlink-install --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
colcon build --symlink-install
