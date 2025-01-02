#=========common command==========#
roscore

#============ROS nodes==========#
#RGB-D Inertial 
rosrun ORB_SLAM3_Dev_ROS RGBD_Inertial Vocabulary/ORBvoc.txt Examples_added/ROS/ORB_SLAM3_Dev_ROS/D455/D455_RGBDi_factory.yaml
#RGB-D
rosrun ORB_SLAM3_Dev_ROS RGBD Vocabulary/ORBvoc.txt Examples_added/ROS/ORB_SLAM3_Dev_ROS/D455/D455_RGBDi_factory.yaml
#Mono 
rosrun ORB_SLAM3_Dev_ROS Mono Vocabulary/ORBvoc.txt Examples/Monocular-Inertial/EuRoC.yaml /camera/image_raw:=/cam0/image_raw
#Mono Inertial
rosrun ORB_SLAM3_Dev_ROS Mono_Inertial Vocabulary/ORBvoc.txt Examples/Monocular-Inertial/EuRoC.yaml /camera/infra1/image_rect_raw:=/cam0/image_raw /camera/imu:=/imu0
#Stereo 
rosrun ORB_SLAM3_Dev_ROS Stereo Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/EuRoC.yaml false /camera/left/image_raw:=/cam0/image_raw /camera/right/image_raw:=/cam1/image_raw
#Stereo  Inertial
rosrun ORB_SLAM3_Dev_ROS Stereo_Inertial Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/EuRoC.yaml /camera/infra1/image_rect_raw:=/cam0/image_raw /camera/infra2/image_rect_raw:=/cam1/image_raw /camera/imu:=/imu0 


#============ROS2 nodes==========#
ros2 run ORB_SLAM3_Dev_ROS2 Mono_Inertial Vocabulary/ORBvoc.txt Examples/Monocular-Inertial/EuRoC.yaml 
ros2 run ORB_SLAM3_Dev_ROS2 Stereo_Inertial Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/EuRoC.yaml 
 

