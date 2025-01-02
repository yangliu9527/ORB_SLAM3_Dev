cmake_minimum_required(VERSION 3.5)

# 项目名称
project(ORB_SLAM3_Dev_ROS2)


# 查找 ROS 2 包和其他依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)


# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译选项
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")
add_definitions(-DCOMPILEDWITHC14)

# 自定义模块路径
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

message("Project Source DIR = ${PROJECT_SOURCE_DIR}")
# 包含目录
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/../../../../
  ${PROJECT_SOURCE_DIR}/../../../../include
  ${PROJECT_SOURCE_DIR}/../../../../include/CameraModels
  ${PROJECT_SOURCE_DIR}/../../../../Thirdparty/Sophus
  ${Pangolin_INCLUDE_DIRS}
)

# 依赖的库
set(LIBS 
  ${OpenCV_LIBS} 
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/../../../../Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/../../../../Thirdparty/g2o/lib/libg2o.so
  ${PROJECT_SOURCE_DIR}/../../../../lib/libORB_SLAM3.so
  -lboost_system
)


set(ros2_dependencies
rclcpp
sensor_msgs
std_msgs
rosgraph_msgs
cv_bridge
message_filters
)

# message("libraries: ${rclcpp_LIBRARIES}")
# 添加节点
add_executable(Mono src/ros2_mono.cc)
ament_target_dependencies(Mono PUBLIC ${ros2_dependencies})
target_link_libraries(Mono PUBLIC ${LIBS}) 

add_executable(Stereo src/ros2_stereo.cc)
ament_target_dependencies(Stereo PUBLIC ${ros2_dependencies})
target_link_libraries(Stereo PUBLIC ${LIBS})

add_executable(RGBD src/ros2_rgbd.cc)
ament_target_dependencies(RGBD PUBLIC ${ros2_dependencies})
target_link_libraries(RGBD PUBLIC ${LIBS})

add_executable(Mono_Inertial src/ros2_mono_inertial.cc)
ament_target_dependencies(Mono_Inertial PUBLIC ${ros2_dependencies})
target_link_libraries(Mono_Inertial PUBLIC ${LIBS})

add_executable(Stereo_Inertial src/ros2_stereo_inertial.cc)
ament_target_dependencies(Stereo_Inertial PUBLIC ${ros2_dependencies})
target_link_libraries(Stereo_Inertial PUBLIC ${LIBS})

add_executable(RGBD_Inertial src/ros2_rgbd_inertial.cc)
ament_target_dependencies(RGBD_Inertial PUBLIC ${ros2_dependencies})
target_link_libraries(RGBD_Inertial PUBLIC ${LIBS})

# 安装目标
install(TARGETS 
  Mono
  Stereo
  RGBD
  Mono_Inertial
  Stereo_Inertial
  RGBD_Inertial
  DESTINATION lib/${PROJECT_NAME}
)

# ROS 2 包设置
ament_package()
