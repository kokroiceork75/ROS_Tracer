cmake_minimum_required(VERSION 3.0.2)
project(velodyne_gazebo_plugins)

# 添加 SDFormat 的尋找
find_package(SDFormat9 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  gazebo_ros
)

find_package(gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include ${GAZEBO_INCLUDE_DIRS}
  LIBRARIES gazebo_ros_velodyne_laser gazebo_ros_velodyne_gpu_laser
  CATKIN_DEPENDS roscpp sensor_msgs gazebo_ros
)

# 修改 include_directories，添加 SDFormat 路徑
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat9_INCLUDE_DIRS}
  /usr/include/sdformat-9.10
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${SDFormat9_LIBRARY_DIRS}  # 添加 SDFormat 庫路徑
)

add_library(gazebo_ros_velodyne_laser src/GazeboRosVelodyneLaser.cpp)
target_link_libraries(gazebo_ros_velodyne_laser
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${SDFormat9_LIBRARIES}  # 添加 SDFormat 庫
  RayPlugin
)

add_library(gazebo_ros_velodyne_gpu_laser src/GazeboRosVelodyneLaser.cpp)
target_link_libraries(gazebo_ros_velodyne_gpu_laser
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${SDFormat9_LIBRARIES}  # 添加 SDFormat 庫
  GpuRayPlugin
)

target_compile_definitions(gazebo_ros_velodyne_gpu_laser PRIVATE GAZEBO_GPU_RAY=1)

install(TARGETS gazebo_ros_velodyne_laser gazebo_ros_velodyne_gpu_laser
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
