set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")
find_package(OpenCV REQUIRED)
cmake_minimum_required(VERSION 3.10)
project(controller)

add_compile_options(-std=c++11)

# 查找 catkin 和相關組件
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  nav_core
  costmap_2d
  tf2_ros
  pluginlib
  base_local_planner
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
  cv_bridge
  OpenCV
  image_transport
)
find_package(yaml-cpp REQUIRED)

# 引入 Conan 生成的依賴
include(/home/user/wei_ws/src/3rd/conanbuildinfo.cmake)
conan_basic_setup(NO_OUTPUT_DIRS)

# 查找 PCL 和 Eigen3
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)


# 配置 catkin 包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mpc_controller_plugin
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs pcl_ros pcl_conversions nav_core costmap_2d tf2_ros pluginlib base_local_planner
)

# 指定頭文件路徑
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CONAN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 編譯 mpc_controller_plugin 作為共享庫
add_library(mpc_controller_plugin src/mpc_controller.cpp src/controller.cpp)
target_link_libraries(mpc_controller_plugin
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${CONAN_LIBS}
)

# 現有可執行文件
add_executable(move_along_tracer_4_side src/move_along_tracer_4_side.cpp)
target_link_libraries(move_along_tracer_4_side ${catkin_LIBRARIES})
add_executable(move_along_tracer_4 src/move_along_tracer_4.cpp)
target_link_libraries(move_along_tracer_4 ${catkin_LIBRARIES})
add_executable(odom_new src/odom_new.cpp)
target_link_libraries(odom_new ${catkin_LIBRARIES})
add_executable(sim_along_wall_p3dx_2 src/sim_along_wall_p3dx_2.cpp)
target_link_libraries(sim_along_wall_p3dx_2 ${catkin_LIBRARIES})
add_executable(sub_pub_cmd_test_new src/sub_pub_cmd_test_new.cpp)
target_link_libraries(sub_pub_cmd_test_new ${catkin_LIBRARIES})
add_executable(sim_along_wall_tracer src/sim_along_wall_tracer.cpp)
target_link_libraries(sim_along_wall_tracer ${catkin_LIBRARIES})
add_executable(sim_along_5input src/sim_along_5input.cpp)
target_link_libraries(sim_along_5input ${catkin_LIBRARIES})
add_executable(sim_along_4input_side src/sim_along_4input_side.cpp)
target_link_libraries(sim_along_4input_side ${catkin_LIBRARIES})
add_executable(default_waypoint src/default_waypoint.cpp)
target_link_libraries(default_waypoint ${catkin_LIBRARIES})
add_executable(fuzzy_waypoint src/fuzzy_waypoint.cpp)
target_link_libraries(fuzzy_waypoint ${catkin_LIBRARIES})
add_executable(communication_node src/communication_node.cpp)
target_link_libraries(communication_node ${catkin_LIBRARIES})
add_executable(move_along_one_narrow src/move_along_one_narrow.cpp)
target_link_libraries(move_along_one_narrow ${catkin_LIBRARIES})
add_executable(move_along_one_narrow_side src/move_along_one_narrow_side.cpp)
target_link_libraries(move_along_one_narrow_side ${catkin_LIBRARIES})
add_executable(an_move_along src/an_move_along.cpp)
target_link_libraries(an_move_along ${catkin_LIBRARIES})
add_executable(an_odom src/an_odom.cpp)
target_link_libraries(an_odom ${catkin_LIBRARIES})
add_executable(an_sub src/an_sub.cpp)
target_link_libraries(an_sub ${catkin_LIBRARIES})
add_executable(an_move_along_side src/an_move_along_side.cpp)
target_link_libraries(an_move_along_side ${catkin_LIBRARIES})
add_executable(an_move_along_realworld src/an_move_along_realworld.cpp)
target_link_libraries(an_move_along_realworld ${catkin_LIBRARIES})
add_executable(lidar_checker src/lidar_checker.cpp)
target_link_libraries(lidar_checker ${catkin_LIBRARIES})
add_executable(move_along_oldversion src/move_along_oldversion.cpp)
target_link_libraries(move_along_oldversion ${catkin_LIBRARIES})
add_executable(wei_move_along src/wei_move_along.cpp)
target_link_libraries(wei_move_along ${catkin_LIBRARIES})
add_executable(wei_odom src/wei_odom.cpp)
target_link_libraries(wei_odom ${catkin_LIBRARIES})
add_executable(wei_sub src/wei_sub.cpp)
target_link_libraries(wei_sub ${catkin_LIBRARIES})
add_executable(record_data src/record_data.cpp)
target_link_libraries(record_data ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)
add_executable(wei_odom_v2 src/wei_odom_v2.cpp)
target_link_libraries(wei_odom_v2 ${catkin_LIBRARIES})
add_executable(custom_scan_filter src/custom_scan_filter.cpp)
target_link_libraries(custom_scan_filter ${catkin_LIBRARIES})

# 安裝目標
install(TARGETS mpc_controller_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# 安裝插件描述文件
install(FILES mpc_controller_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
