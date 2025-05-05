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
  geometry_msgs # MPC 需要
  nav_msgs      # MPC 需要
  tf2_geometry_msgs # MPC 需要
  cv_bridge
  image_transport
  move_base_msgs
)

# 查找 OpenCV、PCL、Eigen3 和 yaml-cpp
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED) # 注意之前的 PCL_PEOPLE 警告，但暫時忽略
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# --- 新增：定義 qpOASES 的根目錄 ---
set(QPOASES_ROOT_DIR /home/user/qpOASES) # 您的 qpOASES 根目錄
message(STATUS "Using qpOASES from: ${QPOASES_ROOT_DIR}")

# --- 修改：查找 qpOASES 庫文件 ---
# find_library 會查找 lib<NAMES>.so 或 lib<NAMES>.a
find_library(QPOASES_LIBRARY
             NAMES qpOASES   # 庫文件的基礎名稱 (libqpOASES.a)
             PATHS
                 # --- 添加實際找到的庫文件路徑 ---
                 ${QPOASES_ROOT_DIR}/build/libs  # <--- 根據 'find' 結果添加此路徑
                 # --- 保留其他可能的猜測路徑 (以防萬一) ---
                 ${QPOASES_ROOT_DIR}/lib
                 ${QPOASES_ROOT_DIR}/bin
                 ${QPOASES_ROOT_DIR}/build/lib # 有些項目可能生成在 build/lib
             REQUIRED # 如果找不到就報錯
            )

# 檢查是否找到庫文件
if(NOT QPOASES_LIBRARY)
    # 如果仍然報錯，請仔細檢查 ${QPOASES_ROOT_DIR}/build/libs 目錄下是否有 libqpOASES.a 文件
    message(FATAL_ERROR "qpOASES library (libqpOASES.so or .a) not found in specified paths. Checked paths include ${QPOASES_ROOT_DIR}/build/libs. Please check the qpOASES compilation output.")
else()
    message(STATUS "Found qpOASES library: ${QPOASES_LIBRARY}") # 這裡會顯示找到的完整路徑
endif()

# 引入 Conan 生成的依賴
# 確保這個路徑是正確的
include(/home/user/wei_ws/src/3rd/conanbuildinfo.cmake)
conan_basic_setup(NO_OUTPUT_DIRS)

# 設置 Eigen3 變數
# 使用 find_package 找到的標準變數通常更可靠
# set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR}) # 可以註釋掉，使用 ${Eigen3_INCLUDE_DIRS}
# set(Eigen3_LIBRARIES ${EIGEN3_LIBRARIES})     # 可以註釋掉，使用 ${Eigen3_LIBRARIES}

# 配置 catkin 包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mpc_controller_plugin # 如果這個插件也用qpoases，需要在下面 target_link_libraries 添加
  CATKIN_DEPENDS
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
    geometry_msgs # 添加依賴
    nav_msgs      # 添加依賴
    tf2_geometry_msgs # 添加依賴
  DEPENDS Eigen3 # <--- 在 DEPENDS 中添加 qpOASES
)

# --- 修改：指定頭文件路徑 ---
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}     # 使用標準變數
  ${OpenCV_INCLUDE_DIRS}
  ${CONAN_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}    # 使用標準變數
  #/home/user/wei_ws/src/eigen-QP/ # 您自己的路徑
  ${QPOASES_ROOT_DIR}/include     # qpOASES 頭文件路徑
)

link_directories(${PCL_LIBRARY_DIRS}) # 通常不需要，find_package 會處理
add_definitions(${PCL_DEFINITIONS})

# --- 編譯 mpc_controller_plugin 作為共享庫 ---
add_library(mpc_controller_plugin src/mpc_controller.cpp src/controller.cpp)
target_link_libraries(mpc_controller_plugin
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES} # <--- 確保 Eigen3 被連結
  ${CONAN_LIBS}
  # 如果 mpc_controller_plugin 也使用 qpOASES，取消下面一行的註釋
  # ${QPOASES_LIBRARY}
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
target_link_libraries(record_data ${catkin_LIBRARIES} ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES})
add_executable(wei_odom_v2 src/wei_odom_v2.cpp)
target_link_libraries(wei_odom_v2 ${catkin_LIBRARIES})
add_executable(custom_scan_filter src/custom_scan_filter.cpp)
target_link_libraries(custom_scan_filter ${catkin_LIBRARIES})
add_executable(an_mpc src/an_mpc.cpp)
target_link_libraries(an_mpc ${catkin_LIBRARIES} ${QPOASES_LIBRARY})


# --- 修改：編譯 wei_mpc ---
add_executable(wei_mpc src/wei_mpc.cpp)
target_link_libraries(wei_mpc
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}      # 連結 Eigen3
  ${YAML_CPP_LIBRARIES}    # 連結 yaml-cpp
  ${CONAN_LIBS}            # 連結 Conan 庫
  ${QPOASES_LIBRARY}       # <--- 連結 qpOASES 庫
)

# --- 安裝目標 ---
# 安裝 mpc_controller_plugin 庫
install(TARGETS mpc_controller_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION} # 通常庫不安裝到 bin
)

# 安裝插件描述文件
install(FILES mpc_controller_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 如果您想安裝可執行文件 (例如 wei_mpc)
# install(TARGETS wei_mpc
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
