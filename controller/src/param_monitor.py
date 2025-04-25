#!/usr/bin/env python3
import rospy

param_name = "/velodyne_nodelet_manager_transform/min_range"  # 你想監聽的參數
last_value = None

rospy.init_node("param_monitor", anonymous=True)
rospy.loginfo(f"開始監聽參數 {param_name}")

while not rospy.is_shutdown():
    try:
        current_value = rospy.get_param(param_name)
        if current_value != last_value:
            rospy.loginfo(f"⚠ 參數 {param_name} 被修改為 {current_value}")
            last_value = current_value
    except KeyError:
        rospy.logwarn(f"參數 {param_name} 不存在")
    
    rospy.sleep(1)  # 每秒檢查一次
