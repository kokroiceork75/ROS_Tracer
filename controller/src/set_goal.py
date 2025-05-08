#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionFeedback
from tf.transformations import quaternion_from_euler

class SingleWaypointNav:
    def __init__(self):
        # 只定義一個目標點 
        #<!-- paper1 goal at (0, 7) -->
   	#<!-- paper2 goal at (0, 4.5) -->
   	#<!-- paper3 goal at (2, 5.5) -->
   	#<!-- paper4 goal at (0, 6.5) -->
   	#<!-- paper5 goal at (0, 1.5) -->
   	#<!-- paper6 goal at (3, 2.0) -->
        x = 3.0
        y = 2.0
        self.target = {"x": x, "y": y, "yaw": 0.07}
        
        self.distance_tolerance = 0.3
        self.feedback_received = False
        self.current_robot_x = 0.0
        self.current_robot_y = 0.0
        self.goal_reached = False
        
        # 首先創建訂閱者
        self.feedback_sub = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.feedback_callback)
        
        # 然後創建發布者，並等待連接
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        # 等待發布者連接
        rospy.loginfo("等待發布者連接...")
        rospy.sleep(1.0)  # 等待連接建立
        
        rospy.loginfo("SingleWaypointNav節點已啟動")
    
    def publish_goal(self, waypoint):
        # 確保有足夠的連接
        if self.goal_pub.get_num_connections() == 0:
            rospy.logwarn("沒有節點訂閱目標主題，等待連接...")
            # 循環等待直到有連接
            rate = rospy.Rate(1.0)  # 1 Hz
            while self.goal_pub.get_num_connections() == 0 and not rospy.is_shutdown():
                rospy.loginfo("等待訂閱者連接...")
                rate.sleep()
        
        # 創建並發布目標
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        
        goal_msg.pose.position.x = waypoint["x"]
        goal_msg.pose.position.y = waypoint["y"]
        
        q = quaternion_from_euler(0, 0, waypoint["yaw"])
        goal_msg.pose.orientation.x = q[0]
        goal_msg.pose.orientation.y = q[1]
        goal_msg.pose.orientation.z = q[2]
        goal_msg.pose.orientation.w = q[3]
        
        # 多次發布以確保接收
        for i in range(3):
            rospy.loginfo("發布目標 (嘗試 %d/3): (%.2f, %.2f, yaw=%.2f)", 
                         i+1, waypoint["x"], waypoint["y"], waypoint["yaw"])
            self.goal_pub.publish(goal_msg)
            rospy.sleep(0.5)  # 等待半秒確保發布
    
    def feedback_callback(self, feedback):
        self.current_robot_x = feedback.feedback.base_position.pose.position.x
        self.current_robot_y = feedback.feedback.base_position.pose.position.y
        self.feedback_received = True
    
    def run(self):
        rate = rospy.Rate(2.0)
        
        # 發布目標（只發送一次）
        self.publish_goal(self.target)
        
        # 等待feedback開始接收
        timeout = rospy.Duration(10.0)  # 10秒超時
        start_time = rospy.Time.now()
        
        while not self.feedback_received and (rospy.Time.now() - start_time) < timeout and not rospy.is_shutdown():
            rospy.logwarn("未收到反饋，等待中...")
            rate.sleep()
        
        if not self.feedback_received:
            rospy.logerr("未能收到機器人位置反饋，檢查move_base是否正在運行？")
            return
        
        # 監控到達目標
        while not rospy.is_shutdown() and not self.goal_reached:
            dx = self.current_robot_x - self.target["x"]
            dy = self.current_robot_y - self.target["y"]
            distance = math.sqrt(dx*dx + dy*dy)
            
            rospy.loginfo("距離目標: %.2f (當前位置: x=%.2f, y=%.2f)", 
                         distance, self.current_robot_x, self.current_robot_y)
            
            if distance <= self.distance_tolerance:
                rospy.loginfo("已到達目標！距離: %.2f", distance)
                self.goal_reached = True
                rospy.loginfo("導航成功完成")
            
            rate.sleep()
        
        # 完成後繼續運行一段時間
        if self.goal_reached:
            rospy.loginfo("等待5秒後關閉...")
            rospy.sleep(5.0)
            rospy.loginfo("節點關閉中")

if __name__ == '__main__':
    rospy.init_node('single_waypoint_nav')
    nav = SingleWaypointNav()
    nav.run()
