#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionFeedback
from tf.transformations import quaternion_from_euler

class SingleWaypointNav:
    def __init__(self):
        # 只定義一個目標點
        x = 0.0
        y = 6.5
        self.target = {"x": x, "y": y, "yaw": 0.07}
        
        self.distance_tolerance = 0.3
        self.feedback_received = False
        self.current_robot_x = 0.0
        self.current_robot_y = 0.0
        self.goal_reached = False
        
        # 創建發布者和訂閱者
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.feedback_sub = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.feedback_callback)
        
        rospy.loginfo("SingleWaypointNav node started")
    
    def publish_goal(self, waypoint):
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
        
        rospy.loginfo("Publishing goal: (%.2f, %.2f, yaw=%.2f)", 
                     waypoint["x"], waypoint["y"], waypoint["yaw"])
        self.goal_pub.publish(goal_msg)
    
    def feedback_callback(self, feedback):
        self.current_robot_x = feedback.feedback.base_position.pose.position.x
        self.current_robot_y = feedback.feedback.base_position.pose.position.y
        self.feedback_received = True
    
    def run(self):
        rate = rospy.Rate(2.0)
        
        # 初始發送目標
        self.publish_goal(self.target)
        
        while not rospy.is_shutdown() and not self.goal_reached:
            if not self.feedback_received:
                rospy.logwarn_throttle(5.0, "No feedback received yet...")
                # 再次發送目標
                self.publish_goal(self.target)
            else:
                dx = self.current_robot_x - self.target["x"]
                dy = self.current_robot_y - self.target["y"]
                distance = math.sqrt(dx*dx + dy*dy)
                
                rospy.loginfo("Distance to goal: %.2f (Current pos: x=%.2f, y=%.2f)", 
                             distance, self.current_robot_x, self.current_robot_y)
                
                if distance <= self.distance_tolerance:
                    rospy.loginfo("Goal reached! Distance: %.2f", distance)
                    self.goal_reached = True
                    rospy.loginfo("Navigation completed successfully")
                    # 不再發送新目標
                else:
                    # 如果還沒到達，每隔幾秒重新發送一次目標
                    self.publish_goal(self.target)
            
            rate.sleep()
        
        # 完成後繼續運行一段時間，但不再發送目標
        if self.goal_reached:
            rospy.loginfo("Waiting for 5 seconds before shutting down...")
            rospy.sleep(5.0)
            rospy.loginfo("Node shutting down")

if __name__ == '__main__':
    rospy.init_node('single_waypoint_nav')
    nav = SingleWaypointNav()
    nav.run()
