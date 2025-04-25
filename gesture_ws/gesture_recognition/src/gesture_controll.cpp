#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

class RobotController
{
public:
    RobotController()
    {
        // Initialize the subscriber to the /gesture topic
        gesture_sub = nh.subscribe("/gesture", 10, &RobotController::gestureCallback, this);

        // Initialize the publisher to the /cmd_vel topic
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    }

    void gestureCallback(const std_msgs::String::ConstPtr& msg)
    {
        ROS_INFO("Received Gesture: [%s]", msg->data.c_str());

        geometry_msgs::Twist vel_msg;
        if (msg->data == "5")
        {
            ROS_INFO("Stopping the robot.");
            geometry_msgs::Twist stop_msg;
            stop_msg.linear.x = 0.0;
            stop_msg.angular.z = 0.0;
            cmd_vel_pub.publish(stop_msg);
        }
        else if (msg->data == "good")
        {
            ROS_INFO("Moving the robot forward.");
            vel_msg.linear.x = 0.3;  // Set speed to 0.5 m/s
            vel_msg.angular.z = 0.0; // No angular velocity
            cmd_vel_pub.publish(vel_msg);
        }
        else if (msg->data == "ROCK")
        {
            ROS_INFO("Turning the robot right.");
            vel_msg.linear.x = 0.0;  // No linear velocity
            vel_msg.angular.z = -0.3; // Set angular velocity to turn right
            cmd_vel_pub.publish(vel_msg);
        }
        else if (msg->data == "1")
        {
            ROS_INFO("let the robot backward.");
            vel_msg.linear.x = -0.3;  // No linear velocity
            vel_msg.angular.z = 0.0; // Set angular velocity to turn right
            cmd_vel_pub.publish(vel_msg);
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber gesture_sub;
    ros::Publisher cmd_vel_pub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gesture_control");
    RobotController robotController;
    ros::spin();
    return 0;
}
