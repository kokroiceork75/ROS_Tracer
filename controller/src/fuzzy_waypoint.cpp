#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <cmath>

struct Waypoint {
    double x;
    double y;
    double yaw;
};

class FuzzyWaypointLoop {
public:
    FuzzyWaypointLoop()
    : current_goal_index_(0),
      distance_tolerance_(0.5),
      loopCount_(0),
      maxLoops_(3),
      feedback_received_(false)
    {
        Waypoint w1 = {2.0, 0.0, 0.07};
        Waypoint w2 = {7.585, 0.3, 0.8};
        waypoints_.push_back(w1);
        waypoints_.push_back(w2);

        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
        feedback_sub_ = nh_.subscribe("/move_base/feedback", 10, &FuzzyWaypointLoop::feedbackCallback, this);

        ROS_INFO("FuzzyWaypointLoop node started. maxLoops_=%d", maxLoops_);
    }

    void run() {
        ros::Rate rate(2.0);
        while (ros::ok()) {
            if (waypoints_.empty()) {
                ROS_WARN("No waypoints to visit");
                ros::spinOnce();
                rate.sleep();
                continue;
            }

            if (loopCount_ >= maxLoops_) {
                ROS_INFO("Reached maxLoops_=%d. Shutting down node.", maxLoops_);
                ros::shutdown();
                break;
            }

            if (current_goal_index_ >= waypoints_.size()) {
                loopCount_++;
                ROS_INFO("Finished loop %d / %d", loopCount_, maxLoops_);
                if (loopCount_ >= maxLoops_) {
                    ROS_INFO("Reached maxLoops_=%d. Shutting down node.", maxLoops_);
                    ros::shutdown();
                    break;
                } else {
                    current_goal_index_ = 0;
                    ROS_INFO("Restart waypoint loop... (Next loop index: %d)", loopCount_ + 1);
                }
            }

            Waypoint current_waypoint = waypoints_[current_goal_index_];
            if (!feedback_received_) {
                ROS_WARN_THROTTLE(5.0, "No feedback received yet. Publishing goal...");
                publishGoal(current_waypoint);
            } else {
                double dx = current_waybot_x_ - current_waypoint.x;
                double dy = current_waybot_y_ - current_waypoint.y;
                double distance = std::sqrt(dx*dx + dy*dy);
                ROS_INFO("Distance to goal %zu: %.2f (Current pos: x=%.2f, y=%.2f)", 
                         current_goal_index_, distance, current_waybot_x_, current_waybot_y_);

                if (distance <= distance_tolerance_) {
                    ROS_INFO("Arrived at waypoint %zu, switching to next...", current_goal_index_);
                    current_goal_index_++;
                    ros::Duration(3.0).sleep(); // 縮短等待時間
                    publishGoal(waypoints_[current_goal_index_ % waypoints_.size()]); // 立即發送下一個目標
                } else {
                    publishGoal(current_waypoint);
                }
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher goal_pub_;
    ros::Subscriber feedback_sub_;
    std::vector<Waypoint> waypoints_;
    size_t current_goal_index_;
    double distance_tolerance_;
    int loopCount_;
    int maxLoops_;
    double current_waybot_x_ = 0.0;
    double current_waybot_y_ = 0.0;
    double current_waybot_yaw_ = 0.0;
    bool feedback_received_;

    void publishGoal(const Waypoint& wpt) {
        geometry_msgs::PoseStamped goal_msg;
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.header.frame_id = "map";
        goal_msg.pose.position.x = wpt.x;
        goal_msg.pose.position.y = wpt.y;

        tf2::Quaternion q;
        q.setRPY(0, 0, wpt.yaw);
        goal_msg.pose.orientation = tf2::toMsg(q);

        ROS_INFO("Publishing goal %zu: (%.2f, %.2f, yaw=%.2f)", current_goal_index_, wpt.x, wpt.y, wpt.yaw);
        goal_pub_.publish(goal_msg);
    }

    void feedbackCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr & fb) {
        current_waybot_x_ = fb->feedback.base_position.pose.position.x;
        current_waybot_y_ = fb->feedback.base_position.pose.position.y;
        feedback_received_ = true;
        ROS_INFO_THROTTLE(5.0, "Feedback received: x=%.2f, y=%.2f", current_waybot_x_, current_waybot_y_);

        tf2::Quaternion q;
        tf2::convert(fb->feedback.base_position.pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        current_waybot_yaw_ = yaw;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fuzzy_waypoint_loop");
    FuzzyWaypointLoop loop;
    loop.run();
    return 0;
}