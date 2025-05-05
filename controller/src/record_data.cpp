#include <ros/ros.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <fstream>
#include <cmath>
#include <signal.h>

// 全局變數
double record_interval = 0.17; // 記錄間隔（秒）
double allow_rms = 0.001;      // 距離閾值（米）
std::string end_condition = "ctrl_c"; // 結束條件：distance, ctrl_c, both

std::string SAVE_PATH = "/home/user/wei_ws/src/controller/src/data/";
std::ofstream dataFile;
move_base_msgs::MoveBaseActionFeedback::ConstPtr last_feedback;
geometry_msgs::Twist::ConstPtr last_cmd_vel;
std_msgs::Int32::ConstPtr last_controller_flag; // 新增：儲存最新的 controller_flag
ros::Time start_time;
ros::Time last_record_time;
double goal_x = 0.0, goal_y = 0.0;
bool goal_reached = false;
bool recording_started = false;
bool goal_received = false;
double total_linear_vel = 0.0;
double total_angular_vel = 0.0;
int vel_count = 0;
bool shutdown_requested = false; // 標記是否收到 Ctrl+C

// 處理 Ctrl+C 信號
void signalHandler(int sig) {
    if (sig == SIGINT && (end_condition == "ctrl_c" || end_condition == "both")) {
        ROS_INFO("Received Ctrl+C, shutting down gracefully...");
        shutdown_requested = true;
        ros::shutdown();
    }
}

// 新增：controller_flag 回調函數
void controllerFlagCallback(const std_msgs::Int32::ConstPtr& msg) {
    last_controller_flag = msg;
    ROS_DEBUG("Received controller_flag: %d", msg->data);
}

void feedbackCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) {
    last_feedback = msg;
    
    // 如果已經收到目標點，且結束條件允許檢查距離
    if (goal_received && (end_condition == "distance" || end_condition == "both")) {
        double dx = msg->feedback.base_position.pose.position.x - goal_x;
        double dy = msg->feedback.base_position.pose.position.y - goal_y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        ROS_INFO("Current position: x = %f, y = %f, Distance to goal: %f", 
                 msg->feedback.base_position.pose.position.x, 
                 msg->feedback.base_position.pose.position.y, 
                 distance);
        
        if (distance < allow_rms) {
            goal_reached = true;
            ROS_INFO("Goal reached! Shutting down...");
            ros::shutdown();
        }
    }
    
    // 如果已經開始記錄，檢查時間間隔後寫入檔案
    if (recording_started && dataFile.is_open()) {
        ros::Time current_time = ros::Time::now();
        double time_since_last_record = (current_time - last_record_time).toSec();
        
        if (time_since_last_record >= record_interval) {
            // 驗證時間戳是否有效
            double record_time = msg->header.stamp.isValid() ? msg->header.stamp.toSec() : current_time.toSec();
            ROS_DEBUG("Recording data at time: %f", record_time);
            
            dataFile << record_time << "\t"
                     << msg->feedback.base_position.pose.position.x << "\t"
                     << msg->feedback.base_position.pose.position.y << "\t"
                     << msg->feedback.base_position.pose.orientation.x << "\t"
                     << msg->feedback.base_position.pose.orientation.y << "\t"
                     << msg->feedback.base_position.pose.orientation.z << "\t"
                     << msg->feedback.base_position.pose.orientation.w << "\t"
                     << (last_cmd_vel ? last_cmd_vel->linear.x : 0.0) << "\t"
                     << (last_cmd_vel ? last_cmd_vel->angular.z : 0.0) << "\t"
                     << (last_controller_flag ? last_controller_flag->data : 0) // 新增：記錄 controller_flag
                     << std::endl << std::flush; // 立即寫入文件
            
            last_record_time = current_time;
        }
    }
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    last_cmd_vel = msg;
    
    if (recording_started) {
        total_linear_vel += msg->linear.x;
        total_angular_vel += msg->angular.z;
        vel_count++;
    }
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    goal_x = msg->pose.position.x;
    goal_y = msg->pose.position.y;
    goal_received = true;
    
    // 在收到目標點時開始記錄
    if (!recording_started) {
        recording_started = true;
        start_time = ros::Time::now();
        last_record_time = start_time;
        ROS_INFO("Recording started on goal received!");
    }
    
    ROS_INFO("Goal received: x = %f, y = %f", goal_x, goal_y);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "record_data");
    ros::NodeHandle nh;
    
    // 從 ROS 參數伺服器讀取結束條件
    nh.param<std::string>("end_condition", end_condition, "both");
    if (end_condition != "distance" && end_condition != "ctrl_c" && end_condition != "both") {
        ROS_ERROR("Invalid end_condition: %s. Using 'both' as default.", end_condition.c_str());
        end_condition = "both";
    }
    ROS_INFO("Using end condition: %s", end_condition.c_str());
    
    // 註冊 Ctrl+C 信號處理器
    signal(SIGINT, signalHandler);
    
    dataFile.open(SAVE_PATH + "data.txt");
    if (!dataFile) {
        ROS_ERROR("Failed to open file for writing. Please check the path: %s", SAVE_PATH.c_str());
        return -1;
    }
    
    // 更新文件頭，包含 controller_flag
    dataFile << "time\tx\ty\torientation_x\torientation_y\torientation_z\torientation_w\tlinear_vel\tangular_vel\tcontroller_flag" << std::endl;
    
    ros::Subscriber feedback_sub = nh.subscribe("/move_base/feedback", 10, feedbackCallback);
    ros::Subscriber cmd_sub = nh.subscribe("/cmd_vel", 10, cmdVelCallback);
    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, goalCallback);
    ros::Subscriber controller_flag_sub = nh.subscribe("/controller_flag", 10, controllerFlagCallback); // 新增：訂閱 controller_flag
    
    ROS_INFO("Waiting for goal to start recording...");
    
    try {
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception occurred: %s", e.what());
    }
    
    // 清理資源
    if (dataFile.is_open()) {
        dataFile.close();
        ROS_INFO("Data file closed.");
    }
    
    if (vel_count > 0) {
        double avg_linear_vel = total_linear_vel / vel_count;
        double avg_angular_vel = total_angular_vel / vel_count;
        ROS_INFO("Average Linear Velocity: %f m/s", avg_linear_vel);
        ROS_INFO("Average Angular Velocity: %f rad/s", avg_angular_vel);
    }
    
    ROS_INFO("Recording finished! Shutdown reason: %s", 
             goal_reached ? "Goal reached" : (shutdown_requested ? "Ctrl+C" : "Other"));
    return 0;
}