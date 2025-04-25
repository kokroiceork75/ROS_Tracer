#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>

#define maxLoops 4  // 可以改循環次數

// 為方便起見，定義一個別名
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * @brief 建立一個 MoveBaseGoal，設定位置與朝向
 * @param x 目標點 x 座標
 * @param y 目標點 y 座標
 * @param yaw 以弧度表示的旋轉角度
 * @return move_base_msgs::MoveBaseGoal 目標點訊息
 */
move_base_msgs::MoveBaseGoal createGoal(double x, double y, double yaw)
{
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map"; // 根據你的環境來設定參考座標框架
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;

    // 將 yaw 轉換成四元數
    tf::Quaternion quaternion = tf::createQuaternionFromYaw(yaw);
    goal.target_pose.pose.orientation.x = quaternion.x();
    goal.target_pose.pose.orientation.y = quaternion.y();
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();

    return goal;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "two_way_navigation_cpp");
    ros::NodeHandle nh;

    // 建立 move_base 的 Action Client (true 表示自動等待 server 連線)
    MoveBaseClient ac("move_base", true);

    ROS_INFO("wait for move_base Action Server launch...");
    ac.waitForServer();
    ROS_INFO("success to connect to move_base Action Server");
    ROS_INFO("Max Loop is: %d -.-#", maxLoops);


    // 定義兩個目標點 (請依實際需求調整座標與 yaw 值)
    move_base_msgs::MoveBaseGoal goal1 = createGoal(-4.563, 0.145, -1.655);   // 目標點 1
    move_base_msgs::MoveBaseGoal goal2 = createGoal(-0.006, 0.003, 1.562);      // 目標點 2

    bool toggle = true; // 用來切換目標點的旗標

    // 設定最大循環次數
    // int maxLoops = 10; // 例如循環 10 次
    int loopCount = 0;

    // 此 Rate 可用來在每次迴圈後暫停一段時間 (例如每 5 秒切換一次)
    ros::Rate rate(0.15); // 0.2 Hz -> 每 5 秒一次

    while (ros::ok() && loopCount < maxLoops)
    {
        move_base_msgs::MoveBaseGoal current_goal;
        if (toggle)
        {
            current_goal = goal1;
            ROS_INFO("----Going to Goal 1----");
        }
        else
        {
            current_goal = goal2;
            ROS_INFO("----Going to Goal 2----");
        }

        // 發送目標點
        ac.sendGoal(current_goal);

        // 設定等待結果的超時時間 (例如 60 秒)
        bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));

        if (!finished_before_timeout)
        {
            ROS_ERROR("overtime, cancel the navigation");
            ac.cancelGoal();
        }
        else
        {
            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Arrive!!!! //^3^//");
            }
            else
            {
                ROS_WARN("navigate fail, state: %s", ac.getState().toString().c_str());
            }
        }

        // 切換目標點，進行往返
        toggle = !toggle;
        loopCount++;  // 累計循環次數
        rate.sleep();
    }
    ROS_INFO("-----------------------------");
    ROS_INFO("Finished %d loops.", maxLoops);
    ROS_INFO("=============================");
    return 0;
}
