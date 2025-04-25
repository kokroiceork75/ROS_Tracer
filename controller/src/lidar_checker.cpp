#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <iostream>

#define PI 3.14159

class LaserPrinter {
public:
    LaserPrinter() {
        // 訂閱 /scan 主題
        sub_ = n_.subscribe("/scan", 1000, &LaserPrinter::laserCallback, this);
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        // 定義用於儲存掃描數據的陣列
        float laser_temp_scan[scan->ranges.size() + 1]; // 原始數據
        float laser_temp[361] = {0};                    // 按角度索引的數據（0-360度）

        // 第一步：將掃描數據存入臨時陣列
        for (int i = 1; i <= scan->ranges.size(); i++) {
            laser_temp_scan[i] = scan->ranges[i];
        }

        // 第二步：將數據轉換為角度索引（與原始程式碼一致）
        for (int i = 0; i <= scan->ranges.size(); ++i) {
            double angle_rad = (scan->angle_min + i * scan->angle_increment) + PI;
            int angle_deg = static_cast<int>(angle_rad * 180 / PI);
            if (angle_deg >= 0 && angle_deg <= 360) { // 確保角度在範圍內
                if (!std::isinf(laser_temp_scan[i]) && laser_temp_scan[i] != 0) {
                    laser_temp[angle_deg] = laser_temp_scan[i];
                }
            }
        }

        // 每隔 10 度列印數據
        ROS_INFO("Laser Scan Data (every 10 degrees):");
        for (int angle = 0; angle <= 360; angle += 10) {
            if (laser_temp[angle] > 0) { // 只列印有效數據
                ROS_INFO("Angle %d deg: %.3f m", angle, laser_temp[angle]);
            } else {
                ROS_INFO("Angle %d deg: No valid data", angle);
            }
        }
        ROS_INFO("-------------------");
    }

private:
    ros::NodeHandle n_;
    ros::Subscriber sub_;
};

int main(int argc, char** argv) {
    // 初始化 ROS 節點
    ros::init(argc, argv, "laser_angle_printer");
    ROS_INFO("Laser Angle Printer Node Started");

    // 創建 LaserPrinter 物件
    LaserPrinter printer;

    // 進入 ROS 迴圈
    ros::spin();

    return 0;
}