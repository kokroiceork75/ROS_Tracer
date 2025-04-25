#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>

class ScanFilter {
public:
  ScanFilter() {
    // 訂閱 /scan 話題
    sub_ = nh_.subscribe("/scan", 10, &ScanFilter::scanCallback, this);
    // 發布 /filtered_scan 話題
    pub_ = nh_.advertise<sensor_msgs::LaserScan>("/filtered_scan", 10);
    // 設置過濾角度（90° 到 270°，轉為 -π 到 π 範圍）
    lower_angle_ = 1.5708;  // 90°
    upper_angle_ = 4.5; // 270° 等於 -90°
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // 創建過濾後的 LaserScan 訊息
    sensor_msgs::LaserScan filtered = *msg;

    // 設置過濾後的角度範圍
    filtered.angle_min = lower_angle_;  // 90°
    filtered.angle_max = upper_angle_;  // -90° (270°)

    // 計算起始和結束索引
    int start_idx = std::ceil((lower_angle_ - msg->angle_min) / msg->angle_increment);
    int end_idx = std::floor((upper_angle_ - msg->angle_min) / msg->angle_increment);

    // 確保索引有效
    start_idx = std::max(0, start_idx);
    end_idx = std::min(static_cast<int>(msg->ranges.size()) - 1, end_idx);

    // 調整 ranges 和 intensities（如果存在）
    std::vector<float> filtered_ranges;
    std::vector<float> filtered_intensities;
    for (int i = start_idx; i <= end_idx; ++i) {
      filtered_ranges.push_back(msg->ranges[i]);
      if (!msg->intensities.empty()) {
        filtered_intensities.push_back(msg->intensities[i]);
      }
    }

    // 更新 filtered 訊息
    filtered.ranges = filtered_ranges;
    filtered.intensities = filtered_intensities;

    // 調整頭部資訊
    filtered.angle_min = msg->angle_min + start_idx * msg->angle_increment;
    filtered.angle_max = msg->angle_min + end_idx * msg->angle_increment;

    // 發布過濾後的數據
    pub_.publish(filtered);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  double lower_angle_;
  double upper_angle_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "custom_scan_filter");
  ScanFilter filter;
  ros::spin();
  return 0;
}