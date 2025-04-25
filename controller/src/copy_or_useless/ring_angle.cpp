#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <map>
#include <vector>
#include <algorithm>

class VelodyneReader {
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;

public:
    VelodyneReader() {
        ROS_INFO("Initializing VelodyneReader...");
        sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 10, 
            &VelodyneReader::callback, this);
        ROS_INFO("Subscribed to /velodyne_points topic");
    }

    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        try {
            // 转换点云格式
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*msg, *cloud);
            
            // 创建一个map来存储0-360度之间的点
            std::map<int, std::vector<double>> ringPointsMap;

            // 遍历点云中的所有点
            for(size_t i = 0; i < cloud->points.size(); i++) {
                // 提取 ring 值
                int ring = static_cast<int>(cloud->points[i].intensity) & 0xF;
                
                // 只处理 ring 8 的点
                if (ring == 3) {
                    // 计算角度
                    double angle = atan2(cloud->points[i].y, cloud->points[i].x);
                    double degree = angle * 180 / M_PI;
                    if(degree < 0) degree += 360;
                    
                    // 计算距离
                    double distance = sqrt(
                        pow(cloud->points[i].x, 2) + 
                        pow(cloud->points[i].y, 2) + 
                        pow(cloud->points[i].z, 2)
                    );
                    
                    // 存储距离值，以角度为索引
                    int angleIndex = static_cast<int>(degree);
                    ringPointsMap[angleIndex].push_back(distance);
                }
            }
            
            // 打印结果
            ROS_INFO("ring :");
            
            // 遍历并打印180-185度的角度点
            for (int angle = 300; angle <= 335; angle++) {
                auto it = ringPointsMap.find(angle);
                if (it != ringPointsMap.end() && !it->second.empty()) {
                    // 计算平均距离
                    double avgDistance = 0;
                    for (double dist : it->second) {
                        avgDistance += dist;
                    }
                    avgDistance /= it->second.size();
                    
                    ROS_INFO("angle %d: %.2fm", angle, avgDistance);
                }
            }
            
        } catch(const std::exception& e) {
            ROS_ERROR("Error processing point cloud: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "velodyne_reader");
    ROS_INFO("Starting Velodyne Reader node");
    
    try {
        VelodyneReader reader;
        ros::spin();
    } catch(const std::exception& e) {
        ROS_ERROR("Error: %s", e.what());
        return 1;
    }
    
    return 0;
}