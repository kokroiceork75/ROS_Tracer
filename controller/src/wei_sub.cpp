#include <ctime> 
#include <math.h>
#include <cmath>
#include <vector>
#include <iostream>   
#include <ros/ros.h>          // 加入ROS公用程序
#include <std_msgs/String.h> 
#include <std_msgs/Float64.h> 
#include <std_msgs/Int32.h> 
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h> 
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <nav_msgs/Path.h>
#include "sensor_msgs/LaserScan.h"

///#include <nav_msgs/OccupancyGrid.h>
using namespace std;
#define MIN(a,b) (a < b ? a : b)
#define save_data_path1    "/home/user/wei_ws/src/controller/src/position_random/real-map2s/position_all_fuzzy_pid_NEW2.txt"  //整條路徑
#define save_along_wall    "/home/user/wei_ws/src/controller/src/position_random/real-map2s/position_along_wall_fuzzy_pid_NEW2.txt" //沿牆
#define save_search_target "/home/user/wei_ws/src/controller/src/position_random/real-map2s/posititon_search_target_fuzzy_pid_NEW2.txt" //尋標
#define save_distance      "/home/user/wei_ws/src/controller/src/position_random/real-map2s/distance-NEW.txt"  //左/右/前測距離
#define save_velocity      "/home/user/wei_ws/src/controller/src/position_random/real-map2s/velocity-NEW.txt" 
#define save_velocity_every "/home/user/wei_ws/src/controller/src/position_random/real-map2s/velocity-every-1.txt"
#define save_every_error   "/home/user/wei_ws/src/controller/src/position_random/real-map2s/dis_error-1.txt"
#define save_position "/home/user/wei_ws/src/controller/src/position_random/amcl_position.txt"
#define max_step 40000
#define robot_margin 0.285
const double dwall = 0.3;
// danger_threshold現在是定值 不是變數
double allow_goal_rms = 0.3;
const double f_along = (dwall + 0.45 + 0.15); // front進沿牆，加多少根據經驗可修改
const double s_along = (dwall + 0.50 + 0.15); // side,加多少根據經驗可修改
const double danger_threshold1 = (robot_margin + dwall), 
				danger_threshold2 = (robot_margin + dwall) / cos(25 * M_PI / 180.0), 
				danger_threshold3 = (0.132 + dwall) / sin(45 * M_PI / 180.0),
            danger_threshold4 =  (0.132 + dwall) / sin(65 * M_PI / 180.0);// 0.132是雷達正前方到機器人邊緣的距離
double goal_x, goal_y, goal_z;
double theta_d; // final goal和機器人角度差
double spin_left = 1.0, spin_right = -spin_left;
int danger = 0, danger_flag = 0; // 2 for left, 3 for right



double fff=0;
int p[max_step+1]={0}, counts=0, last[max_step],ss=111;
int HH;
double amcl_orientation_x,amcl_orientation_y,amcl_orientation_z,amcl_position_z,amcl_position_y,amcl_position_x;
double amcl_x, amcl_y, amcl_z;
double AMCL_x[max_step], AMCL_y[max_step];

double error_orientation_z;
double target_orientation_z, target_position_x, target_position_y;
double left_min, right_min, straight_min;
double decision_left,decision_right; 
double laser_temp[361];
float laser_temp_scan[897];
double vel_s,vel_a,angular_s,angular_a;
double rms, rms_x, rms_y;
int status = 0;
double integral, turn;
double roll, pitch, yaw;
double margin_laser[361];
double position_x[max_step],position_y[max_step];
double error_x,error_y,directions;
int ik ;
double x_,y_;
double roll_s, pitch_s, yaw_s, roll_t, pitch_t, yaw_t;

geometry_msgs::Twist msg;

bool recieved_message = false;
class cmd_sub_pub
{
   private:
      ros::Publisher chatter_pub1;
      ros::Publisher chatter_pub3;
      ros::NodeHandle n;
      ros::Publisher pub;
      ros::Subscriber sub_1;
      ros::Subscriber sub_2;
      ros::Subscriber sub_goal;
      ros::Subscriber sub_goal1;
      ros::Subscriber sub_laser;
      ros::Subscriber sub_amcl;
      ros::Subscriber sub_info1;
      ros::Subscriber sub_info2;
      ros::Subscriber sub_path ;
      ros::Subscriber sub_obstacle_positions;
      ros::Subscriber sub_cmdvel;
      ros::Subscriber sub_odom0;
      ros::Subscriber sub_decision;
      ros::Publisher x1_pub;
      ros::Publisher y1_pub;
      ros::Publisher x2_pub;
      ros::Publisher y2_pub;
      ros::Publisher pub_1;
      ros::Publisher pub_2;
      ros::Publisher controller_flag_pub;
      bool reached_goal;  // **新增變數來標記是否到達目標**

   public:
      cmd_sub_pub()
      {
         pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",3000);
         sub_1 = n.subscribe("/cmd_vel_1",3000, &cmd_sub_pub::speed_Search, this);
         sub_2 = n.subscribe("/cmd_vel_2",3000, &cmd_sub_pub::speed_Along_wall, this);
         chatter_pub1 = n.advertise<std_msgs::Int32>("chatter1", 3000); // for status
         chatter_pub3 = n.advertise<std_msgs::Int32>("chatter3", 3000); // for danger flag
         
         x1_pub=n.advertise<std_msgs::Float64>("data_x1",3000);
         y1_pub=n.advertise<std_msgs::Float64>("data_y1",3000);
         x2_pub=n.advertise<std_msgs::Float64>("data_x2",3000);
         y2_pub=n.advertise<std_msgs::Float64>("data_y2",3000);
         pub_1=n.advertise<std_msgs::Float64>("info_1",3000);
         pub_2=n.advertise<std_msgs::Int32>("info_2",3000);
         // wei need change, 我猜這個是fuzzy controller 的cmd_vel
         // pub = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_velx",3000);
         //sub_laser=n.subscribe("/p3dx/laser/scan",3000, &cmd_sub_pub::lasercallback,this);
         sub_laser=n.subscribe("/scan",3000, &cmd_sub_pub::lasercallback,this);
         sub_goal = n.subscribe("/move_base_simple/goal",3000, &cmd_sub_pub::Search_Target, this);
         sub_goal1 = n.subscribe("/move_base_simple/goal",3000, &cmd_sub_pub::Final_Goal, this);
         // sub_info1 =n.subscribe("chatter1",3000, &cmd_sub_pub::chatter1Callback,this);
         sub_info2 =n.subscribe("chatter2",3000, &cmd_sub_pub::chatter2Callback,this);
         sub_amcl = n.subscribe("/move_base/feedback", 3000, &cmd_sub_pub::amcl_Callback,this);
         // record which controller be uesd
         controller_flag_pub = n.advertise<std_msgs::Int32>("/controller_flag", 3000);
      
         double minimum(int, int , int );
         void pathCallback1(const geometry_msgs::Twist& vel);
         void pathCallback2(const geometry_msgs::Twist& vel);
         void lasercallback(const sensor_msgs::LaserScan::ConstPtr& scan);
         void decision_path(double, double, double);
         void Search_Target(const geometry_msgs::PoseStamped::ConstPtr& goal);
         void Search_amcl(const nav_msgs::Odometry::ConstPtr& amcl);
         void chatter1Callback(const std_msgs::String::ConstPtr& info);
         void amcl_Callback(const::move_base_msgs::MoveBaseActionFeedback::ConstPtr & amcl);
         void publisher_info(int jj ) ;
         void Calcuate_positions();

      }
      void Final_Goal(const geometry_msgs::PoseStamped::ConstPtr& F_goal) 
      {
        goal_x = F_goal->pose.position.x;
        goal_y = F_goal->pose.position.y;
        tf2::Quaternion s;
        tf2::convert(F_goal->pose.orientation, s);
        tf2::Matrix3x3(s).getRPY(roll_s, pitch_s, yaw_s);
        goal_z = yaw_s;
        // 座標系轉換，與 amcl_pose_sub_pub_1 一致
        if (goal_z <= -M_PI / 2 && goal_z >= -M_PI)
            goal_z = -(M_PI / 2 + (M_PI + goal_z));
        else
            goal_z = -(goal_z - M_PI / 2);
      }
      void lasercallback(const sensor_msgs::LaserScan::ConstPtr& scan)
      {
         int k ,g;
         for(int i=1;i<=scan->ranges.size();i++)
         {
            laser_temp_scan[i]=scan->ranges[i]; ///202009 180 to 270
         }
         for (int i = 0; i <= scan->ranges.size(); ++i)
         {
            double angle_rad = (scan->angle_min + i * scan->angle_increment) + M_PI;
            int angle_deg = angle_rad*180/M_PI;
            if (std::isinf(laser_temp_scan[i]))
				{
					laser_temp[angle_deg] = 10.4999;
				}		
				else
				{
					// 過濾小於0.5公尺的盲區值
					if (laser_temp_scan[i] <= 0.5)
					{
						laser_temp[angle_deg] = 0.4997;
					}
					else
					{
						laser_temp[angle_deg] = laser_temp_scan[i];
					}
				}		
         }
         left_min=minimum(205,269,k);
         right_min=minimum(90,155,k);
         straight_min=minimum(155,205,k);
      }
      // wei need change, no use anymore
      // void decision_path(double i, double j, double k ,int jj)
      // {  
      //    margin_laser[35] = 0.49207; // wei change Tracer20250109
      //    margin_laser[40] = 0.44073; 
      //    margin_laser[45] = 0.40190; 
      //    margin_laser[50] = 0.37247; 
      //    margin_laser[310] = 0.37247; 
      //    margin_laser[315] = 0.40190; 
      //    margin_laser[320] = 0.44073; 
      //    margin_laser[325] = 0.49207; 
         
      //    margin_laser[90] = 0.29010;
      //    margin_laser[95] = 0.29170;
      //    margin_laser[100] = 0.29562;
      //    margin_laser[105] = 0.30188;
      //    margin_laser[110] = 0.31096;
      //    margin_laser[115] = 0.31017;
      //    margin_laser[120] = 0.26386;
      //    margin_laser[125] = 0.23164;
      //    margin_laser[130] = 0.20743;
      //    margin_laser[135] = 0.18912;
      //    margin_laser[140] = 0.17525;
      //    margin_laser[145] = 0.16425;
      //    margin_laser[150] = 0.15581;
      //    margin_laser[155] = 0.14915;
      //    margin_laser[160] = 0.14418;
      //    margin_laser[165] = 0.14049;
      //    margin_laser[170] = 0.13806;
      //    margin_laser[175] = 0.13670;
      //    margin_laser[180] = 0.13641;
      //    margin_laser[185] = 0.13715;
      //    margin_laser[190] = 0.13898;
      //    margin_laser[195] = 0.14192;
      //    margin_laser[200] = 0.14617;
      //    margin_laser[205] = 0.15179;
      //    margin_laser[210] = 0.15924;
      //    margin_laser[215] = 0.16863;
      //    margin_laser[220] = 0.18088;
      //    margin_laser[225] = 0.19669;
      //    margin_laser[230] = 0.21694;
      //    margin_laser[235] = 0.24445;
      //    margin_laser[240] = 0.28163;
      //    margin_laser[245] = 0.31727;
      //    margin_laser[250] = 0.30667;
      //    margin_laser[255] = 0.29880;
      //    margin_laser[260] = 0.29361;
      //    margin_laser[265] = 0.29069;
      //    margin_laser[269] = 0.29005;
      //    margin_laser[270] = 0.29005;
      //    p[jj]=0;
      //    int ln=0;
      //    AMCL_x[jj]=amcl_x;
      //    AMCL_y[jj]=amcl_y;
      //    rms_x=AMCL_x[jj]-target_position_x;
      //    rms_y=AMCL_y[jj]-target_position_y;
      //    rms= sqrt(pow(rms_x,2)+pow(rms_y,2));

      void publisher_info(int jj ) //印出訊息並傳遞數值給其他程式從pub1~pub10;
      {        
         std_msgs::Float64 info_1;
         std_msgs:: Int32  info_2, info_status, info_danger;

         info_1.data = ss;
         info_2.data = HH;
         info_status.data = status;
         info_danger.data = danger;
         pub_1.publish(info_1); 
         pub_2.publish(info_2); 
         chatter_pub1.publish(info_status);
         chatter_pub3.publish(info_danger);
         
      }
      // 儲存位置
      void save_position_data(int jj) 
      {
         FILE* pfout = fopen(save_position, "a");
         if (pfout == NULL) {
               printf("Fail to open file\n");
               exit(1);
         }
         fprintf(pfout, "%lf \t %lf \t %lf\n", amcl_x, amcl_y, amcl_z);
         fclose(pfout);
      }
      double minimum(int i, int j ,int &k)
      {  
         double laser_min=100;
         for ( k=i ;k<j;k++)
         {   
            if (laser_min>laser_temp[k])
            { 
               laser_min=laser_temp[k];
            }
         }
         return laser_min;
      }
      void check_danger(int jj)
      {
         danger = 0, danger_flag = 0;
         // wei need change, spin in place implementation
         // bool right_danger = (laser_temp[165] <= 0.51) ||(laser_temp[125] <= 0.51) || (laser_temp[110] <= 0.51) /*|| (laser_temp[90] < danger_threshold1 && laser_temp[115] < danger_threshold2 && laser_temp[135] < danger_threshold3)*/;
         // bool left_danger = (laser_temp[195] <= 0.51) || (laser_temp[235] <= 0.51) || (laser_temp[250] <= 0.51) /*||(laser_temp[270] < danger_threshold1 && laser_temp[245] < danger_threshold2 && laser_temp[225] < danger_threshold3)*/;

         int k; // 用於 minimum 函數的索引
         // 檢查右側危險：165-170度、120-125度、105-110度的最小值
         double spin_dis = 0.51;
         bool right_danger = (minimum(165, 171, k) <= spin_dis) || 
                           ((minimum(113, 118, k) <= spin_dis) /*&& (minimum(120, 126, k) <= spin_dis)*/) || 
                           (minimum(97, 103, k) <= spin_dis);
                           
         // 檢查左側危險：190-195度、230-235度、245-250度的最小值
         bool left_danger = (minimum(190, 196, k) <= spin_dis) || 
                           ((minimum(242, 247, k) <= spin_dis) /*&& (minimum(235, 241, k) <= spin_dis)*/) || 
                           (minimum(257, 263, k) <= spin_dis);
         double mini_1 = minimum(165, 171, k), mini_2 = minimum(113, 118, k), mini_3 = minimum(97, 103, k), mini_4 = minimum(112, 117, k);
         double mini_5 = minimum(190, 196, k), mini_6 = minimum(242, 247, k), mini_7 = minimum(257, 263, k), mini_8 = minimum(243, 248, k);
         printf("right danger = %.2f\t%.2f\t%.2f\t%.2f\n", mini_1, mini_2, mini_3, mini_4);
         printf("left  danger = %.2f\t%.2f\t%.2f\t%.2f\n", mini_5, mini_6, mini_7, mini_8);

         // bool left_danger = false, right_danger = false;
         // for(int i = 115; i <= 175; i = i+5)
         // {
         //    if(laser_temp[i] <= 0.5)
         //    {
         //       right_danger = true;
         //       break;
         //    }
         // }
         // for(int i = 185; i <= 245; i = i+5)
         // {
         //    if(laser_temp[i] <= 0.5)
         //    {
         //       left_danger = true;
         //       break;
         //    }
         // }
         
         if (left_danger || right_danger) 
         {
            danger = 1;
            if (left_danger && !right_danger) 
            {
               printf("danger state：左側過近，需原地右轉\n");
               danger_flag = 2;
               
            } 
            else if (right_danger && !left_danger) 
            {
               printf("danger state：右側過近，需原地左轉\n");
               danger_flag = 3;
               
            } 
            else 
            {
               printf("danger state：兩側過近，需原地右轉\n"); // 預設右轉，可調整
               danger_flag = 2;
               
            }
         } 
         else 
         {
            danger = 0;
            printf("safeeeeeeeeeee：no danger state\n");
         }
      }
      // wei need change
      void decide_controller(int jj) 
      {
         double distance_to_goal = sqrt(pow(goal_x - amcl_x, 2) + pow(goal_y - amcl_y, 2));
         std_msgs::Int32 controller_flag;
         if (distance_to_goal < allow_goal_rms && !reached_goal) 
         {
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            pub.publish(msg);
            ROS_INFO("Arrived at final goal!");
            ROS_INFO("#####################################################");
            reached_goal = true;
            return;
         }
         // 計算目標方向與當前方向的差異 theta_d == relative_angle
         double target_angle = atan2(goal_y - amcl_y, goal_x - amcl_x);

         // theta_d = goal_z - amcl_z;
         // if (theta_d > M_PI) theta_d -= 2 * M_PI;
         // else if (theta_d < -M_PI) theta_d += 2 * M_PI;
         // 分類方向
         // wei need change
         theta_d = target_angle - amcl_z;
         if(theta_d > M_PI)   theta_d = theta_d - 2 * M_PI;
         else if(theta_d < -M_PI)   theta_d = theta_d + 2 * M_PI;

         // printf("目標角度 (target_angle): %f 度\n", target_angle * 180 / M_PI);
         // printf("機器人角度 (amcl_z): %f 度\n", amcl_z * 180 / M_PI);
         // printf("角度差 (theta_d): %f 度\n", theta_d * 180 / M_PI);
         // printf("目標方向 (status): %d\n", status);
         // printf("機器人位置 (amcl_x, amcl_y): (%f, %f)\n", amcl_x, amcl_y);
         // printf("目標位置 (goal_x, goal_y): (%f, %f)\n", goal_x, goal_y);
         if (fabs(theta_d) <= 0.1745) 
         { // ±10° 為直行
            status = 1;
            printf("方向：前方\n");
         } 
         else if (theta_d > 0.1745) 
         { // 左轉
            status = 2;
            printf("方向：左方\n");
         } 
         else if (theta_d < -0.1745) 
         { // 右轉
            status = 3;
            printf("方向：右方\n");
         }

         // 先檢查危險狀態, 手動機制1
         check_danger(jj);
         if (danger == 1) 
         {
            if(danger_flag == 2)
            {
               msg.linear.x = 0.0;
               msg.angular.z = spin_right;
               controller_flag.data = 3; // Spin 控制器
            }
            else if(danger_flag == 3)
            {
               msg.linear.x = 0.0;
               msg.angular.z = spin_left;
               controller_flag.data = 3; // Spin 控制器
            }
            // printf("laser[165] = %f\tlaser[125] = %f\tlaser[110] = %f\n", laser_temp[165], laser_temp[125], laser_temp[110]);
            // printf("laser[195] = %f\tlaser[235] = %f\tlaser[250] = %f\n", laser_temp[195], laser_temp[235], laser_temp[250]);
            
         } 
         else
         {
            // 檢查 theta_d 範圍內是否有障礙物
            // wei need change
            bool obstacle_in_theta_d[4] = {0, 0, 0, 0}; // [1] for straight, [2] for left, [3] for right, 0代表沒障礙物，1有障礙物

            for(int i = 155; i <= 205; i = i+5)
            {
               if(laser_temp[i] <= f_along)
               {
                  obstacle_in_theta_d[1] = true;
                  printf("obstacle: SSSSSSSSSSSSSSSS\n");
                  break;
               }
            }
            for(int i = 90; i <= 155; i = i+5)
            {
               if(laser_temp[i] <= s_along)
               {
                  obstacle_in_theta_d[3] = true;
                  printf("obstacle: RRRRRRRRRRRRRRRR\n");
                  break;
               }
            }
            for(int i = 205; i <= 269; i = i+5)
            {
               if(laser_temp[i] <= s_along)
               {
                  obstacle_in_theta_d[2] = true;
                  printf("obstacle: LLLLLLLLLLLLLLLL\n");
                  break;
               }
            }
            printf("status: %d\n", status);
            if (obstacle_in_theta_d[1] || obstacle_in_theta_d[status]) // 後面是判斷障礙物是否跟目標同方向
            {
               // 使用 wei_move_along.cpp
               msg.linear.x = vel_a; // vel_a = vel from along wall
               msg.angular.z = angular_a;
               controller_flag.data = 2; // Obstacle Boundary Following 控制器
               printf("++++++++++++++++++++++++++++++++  選擇：wei_move_along.cpp  ++++++++++++++++++++++++++++\n");
               printf("left_min = %f\tright_min = %f\n", left_min, right_min);
            } 
            else 
            {
               // 使用 wei_odom.cpp
               msg.linear.x = vel_s; // vel_s = vel from search 
               msg.angular.z = angular_s;
               controller_flag.data = 1; // Target Search 控制器
               printf("+++++++++++++++++++++++++++++++  選擇：wei_odom.cpp  ++++++++++++++++++++++++++++++\n");
               printf("left_min = %f\tright_min = %f\n", left_min, right_min);
            }

         }
         // 發布最終速度
         printf("linear: %f\tangular: %f\n", msg.linear.x, msg.angular.z);
         pub.publish(msg);
         controller_flag_pub.publish(controller_flag);
      }
      void stop()
      {
         msg.linear.x =0.;
         msg.angular.z=0.;
         pub.publish(msg);
         std_msgs::Int32 controller_flag;
         controller_flag.data = 0;
         controller_flag_pub.publish(controller_flag);
         ROS_INFO("+++++++++++++++ Navigation is the  end +++++++++++++++\n");
         // ros::shutdown();	wei change 20250306
      } 
      void Search_Target(const geometry_msgs::PoseStamped::ConstPtr& goal)
      {
      int ik=10;
         target_position_x=goal->pose.position.x; // same as goal_x
         target_position_y=goal->pose.position.y;
         reached_goal = false;  // **重置到達標記，準備開始移動**
         tf2::Quaternion s;
         tf2::convert(goal->pose.orientation,s);
         tf2::Matrix3x3(s).getRPY(roll,pitch,yaw);
         target_orientation_z = yaw;
      
      }
      // wei need change, 好像沒用處
      void Path_Callback(const ::nav_msgs::Path::ConstPtr & msg) //規劃路徑後產生的所有點,這裡取第10個目標作為區域目標
      { 
         int i=0;
         std::vector<geometry_msgs::PoseStamped> data = msg->poses;
         for(std::vector<geometry_msgs::PoseStamped>::const_iterator it= data.begin(); it!= data.end(); ++it)
         {  
            position_x[i]=msg->poses[i].pose.position.x;
            position_y[i]=msg->poses[i].pose.position.y;
            // orientation_z[i]=msg->poses[i].pose.orientation.z;
            i++;       
         }  
      
         ik=10;
      } 

      // wei need change
      // void chatter1Callback(const std_msgs::Int32::ConstPtr& info1){
      //       status =  info1->data;
      // }
      void chatter2Callback(const std_msgs::Float64::ConstPtr& info2){
            error_orientation_z =  info2->data;
      }
      void speed_Search(const geometry_msgs::Twist::ConstPtr& vel1){
         vel_s = vel1->linear.x;
         angular_s = vel1->angular.z;
         recieved_message =true; 
      }
      void speed_Along_wall(const geometry_msgs::Twist::ConstPtr& vel2){
         vel_a = vel2->linear.x;
         angular_a = vel2->angular.z;
      }
      void amcl_Callback(const::move_base_msgs::MoveBaseActionFeedback::ConstPtr & amcl)
      {
         //double x_,y_;
         amcl_x= amcl->feedback.base_position.pose.position.x;
         amcl_y = amcl->feedback.base_position.pose.position.y;
         // wei need change, amcl_position_xyz and the following no use
         // amcl_position_z = amcl->feedback.base_position.pose.position.z;
         // amcl_orientation_x = amcl->feedback.base_position.pose.orientation.x;
         // amcl_orientation_y = amcl->feedback.base_position.pose.orientation.y;
         // amcl_orientation_z = amcl->feedback.base_position.pose.orientation.z; 
            // printf("\n\nread file\n");
         tf2::Quaternion q;
         tf2::convert(amcl->feedback.base_position.pose.orientation, q);
         tf2::Matrix3x3(q).getRPY(roll_t, pitch_t, yaw_t);
         amcl_z = yaw_t;
         ROS_INFO("sub_amcl_z: %f", amcl_z);
         // 座標系轉換，與 amcl_pose_sub_pub_1 一致
         // wei need change
         // if (amcl_z <= -M_PI / 2 && amcl_z >= -M_PI)
         //       amcl_z = -(M_PI + M_PI / 2 + amcl_z);
         // else
         //       amcl_z = -(amcl_z - M_PI / 2);

         // for(int i=1;i<=600;i++)
         // {
         //    // wei need change, no use   
         //    // if (sqrt(pow(position_x[i]-amcl_x,2)+pow(position_y[i]-amcl_y,2))>0.1 && sqrt(pow(position_x[i]-amcl_x,2)+pow(position_y[i]-amcl_y,2))<=0.15)
         //    // {
         //    //    x_=position_x[i];
         //    //    y_=position_y[i];
         //    //    //ik=i;
         //    // }
         //    // wei need change
         //    // double distance = sqrt(pow(amcl_x - target_position_x, 2) + pow(amcl_y - target_position_y, 2)); // 判斷是否到達終點, target_position_x == goal_x
         //    // if (distance < 0.05 && !this->reached_goal) 
         //    // { // **0.05 公尺內視為到達**
         //    //    ROS_INFO("Arrived at goal! Waiting for next target...");
         //    //    stop();  // **停止移動**
         //    //    reached_goal = true; // **確保不會重複觸發**
         //    // }
         //    //ik++;
         // }      
      }
      void save_info( int jj){
         if(AMCL_x[jj]!=0. && AMCL_y[jj]!=0.)
         {
            FILE *pfout1; //all path ;
            printf("\nread file\n");
            pfout1=fopen(save_data_path1,"a");
            if(pfout1==NULL){
               printf("Fail to open file");
               exit(1);
            }
            fprintf(pfout1,"%lf \t %lf\n", AMCL_x[jj], AMCL_y[jj]);
            fclose(pfout1);
         }
         // wei need change, 似乎沒用到
         if (right_min<=0.3 || left_min<=0.3 ||straight_min<=0.3){
            if (AMCL_x[jj]!=AMCL_x[jj-1]){   
      
               std_msgs::Float64 data_x2,data_y2; //outpt x2,y2 locate's information;
               data_x2.data=AMCL_x[jj];
               data_y2.data=AMCL_y[jj];
               x2_pub.publish(data_x2);
               y2_pub.publish(data_y2);
            } 
         }
         else if(p[jj]==1)
         {
            if (AMCL_x[jj]!=AMCL_x[jj-1])
            {   
            
               FILE *pfout2; //all path ;
               printf("\nread file\n");
               pfout2=fopen(save_search_target,"a");
                  if(pfout2==NULL){
                     printf("Fail to open file");
                     exit(1);
                  }
               fprintf(pfout2,"%lf \t %lf\n",AMCL_x[jj],AMCL_y[jj]);
               fclose(pfout2);
               

               std_msgs::Float64 data_x1,data_y1;  //outpt x1,y1 locate's information;
               data_x1.data=AMCL_x[jj];
               data_y1.data=AMCL_y[jj];
               x1_pub.publish(data_x1);
               y1_pub.publish(data_y1);
            }
         }
         else if (p[jj]==2)
         {
            if (AMCL_x[jj]!=AMCL_x[jj-1]) 
            {  
               FILE *pfout3; //all path ;
               printf("\nread file\n");
               pfout3=fopen(save_along_wall,"a");
                  if(pfout3==NULL){
                     printf("Fail to open file");
                     exit(1);
                  }
               fprintf(pfout3,"%lf \t %lf\n", AMCL_x[jj], AMCL_y[jj]);
               fclose(pfout3);
               
               std_msgs::Float64 data_x2,data_y2; //outpt x2,y2 locate's information;
               data_x2.data = AMCL_x[jj];
               data_y2.data = AMCL_y[jj];
               x2_pub.publish(data_x2);
               y2_pub.publish(data_y2);
            } 
         }  
            
         if(jj>5)
            fff += sqrt(pow(AMCL_x[jj]-AMCL_x[jj-1],2)+pow(AMCL_y[jj]-AMCL_y[jj-1],2));
      }
      void save_wall_dis_others(){
            FILE *pfout33;
            // printf("111111111111111111111\n");
            pfout33=fopen(save_every_error,"a");//"controller/20210701/save_ave_dis_error.txt"
            if(pfout33==NULL)
            {
               printf("Fail to open file");
               exit (1);
            }
            fprintf(pfout33,"%lf\t%lf\t%lf\n",left_min, right_min, straight_min);
            fclose(pfout33);
         }
};
int main(int argc,char ** argv)
{
   ros::init(argc, argv, "cmd_sub_pub"); 
   cmd_sub_pub obj;
	//ros::Rate a(100);  
	
	while(counts < max_step && ros::ok())
	{ 
      // wei need change
      // if(target_position_x!=0)
		// {
         counts++;
      // }
      cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<endl<<endl;
		printf("counts: %d \n",counts);
      obj.decide_controller(counts);
      // obj.decision_path(right_min, left_min,straight_min,counts);  // wei need change, no use anymore
      obj.publisher_info(counts);
      obj.save_position_data(counts);
      obj.save_info(counts);
      // wei need change
      // if ( msg.linear.x ==0 & msg.angular.z==0 && counts>100 ){
      //    cout<<"in the end "<<endl;
      //     obj.stop();
      // }
      obj.save_wall_dis_others();
      if(counts < 5)
      {
         obj.stop();
      }


      FILE *pfout_vel; //all path ;
      printf("\nread file\n");
      pfout_vel=fopen(save_velocity_every,"a");
      if(pfout_vel==NULL)
      {
         printf("Fail to open file");
         exit(1);
      }  
      fprintf(pfout_vel,"%lf \n ",msg.linear.x);
      fclose(pfout_vel); //all path ;

		ros::spinOnce();
      ros::Duration(0.01).sleep();
	}
      FILE *pfout4; //all path ;
      printf("\nread file\n");
      pfout4=fopen(save_distance,"a");
      if(pfout4==NULL){
         printf("Fail to open file");
         exit(1);
      }  
      fprintf(pfout4,"%lf \n",fff);
      fclose(pfout4);

      FILE *pfoutv; //all path ;
      printf("\nread file\n");
      pfoutv=fopen(save_velocity,"a");
      if(pfoutv==NULL){
         printf("Fail to open file");
         exit(1);
      }  
      fprintf(pfoutv,"%d \t %lf\t %lf\n",counts,(double)counts/100.,fff / ((double)counts/100.));
      fclose(pfoutv); //all path ;
    
    return 0;
}



