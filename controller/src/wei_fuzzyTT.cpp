#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <chrono>
#include <time.h>
#include <algorithm>
#include <ros/ros.h>
// #include "b-spline.h"
////#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/LaserScan.h"
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>

#define save_distance      "/home/user/wei_ws/src/controller/src/position_random/2easy-distance.txt"  //左/右/前測距離
#define save_local_target  "/home/user/wei_ws/src/controller/src/position_random/local_target-111.txt"//區域目標
#define save_local_target_change  "position_random/local_target-change1.txt"//區域目標
#define save_old "/home/user/wei_ws/src/controller/src/position_random/old_.txt"
#define save_new "/home/user/wei_ws/src/controller/src/position_random/new_.txt"
#define load_data_clu "/home/user/wei_ws/src/controller/src/fuzzy_controller/along_wall_controller/10.txt" // 規則數量文件
#define load_data_FC "/home/user/wei_ws/src/controller/src/fuzzy_controller/fuzzy_TT_controller/temp_direct_speed.txt" // 修改後: 4輸入2輸出的控制器參數
#define position_target    "/home/user/wei_ws/src/controller/src/position_random/position_targetxx.txt"

#define max_step 40000
#define _out_varl 2   /*  輸出變數 (左輪速, 右輪速) */
#define _in_varl  4   /*  輸入變數    */
#define _rule_number 10   /* 規則數 (這個值應該與 load_data_clu 中的值一致, 或者動態讀取) */
#define  _max_rule  30    /* 開記憶體空間 */
#define _rule_delta 10 // 可能是用於 test_open 中讀取規則的數量上限，應與 _rule_number 相關
#define left_wheel_speed 0.5  /* 左輪輪速上限 (m/s 或 rad/s, 根據實際情況) */
#define right_wheel_speed 0.5 /* 右輪輪速上限 (m/s 或 rad/s) */
#define radius 0.03 // 輪子半徑 (m)，如果輪速是角速度則需要

// 新增：模糊控制器輸出輪速的範圍
#define _WHEEL_SPEED_MAX  0.5  // 假設最大線速度為 0.5 m/s
#define _WHEEL_SPEED_MIN -0.5 // 假設最小線速度（允許後退）為 -0.5 m/s
                               // 或者 _WHEEL_SPEED_MIN 0.0 如果不允許後退


#define MIN(a,b) (a < b ? a : b)
// #define _input_scale (1/sensor_range)  /* 給感測器讀值正規化的參數  */
#define randomize() srand((unsigned) time(NULL)) /*  以時間為基底產生隨機值的初始化，不要動這行  */
#define random(x) (rand() % x)   /*  隨機產生亂數，不要動這行  */

double front_safe_distance = 0.4;
double side_safe_distance = 0.4;
double weight_speed;
double in[_in_varl+1] ; //感測器正規化後的輸入
double previous_in3 = 0.0; // 用於計算 in[4] (距離誤差的變化)

// out_y 不再是PID參數，而是直接的輪速 (v_left, v_right)
// 但由於 Search_Target 中直接使用 K[]，out_y[] 可能不再直接使用
// double out_y[_out_varl+1] ; //馬達輸出

const int _mem_length = 2*_in_varl+ _out_varl ; // 一個rule的長度

int _in_clu ; //rule_number的實際讀取值
int status;
int counts=0;
int a, ik, ss = 111;
int HH;
double final_z ;

double fuzzy_real[_mem_length*_max_rule+1] ; //  實際輸出規則的變數
double K[_out_varl+1] ; // K[1] 左輪速, K[2] 右輪速 (模糊控制器輸出)
double  ave_motor_speed = 0.0;  //平均輪速
double  ave_motor_displacement = 0.0;  //平均位移 ///202011
double min_m[_in_varl+1],max_m[_in_varl+1];//最小中心點 & 最大中心點

double local_target_x[max_step + 1],
        local_target_y[max_step + 1],
        local_target_z[max_step + 1]; // 紀錄全局路線的方向
double amcl_x, amcl_y, amcl_z;
double local_target_x_record[max_step + 1], local_target_y_record[max_step + 1];
double local_error_x, local_error_y;
double local_error_z, local_error_z_deg;
double trans_new_amcl;//重要
double trans_degree;
double error_orientation_z, error_final_z, Error_orientation_z[max_step+1];
double goal_x, goal_y, goal_z;
double dis_x, dis_y, dis_z; // 現在位置與目標的距離

// 移除 PID 相關變數
// double error, intergral ,derivative, lastError, error_1, derivative_1, lastError_1, intergral_1;
double v_left_cmd, v_right_cmd; // 修改後：直接的左右輪速度指令
double final_x,final_y,final_zz;
double angular,linear; // 實際發佈的角速度和線速度
double angular1,linear1; // 從 topic 讀取的數據
double laser_temp[361],margin_laser[361],left_min, right_min, straight_min;
float laser_temp_scan[897];
// Tp 和 turn 不再需要
// double Tp = 0, turn, angle_deviate;
double angle_deviate; // 僅用於 publisher_info
double rms, local_rms;
double roll,pitch,yaw;
double roll_g,pitch_g,yaw_g;
double roll_s,pitch_s,yaw_s;
double sum_error; // 局部目標距離誤差
int js = 0;
const float wheel_distance = 0.3791; // distance between two wheel (m)

using namespace std;
geometry_msgs::Twist msg;
///////////////// 高斯函數 ////////////////////{

inline double phi(double x,double m,double v)
{ return(  exp( - (x-m)*(x-m)/(v*v) )) ; }

class C_Rule
{
  friend class Population ; // 如果沒有 Population class，這行可以移除
  public :
    double  in_mu_1, in_mu_2;
    double  con[_out_varl+1] ; // _out_varl 已改為 2
    void    In_mem_fir(double *,int) ;
    friend  void Fir_str(double *, int) ;
};

C_Rule  _Rule[_max_rule+1];
void C_Rule::In_mem_fir(double *in_params,int _rule) // 參數名改為 in_params 避免與全域 in 衝突
{
  int i;
  in_mu_1 =  1. ;
  // in_mu_2 = 1.; // 如果只用一個 in_mu_
  for(i=1;i<= _in_varl;i++) // _in_varl 已改為 4
  {
    in_mu_1 =  in_mu_1 * phi(in_params[i] , fuzzy_real[(_rule-1)*_mem_length+i*2-1] , fuzzy_real[(_rule-1)*_mem_length+i*2] ) ;
  }
}
class amcl_pose_sub_pub_1
{
  private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Publisher pub_goal;
    ros::Publisher chatter_pub2;
    ros::Subscriber sub_final_goal;
    ros::Subscriber sub_path;
    ros::Subscriber sub_amcl;
    ros::Subscriber sub_laser;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_data1;
    ros::Subscriber sub_data2;
    ros::Subscriber sub_data3;
    ros::Subscriber sub_data4;
  public:
    amcl_pose_sub_pub_1()
    {
      pub=n.advertise<geometry_msgs::Twist>("/cmd_vel_1",3000);
      pub_goal=n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
      chatter_pub2=n.advertise<std_msgs::Float64>("chatter2",3000);
      sub_laser=n.subscribe("/scan",3000,&amcl_pose_sub_pub_1::lasercallback,this);
      sub_final_goal = n.subscribe("/move_base_simple/goal", 3000, &amcl_pose_sub_pub_1::Final_Goal,this);
      sub_path = n.subscribe("/move_base/NavfnROS/plan", 3000, &amcl_pose_sub_pub_1::Path_Callback,this);
      sub_amcl = n.subscribe("/move_base/feedback", 3000, &amcl_pose_sub_pub_1::amcl_Callback,this);
      sub_odom = n.subscribe("/odom",3000, &amcl_pose_sub_pub_1::final_Callback,this); // final_Callback 似乎未使用
      sub_data1 = n.subscribe("/info_1",3000, &amcl_pose_sub_pub_1::chatter1Callback1, this);
      sub_data2 = n.subscribe("/info_2",3000, &amcl_pose_sub_pub_1::chatter1Callback2, this); // HH
      sub_data3 = n.subscribe("/info_3",3000, &amcl_pose_sub_pub_1::chatter1Callback3, this); // linear1
      sub_data4 = n.subscribe("/info_4",3000, &amcl_pose_sub_pub_1::chatter1Callback4, this); // angular1

      // 移除類內部的函數原型聲明，因為它們已經在類外部實現或將在類外部實現
    }
    void chatter1Callback1(const std_msgs::Float64::ConstPtr& info_1){
      ss =  info_1->data;
    }
    void chatter1Callback2(const std_msgs::Int32::ConstPtr& info_2){
      HH =  info_2->data;
    }
    void chatter1Callback3(const std_msgs::Int32::ConstPtr& info_3){
      linear1 =  info_3->data; // 注意類型轉換，如果 info_3 是 Int32
    }
    void chatter1Callback4(const std_msgs::Int32::ConstPtr& info_4){
      angular1 =  info_4->data; // 注意類型轉換
    }
    void Final_Goal(const geometry_msgs::PoseStamped::ConstPtr & F_goal)
    {
      goal_x = F_goal->pose.position.x;
      goal_y = F_goal->pose.position.y;
      tf2::Quaternion s_tf; // 避免與全域 s 衝突
      tf2::convert(F_goal->pose.orientation,s_tf);
      tf2::Matrix3x3(s_tf).getRPY(roll_s,pitch_s,yaw_s);
      goal_z = yaw_s;
      if( (goal_z <= -M_PI/2) && goal_z >= -M_PI)
        goal_z= -(M_PI/2 + (M_PI + goal_z));
      else
        goal_z = -(goal_z - M_PI/2);
    }

    void final_Callback(const nav_msgs::Odometry::ConstPtr& msg_odom) // 參數名修改避免衝突
    {
      final_x = msg_odom->pose.pose.position.x;
      final_y = msg_odom->pose.pose.position.y;
      // final_zz = msg_odom->pose.pose.orientation.z; // 四元數的z分量不直接是yaw角
      tf2::Quaternion q_odom;
      tf2::convert(msg_odom->pose.pose.orientation, q_odom);
      double temp_roll, temp_pitch;
      tf2::Matrix3x3(q_odom).getRPY(temp_roll, temp_pitch, final_zz); // 獲取yaw角
    }
    void lasercallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
      // ... (lasercallback 邏輯基本不變) ...
      for (int i = 0; i < scan->ranges.size(); ++i) // scan->ranges.size() 是點數, 索引從0開始
      {
        double angle_rad = (scan->angle_min + i * scan->angle_increment); // ROS雷達角度通常是以雷達前方為0度
        // 將 ROS 標準雷達坐標系 (前方x軸, 左方y軸) 轉換為您代碼中習慣的0-360度 (可能需要調整)
        // 假設您的 laser_temp[0] 是雷達的正後方，laser_temp[180] 是正前方
        angle_rad += M_PI; // 偏移180度
        if (angle_rad > 2 * M_PI) angle_rad -= 2*M_PI;
        if (angle_rad < 0) angle_rad += 2*M_PI;

        int angle_deg = static_cast<int>(angle_rad * 180.0 / M_PI);
        angle_deg = angle_deg % 360; // 確保在0-359之間

        if (std::isinf(scan->ranges[i]))
        {
            laser_temp[angle_deg] = 10.4999; // 最大探測距離
        }
        else if (scan->ranges[i] <= 0.15) // 假設0.15是雷達盲區或無效讀數
        {
            laser_temp[angle_deg] = 0.15001; // 略大於盲區的值
        }
        else
        {
            laser_temp[angle_deg] = scan->ranges[i];
        }
      }
    }
    void amcl_Callback(const::move_base_msgs::MoveBaseActionFeedback::ConstPtr & amcl_feedback) //參數名修改
    {
      amcl_x = amcl_feedback->feedback.base_position.pose.position.x;
      amcl_y = amcl_feedback->feedback.base_position.pose.position.y;
      tf2::Quaternion q_amcl; //參數名修改
      tf2::convert(amcl_feedback->feedback.base_position.pose.orientation,q_amcl);
      tf2::Matrix3x3(q_amcl).getRPY(roll,pitch,yaw);
      amcl_z = yaw; // amcl_z 直接是弧度制的 yaw
      // 您的坐標系轉換:
      // if( amcl_z <= -M_PI/2 && amcl_z >= -M_PI)
      //   amcl_z= -( M_PI + M_PI/2 + amcl_z);
      // else
      //   amcl_z = -(amcl_z - M_PI/2);
      // 注意：ROS中amcl輸出的yaw角通常在[-PI, PI]範圍內，前方為0度。
      // 如果您需要不同的坐標系，請確認轉換邏輯。
    }
    void Path_Callback(const ::nav_msgs::Path::ConstPtr & path_msg) //參數名修改
    {
      double xx1_prev = 0, yy1_prev = 0; // 用於比較路徑點是否改變
      int i=0;
      std::vector<geometry_msgs::PoseStamped> data = path_msg->poses;
      if (data.empty()) {
          ROS_WARN("Received empty path in Path_Callback");
          ik = -1; // 表示沒有有效路徑點
          return;
      }

      double min_dist_to_robot = std::numeric_limits<double>::max();
      int best_ik = 0; // 先選擇第一個點作為備選

      for(i = 0; i < data.size(); ++i)
      {
        local_target_x[i] = path_msg->poses[i].pose.position.x;
        local_target_y[i] = path_msg->poses[i].pose.position.y;
        tf2::Quaternion g_tf; //參數名修改
        tf2::convert(path_msg->poses[i].pose.orientation,g_tf);
        tf2::Matrix3x3(g_tf).getRPY(roll_g,pitch_g,yaw_g);
        local_target_z[i] = yaw_g; // yaw_g 是弧度

        // 尋找路徑上最接近但又在前方一定距離的點，或者一個固定的預瞄點
        double dist_to_robot = sqrt(pow(amcl_x-local_target_x[i],2)+pow(amcl_y-local_target_y[i],2));

        // 選擇一個預瞄距離（例如1米）的點，或者路徑上第N個點
        // 這裡的邏輯是選擇一個在合理預瞄範圍內的點
        double lookahead_distance = 1.0; // 可調參數
        if (dist_to_robot >= lookahead_distance * 0.8 && dist_to_robot < lookahead_distance * 1.5) {
            if (dist_to_robot < min_dist_to_robot) { // 確保選擇的是較近的滿足條件的點
                 min_dist_to_robot = dist_to_robot;
                 best_ik = i;
            }
        }
      }
      // 如果沒有找到理想的預瞄點，可以選擇路徑上固定索引的點，或者最後一個點
      if (min_dist_to_robot == std::numeric_limits<double>::max()) {
          if (data.size() > 10) best_ik = 10; // 選擇第10個點
          else if (!data.empty()) best_ik = data.size() -1; // 選擇最後一個點
          else best_ik = 0; // 預設
      }
      ik = best_ik;


      if (data.size() > 0 && (xx1_prev != local_target_x[ik] || yy1_prev != local_target_y[ik]))
      {
        FILE *pfoutold;
        pfoutold=fopen(position_target,"a");
        if(pfoutold==NULL) { /* ... error handling ...*/ }
        else {
            // 記錄選擇的局部目標點
            fprintf(pfoutold,"%lf \t %lf\t",local_target_x[ik],local_target_y[ik]);
            fprintf(pfoutold,"\n");
            fclose(pfoutold);
        }
        xx1_prev = local_target_x[ik];
        yy1_prev = local_target_y[ik];
      }

      // 如果路徑點太少，直接使用全局目標
      if(data.size() <= 20 && data.size() > 0)
      {
          local_target_x[ik] = goal_x;
          local_target_y[ik] = goal_y;
          local_target_z[ik] = goal_z; // goal_z 也是弧度
      } else if (data.empty()) {
          // 如果路徑為空，可能意味著已到達目標或規劃失敗，直接使用全局目標
          local_target_x[ik] = goal_x;
          local_target_y[ik] = goal_y;
          local_target_z[ik] = goal_z;
          ROS_WARN_ONCE("Path is empty, using global goal as local target.");
      }
    }
    void coordinate_transform (int jj)
    {
      if (ik < 0 || ik >= max_step) { // 檢查 ik 的有效性
          ROS_ERROR("Invalid ik value in coordinate_transform: %d", ik);
          // 可以設置一個默認行為，例如讓機器人停下
          local_error_x = 0;
          local_error_y = 0;
          error_orientation_z = 0; // 避免未初始化
          sum_error = 0;
      } else {
          local_error_x = local_target_x[ik] - amcl_x;
          local_error_y = local_target_y[ik] - amcl_y;
      }
      dis_x = goal_x - amcl_x;
      dis_y = goal_y - amcl_y;
      rms = sqrt(pow(dis_x,2)+pow(dis_y,2)); // 全局目標距離

      // error_final_z = goal_z - amcl_z; // 全局目標角度差，注意角度的周期性
      // 規範化角度差到 [-PI, PI]
      error_final_z = atan2(sin(goal_z - amcl_z), cos(goal_z - amcl_z));

    }
    void fuzzy_in(int jj)
    {
        if (ik < 0) { // 如果ik無效 (例如路徑為空)
            ROS_WARN("fuzzy_in: ik is invalid, setting inputs to zero.");
            in[1] = 0.0;
            in[2] = 0.0;
            in[3] = 0.0;
            in[4] = 0.0;
            error_orientation_z = 0.0; // 確保 error_orientation_z 被初始化
            sum_error = rms; // 如果沒有局部目標，距離誤差就是全局距離誤差
            Error_orientation_z[jj] = 0.0;
            previous_in3 = in[3];
            return;
        }

        // 計算指向局部目標的角度 local_error_z_rad (弧度)
        double local_error_z_rad = atan2(local_error_y, local_error_x);

        // 計算機器人需要轉向的角度 (弧度), error_orientation_z
        // error_orientation_z = 局部目標方向 - 機器人當前方向
        error_orientation_z = local_error_z_rad - amcl_z;
        // 規範化到 [-PI, PI]
        while(error_orientation_z > M_PI) error_orientation_z -= 2*M_PI;
        while(error_orientation_z < -M_PI) error_orientation_z += 2*M_PI;

        Error_orientation_z[jj] = error_orientation_z;

        // 輸入1: 角度誤差的絕對值正規化
        in[1] = fabs(error_orientation_z) / M_PI; // 正規化到 [0, 1]

        // 輸入2: 角度誤差變化率正規化
        if (jj == 1 || counts <= 1) { // 第一次
            in[2] = 0.0;
        } else {
            double delta_angle_error = Error_orientation_z[jj] - Error_orientation_z[jj-1];
            // 處理角度跳變 (從 PI 到 -PI 或反之)
            if (delta_angle_error > M_PI) delta_angle_error -= 2*M_PI;
            if (delta_angle_error < -M_PI) delta_angle_error += 2*M_PI;
            // 正規化，假設變化率在 +/- (M_PI/2) rad/step 內
            in[2] = fabs(delta_angle_error) / (M_PI/2.0) ; // 正規化到 [0, 1] (假設變化範圍)
            if (in[2] > 1.0) in[2] = 1.0;
        }

        // 輸入3: 距離局部目標的誤差正規化
        sum_error = sqrt(pow(local_error_x, 2)+pow(local_error_y, 2));
        double max_sum_error_for_norm = 2.0; // 假設正規化時的最大距離誤差為2米
        in[3] = sum_error / max_sum_error_for_norm;
        if (in[3] > 1.0) in[3] = 1.0;

        // 輸入4: 距離誤差的變化率正規化
        if (jj == 1 || counts <=1) {
            in[4] = 0.0;
        } else {
            double delta_in3 = in[3] - previous_in3;
            // 正規化 in[4]。假設距離正規化後，其變化率在 +/- 0.2 之間
            double norm_factor_in4 = 0.2;
            in[4] = delta_in3 / norm_factor_in4; // 映射到 [-1, 1] 附近
            // 再將其映射到 [0, 1] 給高斯函數
            in[4] = (in[4] + 1.0) / 2.0;
            if (in[4] > 1.0) in[4] = 1.0;
            if (in[4] < 0.0) in[4] = 0.0;
        }
        previous_in3 = in[3]; // 更新 previous_in3

        // ROS_INFO("Fuzzy Inputs: in[1]=%.2f, in[2]=%.2f, in[3]=%.2f, in[4]=%.2f", in[1],in[2],in[3],in[4]);
        // ROS_INFO("LocalTarget(ik:%d): x=%.2f, y=%.2f. Robot: x=%.2f, y=%.2f, z=%.2f(rad)", ik, local_target_x[ik], local_target_y[ik], amcl_x, amcl_y, amcl_z);
        // ROS_INFO("error_orientation_z=%.2f (rad)", error_orientation_z);
    }

    void decideRotation(int &collision_flag) // 參數名修改
    {
      collision_flag = 0; // 默認沒有碰撞風險
      int k_laser_idx; // 避免與全域K衝突
      left_min = minimum(205, 269, k_laser_idx); // 假設雷達數據 laser_temp 索引對應角度
      right_min = minimum(90, 155, k_laser_idx);
      straight_min = minimum(155, 205, k_laser_idx); // 假設180度是前方

      // 這裡的 margin_laser 是固定值，用於碰撞檢測
      // 實際應用中，這些值可能需要根據機器人形狀動態計算，或者使用 costmap
      // 簡易碰撞檢測示例：
      if (straight_min < front_safe_distance * 0.8) { // 前方太近
          collision_flag = 1;
          ROS_WARN("Collision Warning: Front too close! straight_min=%.2f", straight_min);
          return;
      }
      if (left_min < side_safe_distance * 0.8) { // 左側太近
          collision_flag = 2; // 表示左側有障礙
          ROS_WARN("Collision Warning: Left too close! left_min=%.2f", left_min);
          return;
      }
      if (right_min < side_safe_distance * 0.8) { // 右側太近
          collision_flag = 3; // 表示右側有障礙
          ROS_WARN("Collision Warning: Right too close! right_min=%.2f", right_min);
          return;
      }
    }
    void Search_Target(int jj)
    {
      int collision_status = 0;
      // decideRotation(collision_status); // 如果需要碰撞避免，取消註釋並實現其邏輯

      // if (collision_status != 0) {
      //    ROS_WARN("Collision detected by decideRotation, attempting avoidance.");
      //    // 在這裡實現碰撞避免邏輯，例如：
      //    if (collision_status == 1) { // 前方障礙
      //        msg.linear.x = 0.0;
      //        msg.angular.z = 0.5; // 嘗試右轉
      //    } else if (collision_status == 2) { // 左方障礙
      //        msg.linear.x = 0.1; // 慢速前進
      //        msg.angular.z = -0.3; // 右轉
      //    } else if (collision_status == 3) { // 右方障礙
      //        msg.linear.x = 0.1;
      //        msg.angular.z = 0.3; // 左轉
      //    }
      //    pub.publish(msg);
      //    linear = msg.linear.x;
      //    angular = msg.angular.z;
      //    return;
      // }

      // K[1] 是模糊控制器輸出的左輪目標線速度 (m/s)
      // K[2] 是模糊控制器輸出的右輪目標線速度 (m/s)
      // 這些值應該已經在 _WHEEL_SPEED_MIN 和 _WHEEL_SPEED_MAX 之間
      v_left_cmd = K[1];
      v_right_cmd = K[2];

      // 進行速度限制 (雖然模糊系統的輸出應該已經在範圍內，但多一層保險)
      if (v_left_cmd > _WHEEL_SPEED_MAX) v_left_cmd = _WHEEL_SPEED_MAX;
      if (v_left_cmd < _WHEEL_SPEED_MIN) v_left_cmd = _WHEEL_SPEED_MIN;
      if (v_right_cmd > _WHEEL_SPEED_MAX) v_right_cmd = _WHEEL_SPEED_MAX;
      if (v_right_cmd < _WHEEL_SPEED_MIN) v_right_cmd = _WHEEL_SPEED_MIN;

      // 從左右輪速計算機器人的線速度和角速度
      msg.linear.x = (v_left_cmd + v_right_cmd) / 2.0;
      msg.angular.z = (v_right_cmd - v_left_cmd) / wheel_distance; // 注意正負號和輪距

      // 可以對最終的線速度和角速度再做一次全局限制
      double max_linear_vel = 0.7; // m/s
      double max_angular_vel = 1.0; // rad/s
      if (msg.linear.x > max_linear_vel) msg.linear.x = max_linear_vel;
      if (msg.linear.x < -max_linear_vel) msg.linear.x = -max_linear_vel; // 如果允許後退
      if (msg.angular.z > max_angular_vel) msg.angular.z = max_angular_vel;
      if (msg.angular.z < -max_angular_vel) msg.angular.z = -max_angular_vel;

      pub.publish(msg);
      linear = msg.linear.x ;
      angular = msg.angular.z;
      // ROS_INFO("Published cmd_vel: linear=%.2f, angular=%.2f (From v_L=%.2f, v_R=%.2f)", linear, angular, v_left_cmd, v_right_cmd);
    }
  // speed_limit() 不再需要，因為輪速直接由模糊控制器輸出並在Search_Target中限制
  /*
  void speed_limit()
  {
    // 此函數不再適用於直接輪速輸出模型
  }
  */
  void Final_stop(int jj)
  {
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    pub.publish(msg);
    linear = msg.linear.x; // 更新記錄值
    angular = msg.angular.z;
    ROS_INFO("Final_stop called at count %d. Robot stopped.", jj);
  }
  void publisher_info(int jj )
  {
    std_msgs::Float64 info2_msg; // 避免與類成員衝突
    angle_deviate = error_orientation_z * 180.0 / M_PI; // 轉換為角度發布
    info2_msg.data = angle_deviate; // 發布角度誤差（度）
    chatter_pub2.publish(info2_msg);
  }
  void local_target(int jj)
  {
    if (ik < 0) return; // 無有效局部目標

    local_target_x_record[jj] = local_target_x[ik];
    local_target_y_record[jj] = local_target_y[ik];
    if (jj > 0 && (local_target_x_record[jj] != local_target_x_record[jj-1] || local_target_y_record[jj] != local_target_y_record[jj-1] ))
    {
      FILE *pfout1;
      pfout1=fopen(save_local_target,"a");
      if(pfout1!=NULL) {
        fprintf(pfout1,"%lf \t %lf \n", local_target_x[ik], local_target_y[ik]);
        fclose(pfout1);
      } else {
          ROS_ERROR("Failed to open file: %s", save_local_target);
      }
    }
  }
  void stop_at_max_step() // 函數名修改，避免與C++ STL的stop衝突
  {
    if(counts >= max_step) // 等於或大於
    {
      msg.linear.x = 0.;
      msg.angular.z = 0.;
      pub.publish(msg);
      ROS_INFO("Max steps reached. Robot stopped.");
    }
  }
  void Fir_str(double *in_params, int rule_count) //參數名修改, rule_count 是實際使用的規則數
  {
    int j, k;
    for(k=1; k<=rule_count; k++)
    {
      for (j=1; j<=_out_varl; j++) // _out_varl is 2
      {
        // 檢查 fuzzy_real 索引是否越界
        int index = k*_mem_length-_out_varl+j;
        if (index < 1 || index > _mem_length*_max_rule) {
            ROS_ERROR("Fir_str: fuzzy_real index out of bounds: %d for rule %d, out_var %d", index, k, j);
            _Rule[k].con[j] = 0.0; // 安全賦值
            continue;
        }
        _Rule[k].con[j] = fuzzy_real[index] ;
      }
      _Rule[k].In_mem_fir(in_params,k) ;
    }
  }
  void fuzzy(int rule_count) // rule_count 是實際使用的規則數
  {
    int i, j_out; // 避免與迴圈變量j衝突
    double den[_out_varl+1],
          num[_out_varl+1] ;
    for (j_out=1; j_out<=_out_varl; j_out++) // _out_varl is 2
    {
      den[j_out] = 0. ;
      num[j_out] = 0. ;

      for (i=1; i<=rule_count; i++)
      {
        num[j_out] = num[j_out] + _Rule[i].in_mu_1 * _Rule[i].con[j_out];
        den[j_out] = den[j_out] + _Rule[i].in_mu_1 ;
      }
      if ( fabs(den[j_out]) < 1E-8 )
        K[j_out] = 0.;
      else
        K[j_out] = num[j_out]/den[j_out] ;
    }
    // ROS_INFO("Fuzzy output K: K[1](vL)=%.2f, K[2](vR)=%.2f", K[1], K[2]);
  }
  void test_open()
  {
    for(int ssss=1; ssss<=_in_varl; ssss++) // _in_varl is 4
    {
        min_m[ssss]=1.0;
        max_m[ssss]=0.0;
    }
    FILE *fnoise1_rules,*fnoise0_count; // 變數名修改

    fnoise0_count=fopen(load_data_clu,"r");
    if(fnoise0_count==NULL) {
      ROS_FATAL("Cannot open rule count file: %s. Exiting.", load_data_clu);
      ros::shutdown(); // 嚴重錯誤，關閉節點
      exit(1); // 確保退出
    }
    if (fscanf(fnoise0_count,"%d", &_in_clu) != 1) { // _in_clu 是實際的規則數
        ROS_FATAL("Failed to read rule count from %s. Exiting.", load_data_clu);
        fclose(fnoise0_count);
        ros::shutdown();
        exit(1);
    }
    fclose(fnoise0_count);
    ROS_INFO("Successfully read rule count: _in_clu = %d", _in_clu);

    if (_in_clu <= 0 || _in_clu > _max_rule) {
        ROS_FATAL("Invalid rule count %d (must be >0 and <= %d). Exiting.", _in_clu, _max_rule);
        ros::shutdown();
        exit(1);
    }


    fnoise1_rules=fopen(load_data_FC,"r"); // load_data_FC 包含4輸入2輸出的規則
    if(fnoise1_rules==NULL) {
      ROS_FATAL("Cannot open fuzzy rules file: %s. Exiting.", load_data_FC);
      ros::shutdown();
      exit(1);
    }

    // _mem_length = 2*_in_varl + _out_varl = 2*4 + 2 = 10
    // 每個規則有10個double值
    for(int rule_idx=0; rule_idx < _in_clu; rule_idx++) { // 遍歷每個規則
        for (int param_idx = 0; param_idx < _mem_length; ++param_idx) { // 遍歷規則內的每個參數
            int current_fuzzy_real_idx = rule_idx * _mem_length + param_idx + 1; // fuzzy_real 索引從1開始
            if (current_fuzzy_real_idx > _mem_length * _max_rule) {
                ROS_ERROR("test_open: fuzzy_real index out of bounds while reading rule %d, param %d", rule_idx + 1, param_idx + 1);
                fclose(fnoise1_rules);
                ros::shutdown();
                exit(1);
            }
            if (fscanf(fnoise1_rules,"%lf", &fuzzy_real[current_fuzzy_real_idx]) != 1) {
                ROS_ERROR("test_open: Failed to read parameter for rule %d, param %d from %s", rule_idx + 1, param_idx + 1, load_data_FC);
                fclose(fnoise1_rules);
                ros::shutdown();
                exit(1);
            }
        }
    }
    fclose(fnoise1_rules);
    ROS_INFO("Successfully loaded %d fuzzy rules from %s.", _in_clu, load_data_FC);

    // 計算 min_m 和 max_m (這部分通常在訓練時確定，如果這是部署，可能不需要)
    // _rule_delta 應該是 _in_clu
    for(int jj=1; jj<=_in_clu; jj++)
    {
      for(int jjj=1; jjj<=_in_varl; jjj++) // _in_varl is 4
      {
        // 檢查索引
        int center_idx = (jj-1)*_mem_length+jjj*2-1;
        if (center_idx < 1 || center_idx > _mem_length*_max_rule) continue;

        if(fuzzy_real[center_idx] < min_m[jjj])
        {
          min_m[jjj]=fuzzy_real[center_idx];
        }
        if(fuzzy_real[center_idx] > max_m[jjj])
        {
          max_m[jjj]=fuzzy_real[center_idx];
        }
      }
    }
  }
  void Information(int jj)
  {
    printf("--- Count: %d ---\n", jj);
    printf("Robot Pose (amcl): x=%.2f, y=%.2f, yaw=%.2f rad (%.1f deg)\n", amcl_x, amcl_y, amcl_z, amcl_z*180/M_PI);
    if (ik >= 0) {
        printf("Local Target (ik=%d): x=%.2f, y=%.2f\n", ik, local_target_x[ik], local_target_y[ik]);
    } else {
        printf("Local Target: INVALID (ik=%d)\n", ik);
    }
    printf("Global Target: x=%.2f, y=%.2f, yaw=%.2f rad (%.1f deg)\n", goal_x, goal_y, goal_z, goal_z*180/M_PI);
    printf("Distance to Global Goal (rms): %.2f m\n", rms);
    printf("Local Orientation Error: %.2f rad (%.1f deg)\n", error_orientation_z, error_orientation_z*180/M_PI);
    printf("Fuzzy Inputs: ");
    for (int i_in=1 ;i_in<=_in_varl;i_in++) printf("in[%d]=%.3f  ", i_in, in[i_in]);
    printf("\n");
    printf("Fuzzy Outputs (K values -> Wheel Speeds m/s): ");
    for (int i_k=1; i_k<=_out_varl;i_k++) printf("K[%d]=%.3f  ",i_k,K[i_k]);
    printf("\n");
    printf("Commanded Velocities: Linear=%.3f m/s, Angular=%.3f rad/s\n", linear, angular);
    printf("---------------------\n\n");
  }
  double minimum(int start_angle, int end_angle ,int &idx_at_min) //參數名修改
  {
    double laser_min_val = 100.0; // 初始化為較大值
    idx_at_min = start_angle; // 初始化索引
    for (int current_angle_idx = start_angle ; current_angle_idx <= end_angle; current_angle_idx++)
    {
      int actual_idx = current_angle_idx % 360; // 確保索引在0-359
      if (laser_min_val > laser_temp[actual_idx])
      {
        laser_min_val = laser_temp[actual_idx];
        idx_at_min = actual_idx;
      }
    }
    return laser_min_val;
  }

}; // class amcl_pose_sub_pub_1 END

int main(int argc,char ** argv)
{
  // auto start = chrono::high_resolution_clock::now(); // 如果不用於計時可以移除
  ros::init(argc,argv,"fuzzy_direct_controller_node"); // 節點名修改
  amcl_pose_sub_pub_1 ctrl;
  ros::Rate loop_rate(20); // 控制迴圈頻率，例如20Hz

  // 在迴圈開始前載入一次規則
  ctrl.test_open();
  if (!ros::ok()) return 0; // 如果test_open中關閉了ros，則退出

  while(counts < max_step && ros::ok())
  {
    if (goal_x == 0 && goal_y == 0 && counts > 100) { // 假設(0,0)不是有效目標點，或者需要一個flag來啟動
        // ROS_INFO_THROTTLE(5, "No valid goal set yet or goal is (0,0). Waiting...");
        // ctrl.Final_stop(counts); // 保持停止狀態
        // ros::spinOnce();
        // loop_rate.sleep();
        // continue; // 等待目標
    }

    counts++;
    // ROS_INFO("Processing count: %d", counts);

    ctrl.coordinate_transform(counts);
    ctrl.fuzzy_in(counts) ;
    ctrl.Fir_str(in , _in_clu) ; // 使用實際讀取的規則數 _in_clu
    ctrl.fuzzy(_in_clu);       // 使用實際讀取的規則數 _in_clu

    ctrl.publisher_info(counts); // 發布調試資訊

    if(counts < 10){ // 初始幾步可能amcl未穩定，或給系統一點時間
      ctrl.Final_stop(counts);
      ROS_INFO("Initial steps, robot stopped.");
    } else {
        if(rms > 0.15){ // 如果離全局目標還遠
          ctrl.Search_Target(counts);
        }
        else // 認為已到達全局目標附近
        {
         ROS_INFO("Close to global goal (rms=%.2f). Stopping.", rms);
         ctrl.Final_stop(counts);
         // 可以考慮在這裡 break 或者設置一個標誌讓迴圈結束
         // break;
        }
    }
    // ctrl.speed_limit() ; // 不再需要，已在Search_Target中處理
    ctrl.Information(counts);
    ctrl.local_target(counts); // 記錄局部目標
    ctrl.stop_at_max_step();    // 檢查是否達到最大步數

    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("Controller loop finished. Final counts: %d", counts);
  ctrl.Final_stop(counts); // 確保結束時機器人停止
  return 0;
}