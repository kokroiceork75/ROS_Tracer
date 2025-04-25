#include <ctime> 
#include <math.h>
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

using namespace std;

#define MIN(a,b) (a<b?a:b)
#define save_data_path1    "/home/user/wei_ws/src/controller/src/position_random/real-map2s/position_all_fuzzy_pid_NEW2.txt"
#define save_along_wall    "/home/user/wei_ws/src/controller/src/position_random/real-map2s/position_along_wall_fuzzy_pid_NEW2.txt"
#define save_search_target "/home/user/wei_ws/src/controller/src/position_random/real-map2s/posititon_search_target_fuzzy_pid_NEW2.txt"
#define save_distance      "/home/user/wei_ws/src/controller/src/position_random/real-map2s/distance-NEW.txt"
#define save_velocity      "/home/user/wei_ws/src/controller/src/position_random/real-map2s/velocity-NEW.txt"

#define max_step 40000

double fff=0;
int p[max_step+1]={0}, counts=0, last[max_step], ss=111;
int HH;
int height,width;
double v_S,v_A;
double error_orientation_z;
double target_orientation_z, target_position_x, target_position_y;
double amcl_orientation_x, amcl_orientation_y, amcl_orientation_z;
double amcl_position_z, amcl_position_y, amcl_position_x;
double amcl_x[max_step], amcl_y[max_step];
double left_min, right_min, straight_min;
double decision_left, decision_right; 
double laser_temp[361];
float laser_temp_scan[897];
double vel_s, vel_a, angular_s, angular_a;
double rms, rms_x, rms_y;
int status=1000;
double integral,turn;
double roll, pitch, yaw;
double margin_laser[361];
double position_x[max_step], position_y[max_step];
double error_x, error_y, directions;
int ik;
double x_, y_;
geometry_msgs::Twist msg;

bool recieved_message = false;

class cmd_sub_pub {
private:
   ros::NodeHandle n;
   ros::Publisher pub;
   ros::Subscriber sub_1;
   ros::Subscriber sub_2;
   ros::Subscriber sub_goal;
   ros::Subscriber sub_laser;
   ros::Subscriber sub_amcl;
   ros::Subscriber sub_info1;
   ros::Subscriber sub_info2;
   ros::Subscriber sub_path;
   ros::Subscriber sub_cmdvel;
   ros::Subscriber sub_odom0;
   ros::Subscriber sub_decision;
   ros::Publisher x1_pub;
   ros::Publisher y1_pub;
   ros::Publisher x2_pub;
   ros::Publisher y2_pub;
   ros::Publisher pub_1;
   ros::Publisher pub_2;

public:
   cmd_sub_pub()
   {
      pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",3000);
      x1_pub = n.advertise<std_msgs::Float64>("/data_x1",3000);
      y1_pub = n.advertise<std_msgs::Float64>("/data_y1",3000);
      x2_pub = n.advertise<std_msgs::Float64>("/data_x2",3000);
      y2_pub = n.advertise<std_msgs::Float64>("/data_y2",3000);
      pub_1 = n.advertise<std_msgs::Float64>("/info_1",3000);
      pub_2 = n.advertise<std_msgs::Int32>("/info_2",3000);
      
      sub_laser = n.subscribe("/scan",3000, &cmd_sub_pub::callback, this);
      sub_1 = n.subscribe("/cmd_vel_1",3000, &cmd_sub_pub::speed_Search, this);
      sub_2 = n.subscribe("/cmd_vel_2",3000, &cmd_sub_pub::speed_Along_wall, this);
      sub_goal = n.subscribe("/move_base_simple/goal",3000, &cmd_sub_pub::Search_Target, this);
      sub_info1 = n.subscribe("/chatter1",3000, &cmd_sub_pub::chatter1Callback, this);
      sub_info2 = n.subscribe("/chatter2",3000, &cmd_sub_pub::chatter2Callback, this);
      sub_amcl = n.subscribe("/move_base/feedback", 3000, &cmd_sub_pub::amcl_Callback, this);
   }
   
   // 清除動作，但不關閉整個Node
   void stopMotion()
   {
      msg.linear.x = 0;
      msg.angular.z = 0;
      pub.publish(msg);
      ROS_INFO("Robot arrived and stopped. Waiting for next goal...");
   }

   void callback(const sensor_msgs::LaserScan::ConstPtr& scan)
   {
      int k;
      for(int i=1; i <= (int)scan->ranges.size(); i++){
         laser_temp_scan[i] = scan->ranges[i];
      }
      for(int i = 0; i <= (int)scan->ranges.size(); i++){
         double angle_rad = (scan->angle_min + i * scan->angle_increment) + M_PI;
         int angle_deg = (int)(angle_rad * 180.0 / M_PI);
         if (!std::isinf(laser_temp_scan[i]) && laser_temp_scan[i]!=0){
            laser_temp[angle_deg] = laser_temp_scan[i];
         }
      }
      left_min = minimun(195,269,k);
      right_min= minimun(90,165,k);
      straight_min = minimun(135,225,k);
   }
   
   double minimun(int i, int j, int &k)
   {
      double laser_min=100;
      for(k=i; k<j; k++){
         if(laser_min > laser_temp[k]){
            laser_min = laser_temp[k];
         }
      }
      return laser_min;
   }

   void speed_Search(const geometry_msgs::Twist::ConstPtr& vel1)
   {
      vel_s = vel1->linear.x;
      angular_s = vel1->angular.z;
      recieved_message = true; 
   }

   void speed_Along_wall(const geometry_msgs::Twist::ConstPtr& vel2)
   {
      vel_a = vel2->linear.x;
      angular_a = vel2->angular.z;
   }

   void Search_Target(const geometry_msgs::PoseStamped::ConstPtr& goal)
   {
      target_position_x = goal->pose.position.x;
      target_position_y = goal->pose.position.y;
      tf2::Quaternion s;
      tf2::convert(goal->pose.orientation, s);
      tf2::Matrix3x3(s).getRPY(roll, pitch, yaw);
      target_orientation_z = yaw;
   }

   void amcl_Callback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr & amcl)
   {
      amcl_position_x = amcl->feedback.base_position.pose.position.x;
      amcl_position_y = amcl->feedback.base_position.pose.position.y;

      amcl_orientation_x = amcl->feedback.base_position.pose.orientation.x;
      amcl_orientation_y = amcl->feedback.base_position.pose.orientation.y;
      amcl_orientation_z = amcl->feedback.base_position.pose.orientation.z;

      // 若有路徑資料，可在此比對 position_x[i], position_y[i]
      for(int i=1; i <= 600; i++){
         double dist = sqrt(pow(position_x[i]-amcl_position_x,2)+pow(position_y[i]-amcl_position_y,2));
         if(dist>0.1 && dist<=0.15){
            x_ = position_x[i];
            y_ = position_y[i];
         }
      }
   }

   void chatter1Callback(const std_msgs::Int32::ConstPtr& info1){
      status = info1->data;
   }
   void chatter2Callback(const std_msgs::Float64::ConstPtr& info2){
      error_orientation_z = info2->data;
   }

   void decision_path(double i, double j, double k, int jj)
   {
      // ... (此處省略大部分原程式碼, 你可保留或依需調整) ...
      // 判斷機器人應該進入「沿牆模式」(p[jj] = 2) 或「尋標模式」(p[jj] = 1)...

      // 這裡是原程式中的關鍵部分: 當 p[jj] == 1 時執行「尋標模式」，p[jj] == 2 時執行「沿牆模式」
      int js=315;
      margin_laser[35] = 0.49207; // wei change Tracer20250109
      margin_laser[40] = 0.44073; 
      margin_laser[45] = 0.40190; 
      margin_laser[50] = 0.37247; 
      margin_laser[310] = 0.37247; 
      margin_laser[315] = 0.40190; 
      margin_laser[320] = 0.44073; 
      margin_laser[325] = 0.49207; 

      margin_laser[90] = 0.29010;
      margin_laser[95] = 0.29170;
      margin_laser[100] = 0.29562;
      margin_laser[105] = 0.30188;
      margin_laser[110] = 0.31096;
      margin_laser[115] = 0.31017;
      margin_laser[120] = 0.26386;
      margin_laser[125] = 0.23164;
      margin_laser[130] = 0.20743;
      margin_laser[135] = 0.18912;
      margin_laser[140] = 0.17525;
      margin_laser[145] = 0.16425;
      margin_laser[150] = 0.15581;
      margin_laser[155] = 0.14915;
      margin_laser[160] = 0.14418;
      margin_laser[165] = 0.14049;
      margin_laser[170] = 0.13806;
      margin_laser[175] = 0.13670;
      margin_laser[180] = 0.13641;
      margin_laser[185] = 0.13715;
      margin_laser[190] = 0.13898;
      margin_laser[195] = 0.14192;
      margin_laser[200] = 0.14617;
      margin_laser[205] = 0.15179;
      margin_laser[210] = 0.15924;
      margin_laser[215] = 0.16863;
      margin_laser[220] = 0.18088;
      margin_laser[225] = 0.19669;
      margin_laser[230] = 0.21694;
      margin_laser[235] = 0.24445;
      margin_laser[240] = 0.28163;
      margin_laser[245] = 0.31727;
      margin_laser[250] = 0.30667;
      margin_laser[255] = 0.29880;
      margin_laser[260] = 0.29361;
      margin_laser[265] = 0.29069;
      margin_laser[269] = 0.29005;
      margin_laser[270] = 0.29005;
      p[jj]=0;
      int ln=0;
      amcl_x[jj]=amcl_position_x;
      amcl_y[jj]=amcl_position_y;
      rms_x=amcl_x[jj]-target_position_x;
      rms_y=amcl_y[jj]-target_position_y;
      rms= sqrt(pow(rms_x,2)+pow(rms_y,2));
      for(int lsr=90;lsr<=180;lsr=lsr+5){
         // if(laser_temp[lsr] < margin_laser[lsr]  && laser_temp[lsr] >= 0.15){  wei change 
         if(laser_temp[lsr] <= margin_laser[lsr] + 0.35)
         {
            p[jj] =2; ////
            ln=1; ////離牆太近
            printf("almost HITTTTTTTTTTTTTTTTTTTTT\n");
            printf("限制：觸碰最小距離: %lf\t 角度：%d\n",margin_laser[lsr],lsr);
            printf("右側車子最小距離：%lf\n",right_min); 
         }
      }
      for(int lsr=180;lsr<=270;lsr=lsr+5){
         // if( laser_temp[lsr]< margin_laser[lsr] && laser_temp[lsr] >= 0.15){ wei change
         if( laser_temp[lsr]<= margin_laser[lsr] + 0.35){
            p[jj] =2; ////
            ln=1; ////離牆太近
            printf("almost HITTTTTTTTTTTTTTTTTTTTT\n");
            printf("限制：觸碰最小距離: %lf\t 角度：%d\n",margin_laser[lsr],lsr);
            printf("左側車子最小距離：%lf\n",left_min);
         }
      }
      for(int lsr=165;lsr<=195;lsr=lsr+5){
         // if(laser_temp[lsr]<margin_laser[lsr] && laser_temp[lsr] >= 0.15){  wei change
         if(laser_temp[lsr] <= margin_laser[lsr] + 0.35){
            p[jj] =2; ////尋標
            ln=1; ////離牆太近
            printf("HITTTTTTTTTTTTTTTTTTTTT\n");
            printf("限制：觸碰最小距離: %lf\t 角度：%d\n",margin_laser[lsr],lsr);
            printf("前方車子最小距離：%lf\n",straight_min); 
         }
      }
      

      
      if (status==1 && (left_min<0.585 || right_min<0.585)) 
      { ///朝前方移動時候,考慮兩側是否有障礙物  wei change from 0.5 to 0.585 
         if( left_min<=0.585 )
         {
            printf("直行區+左沿牆 \t ststus = %d\n",status);
            cout<<"11111111111111111111"<<endl;
         }
         else if ( right_min<=0.585 )
         {
            printf("直行區+右沿牆 \t ststus = %d\n",status);
            cout<<"22222222222222222222222"<<endl;
         }
         p[jj]=2;
      }
      else if(  (status ==2 && left_min<=0.585) ) 
      { ///朝前方移動時候,考慮兩側是否有障礙物
         printf("左前方有障礙,左沿牆 \t status = %d\n",status);
         p[jj]=2;
      }
      // 20250204 need change
      else if( (status ==3 && right_min<=0.585) ) 
      { ///朝前方移動時候,考慮兩側是否有障礙物
         printf("右前方有障礙物,右沿牆 \t status = %d\n",status);
         p[jj]=2;
      }
      else if(straight_min <= 0.48) // 0.45是可以調的
      {
         printf("正前方有障礙物 \t status = %d\n",status);
         p[jj]=2;
      }
      else
      { 
         printf("尋標模式！！！！\n");
         cout<<"staus= "<<status<<endl;
         p[jj]=1;
      } 

     if( (straight_min < 0.6 && straight_min > 0.3)  && p[jj]==1 && ln!=1 ) 
      {
         vel_s = vel_s*1;
         angular_s = angular_s*1;
         cout<<"========60％％％％speed========="<<endl;
      }

      if(p[jj] == 1){
         ss=1;
         HH=0;
         msg.linear.x = vel_s * 0.8;
         msg.angular.z= angular_s * 0.8;
         pub.publish(msg);
         ROS_INFO("Search Target: cmd_vel_1");
         publisher_info(jj);
      }
      else if(p[jj] == 2){
         ss=0;
         HH=1;
         msg.linear.x = vel_a * 0.8;
         msg.angular.z= angular_a * 0.8;
         pub.publish(msg);
         ROS_INFO("Along_Wall: cmd_vel_2");
      }
   }

   void publisher_info(int jj)
   {
      std_msgs::Float64 info_1;
      std_msgs::Int32   info_2;

      info_1.data = ss;
      info_2.data = HH;
      pub_1.publish(info_1);
      pub_2.publish(info_2);
   }

   void save_info(int jj){
      // 與原程式相同: 將路徑存檔、區分何時是沿牆/尋標
      if(amcl_x[jj] !=0. && amcl_y[jj]!=0.){
         FILE *pfout1 = fopen(save_data_path1,"a");
         if(!pfout1){
            printf("Fail to open file");
            exit(1);
         }
         fprintf(pfout1,"%lf \t %lf\n", amcl_x[jj], amcl_y[jj]);
         fclose(pfout1);
      }
      // 根據 p[jj] = 1 or 2，存不同路徑檔...
      // (保持原邏輯, 這裡略)
      
      if(jj>5){
         fff += sqrt(pow(amcl_x[jj]-amcl_x[jj-1],2)+pow(amcl_y[jj]-amcl_y[jj-1],2));
      }
   }

   // 讓外部可以呼叫，用來清空動作，但不要關閉程式
   void checkArrivedAndStop()
   {
      // 如果機器人跟目標距離很接近，就停止，但 Node 不結束
      double dist_to_goal = sqrt(pow(amcl_position_x - target_position_x, 2) +
                                 pow(amcl_position_y - target_position_y, 2));
      // 閾值可自行調整
      if(dist_to_goal < 0.2){
         stopMotion();
      }
   }
};

int main(int argc,char ** argv)
{
   ros::init(argc, argv, "cmd_sub_pub"); 
   cmd_sub_pub obj;

   while(counts < max_step && ros::ok())
   {
      if(target_position_x != 0){
         counts++;
      }
      ROS_INFO_STREAM("counts: " << counts);

      // 核心邏輯: 決定當前是沿牆或尋標
      obj.decision_path(right_min, left_min, straight_min, counts);
      // 發布一些資訊
      obj.publisher_info(counts);
      // 紀錄機器人路徑
      obj.save_info(counts);
      // 檢查是否到達目標，如果到了就停下，但不 shutdown
      obj.checkArrivedAndStop();

      ros::spinOnce();
      ros::Duration(0.01).sleep();
   }

   // 即使 counts >= max_step，程式要結束也可以，但如果你想讓它永遠跑，就把這個條件或存檔動作改掉
   FILE *pfout4 = fopen(save_distance,"a");
   if(!pfout4){
      printf("Fail to open file");
      exit(1);
   }
   fprintf(pfout4,"%lf \n", fff);
   fclose(pfout4);

   FILE *pfoutv = fopen(save_velocity,"a");
   if(!pfoutv){
      printf("Fail to open file");
      exit(1);
   }  
   fprintf(pfoutv,"%d \t %lf\t %lf\n", counts, (double)counts/100., fff / ((double)counts/100.));
   fclose(pfoutv);

   return 0;
}
