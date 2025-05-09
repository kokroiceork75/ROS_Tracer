//////////////////////////////  include  ///////////////////////////////////////////
#include <sstream>
#include <iostream>
#include <algorithm>
#include "ros/ros.h"
#include <std_msgs/Float64.h> 
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <fstream>
#include <math.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

const double dwall = 0.3;
double vel_adj = 0.5;			
const double f_along = (dwall + 0.45 + 0.25); // front進沿牆，加多少根據經驗可修改
const double s_along = (dwall + 0.50 + 0.25); // side,加多少根據經驗可修改
double robot_margin = 0.285;
const double danger_threshold1 = (robot_margin + dwall), 
				danger_threshold2 = (robot_margin + dwall) / std::cos(20 * M_PI / 180.0), 
				danger_threshold3 = (robot_margin + dwall) / std::cos(45 * M_PI / 180.0);
#define load_data_clu "/home/user/wei_ws/src/controller/src/fuzzy_controller/along_wall_controller/10.txt"
#define load_data_FC "/home/user/wei_ws/src/controller/src/fuzzy_controller/along_wall_controller/temp_w45-1.txt"  // wei for max speed = 50.0 rad/s
#define save_ave_vel "/home/user/wei_ws/src/controller/src/position_random/save_ave_speed.txt"
#define save_ave_error "/home/user/wei_ws/src/controller/src/position_random/save_ave_dis_error.txt" 

#define MIN(a,b) (a<b?a:b)
#define _max_rule 30
#define _rule_number 10   /* 規則數 */ 
#define _in_varl 4
#define _out_varl 2  //輸出個數 
#define max_step 40000
#define _rule_delta 10
#define _input_scale 1.0   //20200416   
#define max_speed 50.0
// #define robot_margin 0.285
#define radius 0.03
int _in_clu;//FC rules
double in[_in_varl+1] ;
double out_y[_out_varl+1] ; //輸出項暫存變數//左右輪輸出
double robot_vel[max_step+1] ;
double  sick_1[max_step+1], 
		sick_2[max_step+1], 
		sick_3[max_step+1], 
		sick_4[max_step+1]; 

double  sick_all[max_step+1], 
		sick_wall[max_step+1];

float far=5;
//const float PI=3.14;
const float x=9;
const float dis= 0.3791; // two wheel distance
double max_angular = (1.5 + 1.5) / dis; 
double ave_speed=0.0;
float ave_distance_error=0;
const int _mem_length = 2*_in_varl+ _out_varl ;
long double fuzzy_real[_mem_length*_max_rule+1] ;
double min_m[_in_varl+1],max_m[_in_varl+1];//最小中心點 & 最大中心點
int ber=1 ; //代表第幾個解，此為一個替代的暫存變數
double deviation_whirl = 0.; //輪速差
int steps,status;
int counts = 0;
int angle_left_min, angle_right_min ;
double left_min;
double right_min;
double straight_min;
//double error_z;
double back_left, back_right;
float laser_temp[361];
float laser_temp_scan[897];
double read_1, read_2, read_3, read_4, read_5, read_6;

double amcl_orientation_x,amcl_orientation_y,amcl_orientation_z,amcl_position_x,amcl_position_y,amcl_position_z;
double amcl_x, amcl_y, amcl_z; // 機器人位置和朝向

double v1, v2;

double position_x1,position_y1,orientation_z1;
double goal_x, goal_y, goal_z; // final goal and final orientation

double roll_s,pitch_s,yaw_s;
int decision_left = 0, decision_right = 0;
int counts_left = 0, counts_right = 0;

using namespace std;
inline float phi(float x,float m,float v)
{ return(  exp( - (x-m)*(x-m)/(v*v) )); }

class C_Rule
{
	public :
		double  in_mu ;
		double  con[_out_varl+1] ;
		void    In_mem_fir(double *,int) ;
		friend  void Fir_str(double *, int,int) ;
		friend  void fuzzy(double *,int,int);

};
C_Rule  _Rule[_max_rule+1] ;
void C_Rule::In_mem_fir(double *in,int _rule) 
{
	int i ;
	in_mu =  1.;
	for(i=1;i<= _in_varl;i++)
	{
		if(in[i] < min_m[i] || in[i] > max_m[i])
		{
			in_mu = in_mu * 1.;
		}
		else
		{
			in_mu = in_mu * phi(in[i] , fuzzy_real[(_rule-1)*_mem_length+i*2-1] , fuzzy_real[(_rule-1)*_mem_length+i*2] ) ;
		}
	}
}

class controller
{
	public:
		controller()
		{
			x1_pub=n.advertise<std_msgs::Float64>("data_x1",3000);
			y1_pub=n.advertise<std_msgs::Float64>("data_y1",3000);
			sub_final_goal = n.subscribe("/move_base_simple/goal", 3000, &controller::Final_Goal,this);
			sub=n.subscribe("/scan",3000,&controller::callback,this);
	    	pub_data1=n.advertise<std_msgs::Float64>("data1",3000);
			pub_data2=n.advertise<std_msgs::Float64>("data2",3000);
			sub_amcl = n.subscribe("/move_base/feedback", 3000, &controller::amcl_Callback,this);	
    		pub=n.advertise<geometry_msgs::Twist>("/cmd_vel_2",3000); // cmd_vel_2
			//sub_orientation =n.subscribe("chatter1",3000, &controller::chatter1Callback,this);
			sub_info1 =n.subscribe("chatter1",3000, &controller::chatterCallback,this);
		}
		void Final_Goal(const geometry_msgs::PoseStamped::ConstPtr & F_goal) //全域目標的X Y Z 的數值
		{
			goal_x = F_goal->pose.position.x;
			goal_y = F_goal->pose.position.y;
			tf2::Quaternion s;
			tf2::convert(F_goal->pose.orientation,s);
			tf2::Matrix3x3(s).getRPY(roll_s,pitch_s,yaw_s);
			goal_z = yaw_s;
			if( goal_z <=-1.5707 && goal_z>=-3.14159 )
				goal_z= (1.5 * M_PI) + goal_z; 
			else
				goal_z = (goal_z -1.5707);  
		}

		double minimum(int i, int j, int &k)
		{  
			double laser_min = 100;
			for (k=i ;k<=j;k++)
			{
				if (laser_min > laser_temp[k])
				{ 
					laser_min = laser_temp[k];
				}
			}
			return laser_min;
		}
		void callback(const sensor_msgs::LaserScan::ConstPtr& scan)
		{
			int k;
			for(int i=1;i<=scan->ranges.size();i++)
			{
				laser_temp_scan[i]=scan->ranges[i]; ///202009 180 to 270
			}
			for (int i = 0; i <= scan->ranges.size(); ++i)
			{
				double angle_rad = (scan->angle_min + i * scan->angle_increment) + M_PI;
				int angle_deg = angle_rad*180/M_PI;
				// if (!std::isinf(laser_temp_scan[i]))
				// {
				// 	if (laser_temp_scan[i]!=0)
				// 	{
				// 		laser_temp[angle_deg] = laser_temp_scan[i];
				// 	}
				// }		
				if (std::isinf(laser_temp_scan[i]))
				{
					laser_temp[angle_deg] = 10.4999;
				}		
				else
				{
					// 過濾小於0.5公尺的盲區值，設為0.5005
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
			left_min = minimum(205, 269, k); // 270 seems zero
			right_min = minimum(90, 155, k);
			straight_min = minimum(155, 205, k);
		}
		void decision_(int j)
		{	
			double xx1,xx2;
			// wei need change, can modify to another distance
			// if (straight_min <= f_along || left_min <= s_along || right_min <= s_along)
			// {
			// 	if (left_min <= s_along){
			// 		decision_left = 1;
			// 		decision_right= 0;
			// 	}
			// 	else if (right_min <= s_along){
			// 		decision_left = 0;
			// 		decision_right= 1;
			// 	}
			//     else if (left_min < right_min && left_min < s_along){
			// 		decision_left = 1;
			// 		decision_right= 0;
			// 	}
			// 	else if (left_min > right_min && right_min < s_along){
			// 		decision_left = 0;
			// 		decision_right= 1;
			// 	}
			// }
			if (status == 1)
			{
				if(straight_min <= f_along || left_min <= s_along || right_min <= s_along)
				{
					if (left_min <= right_min){
						decision_left = 1;
						decision_right= 0;
					}
					else{
						decision_left = 0;
						decision_right= 1;
					}
					// else if (left_min < right_min /*&& left_min < s_along*/){
					// 	decision_left = 1;
					// 	decision_right= 0;
					// }
					// else if (left_min > right_min /*&& right_min < s_along*/){
					// 	decision_left = 0;
					// 	decision_right= 1;
					// }
				}
			}
			// 不需要判斷障礙物方位是因為sub就檢查過了，status＝2的同時左方也有障礙物
		    else if(status == 2)
			{	
			 	decision_right = 0;
				decision_left  = 1;
			
			}
			else if(status == 3)
			{	
			 	decision_right = 1;
				decision_left  = 0;
			
			}
		
			if(decision_left == 1)
			{	
				printf("mode: LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL\n");
				read_1 = laser_temp[192] ;
				read_2 = laser_temp[218] ;
				read_3 = laser_temp[232] ;
				read_4 = laser_temp[261] ;
				read_5 = laser_temp[269] ;
				read_6 = laser_temp[187] ;
				for(size_t i = 180; i < 205; i++)//25
				{	if(read_1>laser_temp[i])
					read_1=std::min(laser_temp[i],far);
				}
				for(size_t i = 205; i < 225; i++)//20
				{	if(read_2>laser_temp[i])
					read_2=std::min(laser_temp[i],far);
				}
				for(size_t i = 225; i < 250; i++)//25
				{	if(read_3>laser_temp[i])
					read_3=std::min(laser_temp[i],far);
				}
				for(size_t i = 250; i < 270; i++)//20
				{	if(read_4>laser_temp[i])
					read_4=std::min(laser_temp[i],far);
				}
				for(size_t i = 180; i < 270; i++)
				{	if(read_5>laser_temp[i])
					read_5=std::min(laser_temp[i],far);
				}			
				for(size_t i = 180; i < 270; i++)  // 270度好像會是0
				{	if(read_6>laser_temp[i])
					read_6=std::min(laser_temp[i],far);
				}
				printf("left_along_wall\n");
			}
			if(decision_right == 1)
			{
				read_1 = laser_temp[158] ;
				read_2 = laser_temp[137] ;
				read_3 = laser_temp[122] ;
				read_4 = laser_temp[105] ;
				read_5 = laser_temp[180] ;
				read_6 = laser_temp[65] ;
				for(size_t i = 155; i <180; i++)//25
				{	if(read_1>laser_temp[i])
					read_1=std::min(laser_temp[i],far);
				}
				for(size_t i = 135; i < 155; i++)//20
				{	if(read_2>laser_temp[i])
					read_2=std::min(laser_temp[i],far);
				}
				for(size_t i = 110; i < 135; i++)//25
				{	if(read_3>laser_temp[i])
					read_3=std::min(laser_temp[i],far);
				}
				for(size_t i = 90; i < 110; i++)//20
				{	if(read_4>laser_temp[i])
					read_4=std::min(laser_temp[i],far);
				}
				//printf("%d\n",4);
				for(size_t i = 90; i <=180; i++)
				{	if(read_5>laser_temp[i])
					read_5=std::min(laser_temp[i],far);
				}			
				//printf("%d\n",5);
				for(size_t i = 90; i <= 180; i++)//175 180
				{	if(read_6>laser_temp[i])
					read_6=std::min(laser_temp[i],far);
				}
				printf("right_along_wall\n");	
			}

		}
		void test_run(int j)
		{
			geometry_msgs::Twist msg;	
			if(decision_left == 1 ) 
			{
			// v1 = out_y[1]*0.098 /1.3; wei need change
				v1 = out_y[1] * radius * vel_adj;
				v2 = out_y[2] * radius * vel_adj;
			}
			if(decision_right == 1 )
			{
				v1 = out_y[2] * radius * vel_adj;
				v2 = out_y[1] * radius * vel_adj;
			}
			msg.linear.x = (v1+v2) / 2 ;
			if((v1-v2)/dis >= max_angular)  msg.angular.z = max_angular; 
			else if ((v1-v2)/dis <= -max_angular) msg.angular.z = -max_angular;
			else msg.angular.z = (v1-v2)/dis;
			pub.publish(msg);

			if(decision_left == 1)
				printf("Lefttttttttttttttttttt\n");
			else if(decision_right == 1)
				printf("Rightttttttttttttttttt\n");
            printf("speed=%lf \t %lf \n",out_y[1],out_y[2]);
			printf("linear=%lf\tangular=%lf\n", msg.linear.x, msg.angular.z);
			pub.publish(msg);		
			/*FILE *pfout;
			pfout=fopen(save_vel,"a");//"controller/20210701/vel.txt"
			if(pfout==NULL){
				printf("Fail to open file");
				exit (1);
			}
			fprintf(pfout,"%f\n",msg.linear.x);
			fclose(pfout);*/
			ave_speed = ave_speed + (v1+v2)*0.5;			
		}

		void chatterCallback(const std_msgs::Int32::ConstPtr& info1)
		{
        	status = info1->data;
   		}
		
		void publisher_info(int jj ) //印出訊息並傳遞數值給其他程式從pub1~pub10;
  		{        
			std_msgs::Float64 info1,info2;
			info1.data = (v1+v2)/2;  //車子與目標的角度差
			info2.data = (v1-v2)/dis;              //經過轉換後的機器人Z軸角度
			pub_data1.publish(info1); 
			pub_data2.publish(info2);
  		}

		void test_open()
		{
			for(int ssss=1; ssss<=_in_varl; ssss++)
			{
				min_m[ssss]=1.0;  //為了找最小值，所以初始的最小值令為1，即為"最大"的最小值。
				max_m[ssss]=0.0;  //為了找最大值，所以初始的最大值令為0，即為"最小"的最大值。		      	
			}
			
			FILE *fnoise1,*fnoise0;  
			printf("read file\n");		
			if((fnoise0 = fopen(load_data_clu,"r"))==NULL)
			{
				printf("Nothing1\n");
				exit(1);
			}
			fscanf(fnoise0,"%d", &_in_clu);
			fclose(fnoise0);
			
			if((fnoise1 = fopen(load_data_FC,"r"))==NULL)
			{
				printf("Nothing2\n");
				exit(1);
			}
			for(int i=1;i<=_mem_length*_in_clu;i++)
			{
				fscanf(fnoise1,"%Lf \n", &fuzzy_real[i]);
				//printf("check%Lf\t",fuzzy_real[i]);
			}  
			fclose(fnoise1);  

			for(int jj=1; jj<=_rule_delta; jj++)
			{
				for(int jjj=1; jjj<=_in_varl; jjj++)
				{   
					if(fuzzy_real[(jj-1)*_mem_length+jjj*2-1] < min_m[jjj])
					{   
						min_m[jjj]=fuzzy_real[(jj-1)*_mem_length+jjj*2-1];
					}
					if(fuzzy_real[(jj-1)*_mem_length+jjj*2-1] > max_m[jjj])
					{
						max_m[jjj]=fuzzy_real[(jj-1)*_mem_length+jjj*2-1];
					}
				}

			}
			
		}
		void get_sensor(int jj)
		{
			sick_1[jj] = read_1;
			sick_2[jj] = read_2; 
			sick_3[jj] = read_3;
			sick_4[jj] = read_4;	
			sick_all[jj] = read_5;
			sick_wall[jj] = read_6;
			if(jj>=5){
				// wei need change
				ave_distance_error = ave_distance_error + fabs(read_6-robot_margin - dwall);
				printf("distance_error == %f\n",fabs(read_6-robot_margin - dwall));
			}
			robot_vel[jj] =out_y[1]+out_y[2]; //功能:左輪+右輪的輪速
			deviation_whirl=out_y[1]-out_y[2];
		}
		void sick_limit(int jj) 
		{
			double _limit = 1.0;  //20200505  5.0 to 1.0
			if (sick_1[jj] >= _limit)
				sick_1[jj] = _limit ;
			if (sick_2[jj] >= _limit)
				sick_2[jj] = _limit ;
			if (sick_3[jj] >= _limit)
				sick_3[jj] = _limit ;
			if (sick_4[jj] >= _limit)
				sick_4[jj] = _limit ;	
			if (sick_all[jj] >= _limit)
				sick_all[jj] = _limit ;					
			if (sick_wall[jj] >= _limit)
				sick_wall[jj] = _limit ;
			printf(" ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ \n" ) ;
			printf("Sick 1 === %f\n" , 		sick_1[counts] );  
			printf("Sick 2 === %f\n" , 		sick_2[counts] );  
			printf("Sick 3 === %f\n" , 		sick_3[counts] );
			printf("Sick 4 === %f\n" , 		sick_4[counts] );
			printf("Sick wall === %f\n" ,sick_wall[counts] ) ;
			printf("left_min===%f\n",left_min);
			printf("right_min===%f\n",right_min);
			printf("stright_min=%f\n",straight_min);
			printf("laser[90] === %f\n" ,laser_temp[90] ) ;
			printf("laser[180] === %f\n" ,laser_temp[180] ) ;
			printf("laser[269] === %f\n" ,laser_temp[269] ) ;
			printf("laser[270] === %f\n" ,laser_temp[270] ) ;
			printf("laser[165] = %f\tlaser[125] = %f\tlaser[110] = %f\n", laser_temp[165], laser_temp[125], laser_temp[110]);
            printf("laser[195] = %f\tlaser[235] = %f\tlaser[250] = %f\n", laser_temp[195], laser_temp[235], laser_temp[250]);


            // printf("laser[210] = %f\tlaser[215] = %f\tlaser[220] = %f\n", laser_temp[210], laser_temp[215], laser_temp[220]);
            // printf("laser[225] = %f\tlaser[230] = %f\tlaser[235] = %f\n", laser_temp[225], laser_temp[230], laser_temp[235]);
            // printf("laser[240] = %f\tlaser[245] = %f\tlaser[250] = %f\n", laser_temp[240], laser_temp[245], laser_temp[250]);
           
		}
		void fuzzy_in(int jj)
		{   
		//感測器讀值最大為5m (參閱副程式: get_sensor ) ，因此乘上1.0代表把所有讀值壓縮到 0~1 之間。  
			in[1] = sick_1[jj]; 
			in[2] = sick_2[jj];  
			in[3] = sick_3[jj];  
			in[4] = sick_4[jj];		
		}
		void Fir_str(double *in, int _rule, int ber)     
		{
			//這裡的ber，等於main函式內的ber，又ber的值為popsize的值。
			//這裡的 _rule，等於設定變數的_rule_number的值。
			//這裡的in，代表輸入的變數的數量
			int j, k;
			for(k=1; k<=_rule; k++)
			{
				for (j=1; j<=_out_varl; j++)
				{
					_Rule[k].con[j] = fuzzy_real[k*_mem_length-_out_varl+j];
				} // 讀取後件部值
				_Rule[k].In_mem_fir(in,k) ; // 計算激發量
			}
		}
		void speed_limit()
		{
			if(out_y[1] >= max_speed)
				out_y[1] = max_speed;
			if(out_y[2] >= max_speed)
				out_y[2] = max_speed;
			if(out_y[1] <= -(max_speed))
				out_y[1] = -max_speed;
			if(out_y[2] <= -(max_speed))
				out_y[2] = -max_speed;
		}
////////////////////////////////////// speed limit    //////////////////////////////////////

///////////////////////////////////////  Defuzzifier  ///////////////////////////////////////
		void fuzzy(int _rule , int ber)  // 權重式平均法
		{
			int i , j;
			double den[_out_varl+1] , num[_out_varl+1] ;
			for (j=1; j<=_out_varl; j++)
			{
				den[j] = 0. ;
				num[j] = 0. ;
				
				for (i=1; i<=_rule; i++)
				{
					num[j] = num[j] + _Rule[i].in_mu * _Rule[i].con[j];
					den[j] = den[j] + _Rule[i].in_mu ;
				}
				if ( fabs(den[j]) < 1e-8 )  //20211215
					out_y[j] = 0 ;
				else
					out_y[j] = num[j]/den[j] ;
				
				//out_y[j] = out_y[j]*left_wheel_speed ;//如果控制器的後件部有正規化則要*最高速
			}
			speed_limit();
			printf("out_y1=%lf \t out_y2=%lf\n ",out_y[1], out_y[2]);
		}

		void stop()
		{
			if(counts == max_step)
			{
				geometry_msgs::Twist msg;
				msg.linear.x =0;
				msg.angular.z=0;
				pub.publish(msg);
			}
		}
		void save_speed()
		{
			FILE *pfout2;
			pfout2=fopen(save_ave_vel,"a");//"controller/20210701/ave_ave_speed.txt"
			if(pfout2==NULL){
				printf("Fail to open file");
				exit (1);
			}
			fprintf(pfout2,"%f\n",(ave_speed/(double)(counts-4)));
			fclose(pfout2);
		}
		void save_wall_dis(){
			FILE *pfout3;
			// printf("111111111111111111111\n");
			pfout3=fopen(save_ave_error,"a");//"controller/20210701/save_ave_dis_error.txt"
			if(pfout3==NULL){
				printf("Fail to open file");
				exit (1);
			}
			fprintf(pfout3,"%f\n",(ave_distance_error/(float)(counts-4)));
			fclose(pfout3);
		}
		// wei need change, the following function is record, but disactivate first
		// void save_wall_dis_others(){
		// 	FILE *pfout33;
		// 	printf("111111111111111111111\n");
		// 	pfout33=fopen(save_every_error,"a");//"controller/20210701/save_ave_dis_error.txt"
		// 	if(pfout33==NULL){
		// 		printf("Fail to open file");
		// 		exit (1);
		// 	}
		// 	fprintf(pfout33,"%lf\t%lf\t%lf\n",left_min, right_min, stright_min);
		// 	fclose(pfout33);
		// }
	    void amcl_Callback(const::move_base_msgs::MoveBaseActionFeedback::ConstPtr & amcl)
		{
      		amcl_x = amcl->feedback.base_position.pose.position.x;
      	    amcl_y = amcl->feedback.base_position.pose.position.y;
      		// amcl_position_z = amcl->feedback.base_position.pose.position.z;
			std_msgs::Float64 data_x1 ,data_y1;
			data_x1.data = amcl_x;
		    data_y1.data = amcl_y;
			cout<<"data_x1= "<<data_x1<<endl;
			cout<<"data_y1= "<<data_y1<<endl;
			x1_pub.publish(data_x1);
            y1_pub.publish(data_y1);
  		}	

	private: 
		ros::NodeHandle n;
		ros::Subscriber sub;
		ros::Subscriber sub_amcl;
		ros::Publisher pub;
		ros::Publisher pub_data1;
		ros::Publisher pub_data2;
		ros::Subscriber sub_orientation;
		ros::Subscriber sub_info1;
		ros::Subscriber sub_final_goal;
		ros::Publisher x1_pub;
		ros::Publisher y1_pub;
		ros::Publisher x2_pub;
		ros::Publisher y2_pub;

};

int main(int argc,char ** argv)
{
	ros::init(argc,argv,"controller");
    controller ctrl;
	//ros::Rate a(100);  
	while(counts < max_step && ros::ok())
	{
		counts++;
		printf("counts:%d\n",counts);
		ctrl.test_open(); //to get max and min
		ctrl.decision_(counts);
		ctrl.get_sensor(counts) ;
		ctrl.sick_limit(counts) ;//20200505
		ctrl.fuzzy_in(counts) ;					//輸入是雷測數值，只是調整fuzzy的input比例			
		ctrl.Fir_str(in , _in_clu , ber) ;	//讀取後件部計算機發量
		ctrl.fuzzy(_in_clu , ber);  		//解模糊產生out_y
		ctrl.publisher_info(counts);
		//ctrl.test_run();  //to do work
		if(counts<5){
			ctrl.stop();
			out_y[1]=0;
			out_y[2]=0; ///202010
		}
		else{
			ctrl.test_run(counts);
		}
		// wei need change
		// ctrl.save_wall_dis_others();
		ros::spinOnce();
		ros::Duration(0.01).sleep(); // original An's, training code is about 5.56 Hz per contorl

		// ros::Duration(0.02).sleep(); // 20ms，50 Hz
		// ros::Duration(0.005).sleep(); // 200 Hz
		// ros::Duration(0.05).sleep(); // 50ms，20 Hz	

		//sros::Duration(0.01).sleep(); //訊息傳送間隔0.01秒
		//	a.sleep();
	}
	ctrl.save_speed();
	ctrl.save_wall_dis();
	ctrl.stop();
    return 0;
}