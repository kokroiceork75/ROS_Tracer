//////////////////////////////  include  ///////////////////////////////////////////
#include <sstream>
#include <iostream>
#include <algorithm>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <fstream>
#include <math.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <cmath>
//////////////////////////////  include  ///////////////////////////////////////////


//////////////////////////////  define區  /////////////////////////////////////////
#define MIN(a,b) (a<b?a:b)
#define _max_rule 30
#define _rule_number 10   /* 規則數 */ 
int _in_clu;//FC rules
#define _in_varl  4
//#define _in_varl  4
#define _out_varl 2  //輸出個數 
//#define max_step 5000   /* 最大步數 */  
#define max_step 60000
#define _rule_delta 10
#define _input_scale 1.0   //20200416   
#define PI 3.14159    
#define r 0.05
#define tran r*2*PI/60
#define left_wheel_speed 4  //如果控制器的後件部有正規化則要*最高速
#define max_speed 15.6
//////////////////////////////  define區  /////////////////////////////////////////
// #define load_data_clu "controller/20211027/2/clu.txt"
// #define load_data_FC "controller/20211027/2/Final_new.txt"
// #define load_data_clu "controller/20220105/Ev1/clu_10.txt"
// #define load_data_FC "controller/20220105/Ev2/Final_new_19.txt"
//#define load_data_clu "controller/20220209/Ev2/_in_clu_19.txt"
//#define load_data_FC "controller/20220209/Ev2/Final_first_19.txt"
#define load_data_clu "controller/nsga_v1/10.txt"

//#define load_data_FC "controller/20230502/wall.txt"

#define load_data_FC "controller/nsga_v1/temp_w1.txt"
//#define load_data_FC "controller/20221118/test.txt"
// #define load_data_clu "controller/20210803/1/clu.txt"
// #define load_data_FC "controller/20210803/1/Final_first_pos.dat"
//#define save_wall "controller/20230502/wall.txt"
//#define save_vel "controller/20230502/vel.txt"
#define save_ave_vel "controller/nsga_v1/save_ave_speed.txt"
#define save_ave_error "controller/nsga_v1/save_ave_dis_error.txt" 
//#define save_data_path1  "save/along_wall.txt" 

//////////////////////////////  變數宣告區  ///////////////////////////////////////{
double in[_in_varl+1] ;

double out_y[_out_varl+1] ; //輸出項暫存變數//左右輪輸出
double robot_vel[max_step+1] ;
double  sick_1[max_step+1], 
		sick_2[max_step+1], 
		sick_3[max_step+1], 
		sick_4[max_step+1]; 

double  sick_all[max_step+1], 
		sick_wall[max_step+1] ;

float far=5;
//const float PI=3.14;
const float x=9;

const float dis=0.57;

double ave_speed=0.0;
float ave_distance_error=0;

const int _mem_length = 2*_in_varl+ _out_varl ;
long double fuzzy_real[_mem_length*_max_rule+1] ;
double min_m[_in_varl+1],max_m[_in_varl+1];//最小中心點 & 最大中心點
int ber=1 ; //代表第幾個解，此為一個替代的暫存變數
double deviation_whirl = 0.; //輪速差

int steps,status;


int count=0;
int angle_left_min,angle_right_min ;
double left_min;
double right_min;
double stright_min;
//double error_z;
double back_left,back_right;
float laser_temp[361];
float laser_temp_scan[897];
double read_1;
double read_2;
double read_3;
double read_4;
double read_5;
double read_6;
double amcl_orientation_x,amcl_orientation_y,amcl_orientation_z,amcl_position_x,amcl_position_y,amcl_position_z;
double v1;
double v2;
int decision_left =0,decision_right =0;
int count_left=0 ,count_right=0;

inline float phi(float x,float m,float v)
{ return(  exp( - (x-m)*(x-m)/(v*v) )) ; }

class C_Rule
{
public :

	double  in_mu ;
	double  con[_out_varl+1] ;
	void    In_mem_fir(double *,int) ;
	friend  void Fir_str(double *, int,int) ;
	friend  void fuzzy(double *,int,int);

} ;

C_Rule  _Rule[_max_rule+1] ;


void C_Rule::In_mem_fir(double *in,int _rule) 
{
  
	int i ;
	in_mu =  1. ;
	
		for(i=1;i<= _in_varl;i++)
		{
			if(in[i] < min_m[i] || in[i] > max_m[i])
			 {
			 	in_mu = in_mu * 1;
			 }
			 else
			 {
			 	in_mu = in_mu * phi(in[i] , fuzzy_real[(_rule-1)*_mem_length+i*2-1] , fuzzy_real[(_rule-1)*_mem_length+i*2] ) ;
			 }
			//in_mu = in_mu * phi(in[i] , fuzzy_real[(_rule-1)*_mem_length+i*2-1] , fuzzy_real[(_rule-1)*_mem_length+i*2] ) ;//20211215

		}
	//}
}




class controller{
	public:
////////////////////////////////////////////  Constructor  //////////////////////////////////////////////	
		controller(){

		    sub=n.subscribe("/scan",1000,&controller::callback,this);
			///sub=n.subscribe("/scan",3000,&controller::callback,this);
	    	pub_data1=n.advertise<std_msgs::Float64>("data1",3000);
			pub_data2=n.advertise<std_msgs::Float64>("data2",3000);
			sub_amcl = n.subscribe("/move_base/feedback", 3000, &controller::amcl_Callback,this);	
			pub=n.advertise<geometry_msgs::Twist>("/cmd_vel",3000);
			//sub_orientation =n.subscribe("chatter1",3000, &controller::chatter1Callback,this);
			sub_info1 =n.subscribe("chatter1",3000, &controller::chatterCallback,this);
		}
////////////////////////////////////////////  Constructor  //////////////////////////////////////////////	

///////////////////////////////  get distance value from different angles from sensor ///////////////////////
	double minimun(int i, int j , int &k  )
   {  
      double laser_min=100;
      for (k=i ;k<=j;k++){
         if (laser_temp[k]!=0){
            if (laser_min>laser_temp[k])
            { 
			  laser_min=laser_temp[k];
			  
			}
         }
         
      }
	  return laser_min;
      

   }
		void callback(const sensor_msgs::LaserScan::ConstPtr& scan){
			int k;
            for(int i=1;i<=scan->ranges.size();i++){//180
				laser_temp_scan[i]=scan->ranges[i]; ///202009 180 to 270
			}
            for (int i = 0; i <= scan->ranges.size(); ++i)
            {
                
                double angle_rad = (scan->angle_min + i * scan->angle_increment) + M_PI;
                int angle_deg = angle_rad*180/M_PI;
                
                if (!std::isinf(laser_temp_scan[i])){
					if (laser_temp_scan[i]!=0){
						laser_temp[angle_deg] = laser_temp_scan[i];
					}
				}
                
            }


			/*for(int i=1;i<=360;i++){//180
				laser_temp[i]=scan->ranges[i]; ///202009 180 to 270
			}	*/
				left_min=minimun(210,315,k); //225~270
			
			
			    right_min=minimun(45,150,k); //80~135
			
			
				stright_min=minimun(150,210,k);
			
		}
		void decision_(int j){	
			
			if (stright_min<=0.5){
				/*if(status==2) {
					decision_left = 0;
					decision_right= 1;
				}
			    else if (status==3){
					decision_right = 0;
					decision_left  = 1;

				}*/
			    if (left_min<right_min){
					decision_left = 1;
					decision_right= 0;
				}
				else if (left_min>right_min){
					decision_left = 0;
					decision_right= 1;
				}
				
			}
		
			 if (status==1){
				
				 if ( left_min <=0.5 ) 
				{
					decision_left = 1;
					decision_right = 0;
				}
				else if( right_min <= 0.5){
					decision_left = 0;
					decision_right = 1;
				}
			}

		    else if(right_min < 0.5 && laser_temp[270] > 0.5 )// && laser_temp[90] < 0.7)// && decision_left == 0  )
			{	
			 	decision_right = 1;
				decision_left  = 0;
			
			}
			else if(left_min <0.5 && laser_temp[90]> 0.5)// && laser_temp[270] < 0.7 )//&& decision_right == 0 )
		    {
				decision_left = 1;
				decision_right= 0;
				
			}
		/*	else if (laser_temp[90]<0.5 && laser_temp[270]< 0.5 )
			{
			    decision_left = 1;
				decision_right= 0;
				
			}
			else if (laser_temp[90]<0.5 && laser_temp[270]< 0.5 )
			{
				decision_right = 1;
				decision_left  = 0;
			
			}
*/
			if(decision_left == 1 ){	
			read_1 = laser_temp[192] ;
			read_2 = laser_temp[218] ;
			read_3 = laser_temp[232] ;
			read_4 = laser_temp[261] ;
			read_5 = laser_temp[270] ;
			read_6 = laser_temp[180] ;

			for(size_t i = 180; i < 200; i++)//20
			{	if(read_1>laser_temp[i])
				read_1=std::min(laser_temp[i],far);
			}
			//printf("%d\n",1);

			for(size_t i = 200; i < 225; i++)//25
			{	if(read_2>laser_temp[i])
				read_2=std::min(laser_temp[i],far);
			}
			//printf("%d\n",2);
			for(size_t i = 225; i < 245; i++)//20
			{	if(read_3>laser_temp[i])
				read_3=std::min(laser_temp[i],far);
			}
			//printf("%d\n",3);
			for(size_t i = 245; i < 270; i++)//25
			{	if(read_4>laser_temp[i])
				read_4=std::min(laser_temp[i],far);
			}
			//printf("%d\n",4);
			for(size_t i = 90; i < 270; i++)
			{	if(read_5>laser_temp[i])
				read_5=std::min(laser_temp[i],far);
			}			
			//printf("%d\n",5);
			for(size_t i = 90; i <= 180; i++)//175 180
			{	if(read_6>laser_temp[i])
				read_6=std::min(laser_temp[i],far);
			}
			printf("left_along_wall");
			}
			if(decision_right == 1){
			read_1 = laser_temp[158] ;
			read_2 = laser_temp[137] ;
			read_3 = laser_temp[122] ;
			read_4 = laser_temp[105] ;
			read_5 = laser_temp[180] ;
			read_6 = laser_temp[1] ;

			for(size_t i = 160; i <=179; i++)//20
			{	if(read_1>laser_temp[i])
				read_1=std::min(laser_temp[i],far);
			}
			//printf("%d\n",1);

			for(size_t i = 135; i < 160; i++)//25
			{	if(read_2>laser_temp[i])
				read_2=std::min(laser_temp[i],far);
			}
			//printf("%d\n",2);
			for(size_t i = 115; i < 135; i++)//20
			{	if(read_3>laser_temp[i])
				read_3=std::min(laser_temp[i],far);
			}
			//printf("%d\n",3);
			for(size_t i = 90; i < 115; i++)//25
			{	if(read_4>laser_temp[i])
				read_4=std::min(laser_temp[i],far);
			}
			//printf("%d\n",4);
			for(size_t i = 90; i < 270; i++)
			{	if(read_5>laser_temp[i])
				read_5=std::min(laser_temp[i],far);
			}			
			//printf("%d\n",5);
			for(size_t i = 90; i <= 180; i++)//175 180
			{	if(read_6>laser_temp[i])
				read_6=std::min(laser_temp[i],far);
			}
			 printf("right_along_wall");	
		}
            if(decision_right == 0){
			read_1 = laser_temp[158] ;
			read_2 = laser_temp[137] ;
			read_3 = laser_temp[122] ;
			read_4 = laser_temp[105] ;
			read_5 = laser_temp[180] ;
			read_6 = laser_temp[1] ;

			for(size_t i = 160; i <=179; i++)//20
			{	if(read_1>laser_temp[i])
				read_1=std::min(laser_temp[i],far);
			}
			//printf("%d\n",1);

			for(size_t i = 135; i < 160; i++)//25
			{	if(read_2>laser_temp[i])
				read_2=std::min(laser_temp[i],far);
			}
			//printf("%d\n",2);
			for(size_t i = 115; i < 135; i++)//20
			{	if(read_3>laser_temp[i])
				read_3=std::min(laser_temp[i],far);
			}
			//printf("%d\n",3);
			for(size_t i = 90; i < 115; i++)//25
			{	if(read_4>laser_temp[i])
				read_4=std::min(laser_temp[i],far);
			}
			//printf("%d\n",4);
			for(size_t i = 90; i < 270; i++)
			{	if(read_5>laser_temp[i])
				read_5=std::min(laser_temp[i],far);
			}			
			//printf("%d\n",5);
			for(size_t i = 90; i <= 180; i++)//175 180
			{	if(read_6>laser_temp[i])
				read_6=std::min(laser_temp[i],far);
			}
			 printf("hahahahahahaha");	
		}
		}
				
///////////////////////////////  get distance value from different angles from sensor  ///////////////////////

		
	

////////////////////////////////////  Output cmd_velocity  ////////////////////////////////////
		void test_run(int j){
			geometry_msgs::Twist msg;				
			// v1=tran*out_y[2];
			// v2=tran*out_y[1];
			// printf(" LeftWheel_Speed == %f\n",x*v2);
			// printf(" RightWheel_Speed == %f\n",x*v1);//左右輪輸出
			// msg.linear.x =x*(v1+v2)/2;
			// msg.angular.z=x*(v1-v2)/dis;
			if(decision_left == 1 ){
			v1 = out_y[1]*0.03*5;
			v2 = out_y[2]*0.03*5;
			}
			if(decision_right == 1 ){
			 v1 = out_y[2]*0.03*5;
			 v2 = out_y[1]*0.03*5;
			}
			printf("LeftWheel_Speed == %f\n",v2);
			printf("RightWheel_Speed == %f\n",v1);
			msg.linear.x =((v1+v2)/2);
			msg.angular.z=((v1-v2)/dis);
     
			//if (((v1+v2)/2)==0 && (v1-v2)/dis==0 && decision_left==1) msg.angular.z = -0.2;
			//else if (((v1+v2)/2)==0 && (v1-v2)/dis==0 && decision_right==1) msg.angular.z = 0.2;
			printf("left=%d\t right=%d\n",decision_left,decision_right);
             printf("speed=%lf \t %lf \n",out_y[1],out_y[2]);
			printf("linear=%lf\n",((v1+v2)/2));
			printf("angular=%lf\n",((v1-v2)/dis));
			pub.publish(msg);
////////////////////////////////////  Output cmd_velocity  ////////////////////////////////////

//////////////////////////////////// Record the average speed /////////////////////////////////				
			/*FILE *pfout;
			pfout=fopen(save_vel,"a");//"controller/20210701/vel.txt"
			if(pfout==NULL){
				printf("Fail to open file");
				exit (1);
			}
			fprintf(pfout,"%f\n",msg.linear.x);
			fclose(pfout);*/
//////////////////////////////////// Record the average speed /////////////////////////////////	
			ave_speed = ave_speed + (v1+v2)*0.5;			
		}
///////////////////////////////////  Open the file from leaning ///////////////////////////////
		void chatterCallback(const std_msgs::Int32::ConstPtr& info1){
         status =  info1->data;
   		}
		
		void publisher_info(int jj ) //印出訊息並傳遞數值給其他程式從pub1~pub10;
  		{        
    	std_msgs::Float64 info1,info2;
    	info1.data = (v1+v2)/2;  //車子與目標的角度差
    	info2.data = (v1-v2)/dis;              //經過轉換後的機器人Z軸角度
  
    	pub_data1.publish(info1); 
    	pub_data2.publish(info2);
  		}

		void test_open(){

			for(int ssss=1; ssss<=_in_varl; ssss++)
			{
					min_m[ssss]=1.0;  //為了找最小值，所以初始的最小值令為1，即為"最大"的最小值。
					max_m[ssss]=0.0;  //為了找最大值，所以初始的最大值令為0，即為"最小"的最大值。		
			////-1~1之間的中心點範圍 
			//max_m[ber][ssss]=0.0;  //為了找最大值，所以初始的最大值令為-1，即為"最小"的最大值。        
					
			}
			 
			FILE *fnoise1,*fnoise0;  
					

			printf("read file\n");		
			//if((fnoise0=fopen("controller/20210701/clu_first.txt","r"))==NULL)
			if((fnoise0=fopen(load_data_clu,"r"))==NULL)
			{
				printf("Nothing1\n");
				exit(1);
			}
			fscanf(fnoise0,"%d", &_in_clu);
			fclose(fnoise0);
			//if((fnoise1=fopen("controller/20210701/Final_first.txt","r"))==NULL)
			if((fnoise1=fopen(load_data_FC,"r"))==NULL)
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
///////////////////////////////////  Open the file from leaning ///////////////////////////////

///////////////////////////////////  Save sensor data to another vector & print out  /////////////////////////
		void get_sensor(int jj)
		{
			sick_1[count] = read_1;
			sick_2[count] = read_2; 
			sick_3[count] = read_3;
			sick_4[count] = read_4;	
			sick_all[count] = read_5;
			sick_wall[count] = read_6;

			printf(" ~~~~~~~~~~~~~~~~~~~~~~~~~~ \n" ) ;
			
			//printf("Sick 1 === %f\n" , 		sick_1[count] );  
			//printf("Sick 2 === %f\n" , 		sick_2[count] );  
			//printf("Sick 3 === %f\n" , 		sick_3[count] );
			//printf("Sick 4 === %f\n" , 		sick_4[count] );
			//printf("Sick wall === %f\n" ,sick_wall[count] ) ;
			//printf("left_min===%f\n",left_min);
			//printf("right_min===%f\n",right_min);
			printf("status=%d\n",status);
			printf("laser[90]=%f\n",laser_temp[90]);
            printf("laser[180]=%f\n",laser_temp[180]);
            printf("laser[269]=%f\n",laser_temp[269]);
			printf("laser[1]=%f\n",laser_temp[1]);
			printf("right_laser=%f\n",right_min);
			printf("left_laser=%f\n",left_min);
			printf("stright_laser=%f\n",stright_min);
			//printf("decision_right ==%d  \t count_right==%d\n",decision_right,count_right);
			//printf("decision_left == %d  \t count_left ==%d\n",decision_left,count_left);
			//printf("min_laser[%d]=%f \n",right_min);
			//printf("p[%d]==%d\n",count,p[count]);
			////////////////////////////////////  Save the distance from wall  ///////////////////////////
			if(jj>=5){
			/*FILE *pfout1;
				pfout1=fopen(save_wall,"a");//"controller/20210701/wall.txt"
				if(pfout1==NULL){
					printf("Fail to open file");
					exit (1);
				}
				fprintf(pfout1,"%f\n",123.2);
			fclose(pfout1);*/
////////////////////////////////////  Save the distance from wall  ///////////////////////////
			ave_distance_error = ave_distance_error + fabs(read_6-0.535);
			printf("distance_error == %f\n",fabs(read_6-0.535));
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
		}
///////////////////////////////////  Save sensor data to another vector & print out  ///////////////////////////


//////////////////////////////////  Rescale sensor data for fuzzy  /////////////////////////////////////////
		void fuzzy_in(int jj)
		{   
		//感測器讀值最大為5m (參閱副程式: get_sensor ) ，因此乘上1.0代表把所有讀值壓縮到 0~1 之間。  

			in[1] = sick_1[jj] * _input_scale ; 
			in[2] = sick_2[jj] * _input_scale ;  
			in[3] = sick_3[jj] * _input_scale ;  
			in[4] = sick_4[jj] * _input_scale ;
			//in[5] =  (deviation_whirl+15.6) * (1/31.2);//平移再normalize//2017_8_2
			//左右輪最高速為7.8			
		}
//////////////////////////////////  Rescale sensor data for fuzzy  /////////////////////////////////////////



///////////////////////////////// Calculate firing strengh  ////////////////////////////////////////////
		void Fir_str(double *in, int _rule, int ber)     
		{
			//這裡的ber，等於main函式內的ber，又ber的值為popsize的值。
			//這裡的 _rule，等於設定變數的_rule_number的值。
			//這裡的in，代表輸入的變數的數量

			int j,k ;

			for(k=1; k<=_rule; k++)
			{
				for (j=1; j<=_out_varl; j++)
				{
					_Rule[k].con[j] = fuzzy_real[k*_mem_length-_out_varl+j];
				} // 讀取後件部值
				_Rule[k].In_mem_fir(in,k) ; // 計算激發量
				
			}
		
		}
///////////////////////////////// Calculate firing strengh  ////////////////////////////////////////////

////////////////////////////////////// speed limit    //////////////////////////////////////
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
					//if ( fuzzy_xrule[ber][i] == _true )
				//	{
						num[j] = num[j] + _Rule[i].in_mu * _Rule[i].con[j];
						den[j] = den[j] + _Rule[i].in_mu ;
				//	}
				}


				if ( fabs(den[j]) < 1e-8 )  //20211215
					out_y[j] = 0 ;
				else
					out_y[j] = num[j]/den[j] ;
				
				//out_y[j] = out_y[j]*left_wheel_speed ;//如果控制器的後件部有正規化則要*最高速
			}
			speed_limit();

		}
///////////////////////////////////////  Defuzzifier  ///////////////////////////////////////u


//////////////////////////////////  Stop the car  ///////////////////////////////////////
		void stop(){
			if(count==max_step)
				{

					geometry_msgs::Twist msg;

					msg.linear.x =0;
					msg.angular.z=0;

					pub.publish(msg);
					
					//exit(1);
				}
		}
//////////////////////////////////  Stop the car  ///////////////////////////////////////
		void save_speed(){
			FILE *pfout2;
			pfout2=fopen(save_ave_vel,"a");//"controller/20210701/ave_ave_speed.txt"
			if(pfout2==NULL){
				printf("Fail to open file");
				exit (1);
			}
			fprintf(pfout2,"%f\n",(ave_speed/(double)(count-4)));
			fclose(pfout2);
		}
		void save_wall_dis(){
			FILE *pfout3;
			printf("111111111111111111111\n");
			pfout3=fopen(save_ave_error,"a");//"controller/20210701/save_ave_dis_error.txt"
			if(pfout3==NULL){
				printf("Fail to open file");
				exit (1);
			}
			fprintf(pfout3,"%f\n",(ave_distance_error/(float)(count-4)));
			fclose(pfout3);
		}
	    void amcl_Callback(const::move_base_msgs::MoveBaseActionFeedback::ConstPtr & amcl){
      		amcl_position_x = amcl->feedback.base_position.pose.position.x;
      	    amcl_position_y = amcl->feedback.base_position.pose.position.y;
      		amcl_position_z = amcl->feedback.base_position.pose.position.z;
      		amcl_orientation_x = amcl->feedback.base_position.pose.orientation.x;
     		amcl_orientation_y = amcl->feedback.base_position.pose.orientation.y;
      		amcl_orientation_z = amcl->feedback.base_position.pose.orientation.z; 
      		/*FILE *pfout5;
      		printf("\n\nread file\n");
      		pfout5=fopen(save_data_path1,"a");
			if(pfout5==NULL){
				printf("Fail to open file");
				exit(1);
			}
         fprintf(pfout5,"%lf \t %lf\n",amcl_position_x,amcl_position_y);
         fclose(pfout5);*/
  		 }	

	private: 

		ros::NodeHandle n;
		
		ros::Subscriber sub;
		
		ros::Subscriber sub_amcl;
		
		ros::Publisher pub;
		ros::Publisher pub_data1;
		ros::Publisher pub_data2;
		ros::Subscriber sub_orientation;
		ros::Subscriber sub_info1  ;

};
//controller  _Rule[_max_rule+1] ;

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"controller");
	
    controller ctrl;
	//ros::Rate a(100);  
	
	while(count<max_step && ros::ok())
	{
		
		count++;
		
		printf("count:%d",count);
		//ros::spinOnce();
		
		ctrl.test_open(); //to get max and min
		ctrl.decision_(count);
		ctrl.get_sensor(count) ;
		ctrl.sick_limit(count) ;//20200505
		ctrl.fuzzy_in(count) ;					//輸入是雷測數值，只是調整fuzzy的input比例			
		ctrl.Fir_str(in , _in_clu , ber) ;	//讀取後件部計算機發量
		ctrl.fuzzy(_in_clu , ber);  		//解模糊產生out_y
		ctrl.publisher_info(count);
		//ctrl.test_run();  //to do work
		if(count<5){
			ctrl.stop();
			out_y[1]=0;
			out_y[2]=0; ///202010
		}
		else{
			ctrl.test_run(count);
		}
		ros::spinOnce();
	
	  ros::Duration(0.01).sleep();
		
	}
	ctrl.save_speed();
	ctrl.save_wall_dis();

	ctrl.stop();
	

    return 0;
}