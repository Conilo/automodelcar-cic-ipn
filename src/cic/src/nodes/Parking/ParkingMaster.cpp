#include <ros/ros.h>
#include <signal.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>  
#include <sensor_msgs/image_encodings.h>
#include <ros/console.h>
#include <vector>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>


std_msgs::Int16 an90, an180, max_180, vel, angDir;
std_msgs::Float32 dist90, dist180, dist270, dist360,espacio,yaw_inic, yaw, yaws, yaw_rel;


 
bool fin= false; 
bool objBandera=false;
//int angDir;
//int vel= 0;
int espacioFlag=0;
float promedio;
float intCent = 0.0;
float x,l1,l2;


//global launch launch_lane WARNING 

class Master_parking
{
	ros::NodeHandle nh_;
	ros::Publisher pubDir,pubVel;
	ros::Subscriber scan_,obj_90_,obj_180_ ,obj_270_,obj_360_,obj_angle90_,obj_angle180_, max180_,yaw2_;
	public:
	  Master_parking() 
	  //: it_(nh_)  
	  {
			scan_ = nh_.subscribe("/scan_follower/scan", 1, &Master_parking::scan_callback, this);
			obj_90_ = nh_.subscribe("/scan_follower/obj_90", 1, &Master_parking::obj90_callback, this);
			obj_180_ = nh_.subscribe("/scan_follower/obj_180", 1, &Master_parking::obj180_callback, this);
			obj_270_ = nh_.subscribe("/scan_follower/obj_270",1,&Master_parking::obj270_callback, this);
			obj_360_ =nh_.subscribe("/scan_follower/obj_360",1, &Master_parking::obj360_callback, this);
			obj_angle90_= nh_.subscribe("/scan_follower/obj_angle90",1, &Master_parking::obj_angle90_callback, this);
			obj_angle180_ =nh_.subscribe("/scan_follower/obj_angle180",1, &Master_parking::obj_angle180_callback, this);
			max180_ =nh_.subscribe("/scan_follower/max180",1, &Master_parking::max180_callback, this);
			yaw2_ =nh_.subscribe("/scan_follower/yaw2",1, &Master_parking::yawF_callback, this);

			pubDir= nh_.advertise<std_msgs::Int16>("/manual_control/steering",1);
			pubVel= nh_.advertise<std_msgs::Int16>("/manual_control/speed",1);
	  }

	  int corr() //x ya esta definido 
	  {
	  	return int(x*40)+123;
	  }

	  void obj90_callback(const std_msgs::Float32 ros_dummie) 
	  {
			dist90.data = ros_dummie.data;
		}

		void obj180_callback(const std_msgs::Float32 ros_dummie) 
		{
			dist180.data = ros_dummie.data;
		}

		void obj270_callback(const std_msgs::Float32 ros_dummie) 
		{
			dist270.data = ros_dummie.data;
		}

		void obj360_callback(const std_msgs::Float32 ros_dummie) 
		{
			dist360.data = ros_dummie.data;
		}

		void obj_angle90_callback(const std_msgs::Int16 ros_dummie) 
		{
			an90.data = ros_dummie.data;
		}

		void obj_angle180_callback(const std_msgs::Int16 ros_dummie) 
		{
			an180.data = ros_dummie.data;
		}

		void max180_callback(const std_msgs::Int16 ros_dummie) 
		{
			max_180.data = ros_dummie.data;
		}

		void yawF_callback(const std_msgs::Float32 ros_dummie)
		{
			yaw.data=ros_dummie.data;
			if (yaw_inic.data<40)
			{
				yaws.data=yaw.data;
			}
			else
			{
				if (yaw.data<0)
				{
					yaws.data = yaw.data +360;
				}
				else
				{
					yaws.data= yaw.data;
				}
			}
		}

		void routine() //espacio ya esta definida como variable global
		{
			yaw_inic.data= yaws.data;
			yaw_rel.data=yaw_inic.data-30;

			while(yaws.data>=yaw_rel.data)
			{
				angDir.data=10;
				vel.data=400;
				pubVel.publish(vel);
				pubDir.publish(angDir);
			}
			ROS_INFO("Hola 1");
			angDir.data=100;
			pubDir.publish(angDir);

			while(an90.data>63)
			{
				angDir.data=100;
				vel.data=300;
				pubVel.publish(vel);
				pubDir.publish(angDir);
			}

			angDir.data=170;
			pubDir.publish(angDir);
			ROS_INFO("Hola 2");

			while (dist270.data >.33) 
			{
				angDir.data=170;
				vel.data=120;
				pubVel.publish(vel);
				pubDir.publish(angDir);
				if (yaws.data>yaw_inic.data)
				{
					ROS_INFO("YA1");
					break;
				}
			}

			while (yaws.data<(yaw_inic.data-3)) 
			{
				angDir.data=30;
				vel.data=-150;
				pubVel.publish(vel);
				pubDir.publish(angDir);
				if (dist360.data<0.25) 
				{
					ROS_INFO("YA");
					break;
				}
			}

			promedio=espacio.data/2.0; 
			ROS_INFO("%f",promedio);
			ROS_INFO("%f",dist270.data);

			while ((promedio-dist270.data)>0.07)
			{
				angDir.data=96;
				vel.data=-140;
				pubVel.publish(vel);
				pubDir.publish(angDir);
			}	
		}

	void scan_callback(const sensor_msgs::LaserScan scan) //FALTA DEFINIR QUE TIPO ES SCAN
	{
		if (fin==false){
			if (espacioFlag==0){
				if (an180.data>=95){
					l1=-dist180.data*cos(an180.data*3.1416/180.0);
					l2=dist90.data*cos(an90.data*3.1416/180.0);
					espacio.data=l1+l2;
					if (espacio.data>0.80){
						espacioFlag=1;
						//launch_lane.shutdown()  //WARNING
						vel.data=-200;
						pubVel.publish(vel);
					}
					ROS_INFO("%f",espacio.data);
				}
			}
			else{	

				if (an180.data<100){
					objBandera=true;
				}
				x=dist180.data*sin(an180.data*3.1416/180.0);
				//limite=corr();
				if (objBandera==true){
					if (max_180.data>=corr()){
						vel.data=0;
						pubVel.publish(vel);
						usleep(1000); //WARNING
						fin=true;
						routine();
					}
				}
			}
		}
		else{
			vel.data=0;
			angDir.data=100;
			pubVel.publish(vel);
			ros::shutdown();   
			pubDir.publish(angDir);
		}
	}
};


int main(int argc, char** argv) 
{
	intCent = 0.0;
	vel.data=0;
	espacioFlag=0;
	fin= false;
	yaw_inic.data=360;

	objBandera=false;

	ros::init(argc, argv, "Master_parking");
	ROS_INFO("Master parking running...");
	Master_parking ic;

	if(ros::isShuttingDown()==true){
		ROS_INFO("Fin del programa");
	}


	ros::spin();
	return 0;
}