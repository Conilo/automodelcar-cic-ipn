#include <ros/ros.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <ros/console.h>
#include <vector>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

std_msgs::Int16 an90, an180, max_180;
std_msgs::Float32 dist90, dist180, dist270, dist360,espacio,yaw_inic, yaw, yaw2, yaws, yaw_rel,dist1,dist2,dist3,dist4, ang1d,ang2d;
sensor_msgs::LaserScan newscan;
std::vector<float> RoI, obj90_r, obj90_i, obj180_r, obj180_i, obj270_r, obj270_i, obj360_r, obj360_i, ranges (360);
 

std::vector<float> intensities;
//int ranges[360];

int intCent ;


float MAX_DISTANCE = .6;
int AMP_ANG_min = 0;
int AMP_ANG_max = 359;
float ang1, ang2;



class Pub_slave
{
	ros::NodeHandle nh;
	ros::Publisher laser_,obj_90_,obj_180_ ,obj_270_,obj_360_,obj_angle90_,obj_angle180_, max180_,yaw2_;
	ros::Subscriber scan_s_, yaw_s_;
	public:
	  Pub_slave() 
	  //: it_(nh)  
	  {
			laser_ = nh.advertise<sensor_msgs::LaserScan>("/scan_follower/scan", 1);
			obj_90_ = nh.advertise<std_msgs::Float32>("/scan_follower/obj_90", 1);
			obj_180_ = nh.advertise<std_msgs::Float32>("/scan_follower/obj_180", 1);
			obj_270_ = nh.advertise<std_msgs::Float32>("/scan_follower/obj_270",1);
			obj_360_ =nh.advertise<std_msgs::Float32>("/scan_follower/obj_360",1);
			obj_angle90_= nh.advertise<std_msgs::Int16>("/scan_follower/obj_angle90",1);
			obj_angle180_ =nh.advertise<std_msgs::Int16>("/scan_follower/obj_angle180",1);
			max180_ =nh.advertise<std_msgs::Int16>("/scan_follower/max180",1);
			yaw2_ =nh.advertise<std_msgs::Float32>("/scan_follower/yaw2",1);
			scan_s_ = nh.subscribe("/scan", 1, &Pub_slave::laser_callback, this);
			yaw_s_ = nh.subscribe("/model_car/yaw", 1, &Pub_slave::yaw_callback, this);
	  }

	  void myShut()
	  {
	  	ROS_INFO("Fin del programa");
	  }

	  void yaw_callback(const std_msgs::Float32 ros_dummie)
	  {
	  	yaw2.data=ros_dummie.data;
	  	yaw2_.publish(yaw2);
	  }

	  void laser_callback(const sensor_msgs::LaserScan scan)
	  {
	  	newscan= scan;


	  	for (int i=0; i<360;i++)
	  	{
	  		intensities.push_back(0.0);
	  		//ranges.push_back(0); // WARNING
	  	}
	  	for (float i=AMP_ANG_min; i<=AMP_ANG_max; i++)
	  	{
	  		RoI.push_back(i);
	  	}

	  	for (int i; i<RoI.size(); i++){
	  		if(scan.ranges[i] < MAX_DISTANCE)
	  		{
	  			ranges[i]= scan.ranges[i];
	  			intensities[i]= scan.intensities[i];
	  		}
	  		if (i <=90){
	  			obj90_r.push_back(scan.ranges[i]);
	  			obj90_i.push_back(i);
	  		}
  			if (i<=180 && i>90){
  				obj180_r.push_back(scan.ranges[i]);
  				obj180_i.push_back(i);
  			}
  			if (i<=270 && i>180){
  				obj270_r.push_back(scan.ranges[i]);
  				obj270_i.push_back(i);
  			}
  			if (i<=360 && i>270){
  				obj360_r.push_back(scan.ranges[i]);
  				obj360_i.push_back(i);
  			}
	  	}
	  	if(obj90_r.empty()==true){
	  		obj90_r.push_back(1.0);
	  		obj90_i.push_back(90);
	  	}
	  	if(obj180_r.empty()==true){
	  		obj180_r.push_back(1.0);
	  		obj180_i.push_back(90);
	  	}
	  	if(obj270_r.empty()==true){
	  		obj270_r.push_back(1.0);
	  		obj270_i.push_back(90);
	  	}
	  	if(obj360_r.empty()==true){
	  		obj360_r.push_back(1.0);
	  		obj360_i.push_back(90);
	  	}

	  	dist1.data=*std::min_element(obj90_r.begin(),obj90_r.end());
	  	dist2.data=*std::min_element(obj180_r.begin(),obj180_r.end());
	  	dist3.data=*std::min_element(obj270_r.begin(),obj270_r.end());
	  	dist4.data=*std::min_element(obj360_r.begin(),obj360_r.end());

	  	float ang1=*std::min_element(obj90_i.begin(),obj90_i.end());
	  	float ang2=*std::min_element(obj180_i.begin(),obj180_i.end());
	  	max_180.data=obj180_i.back();
	  	float x= dist2.data*sin(ang2*3.1416/180); //Warning tipo de sin
	  	ROS_INFO("%f",x);

	  	obj_90_.publish(dist1);
	  	obj_180_.publish(dist2);
	  	obj_270_.publish(dist3);
	  	obj_360_.publish(dist4);

	  	ang1d.data= ang1;
	  	ang2d.data= ang2;
	  	obj_angle90_.publish(ang1d);   
	  	obj_angle180_.publish(ang2d);
	  	max180_.publish(max_180);

	  	newscan.ranges = ranges;
	  	newscan.intensities = intensities;
	  	laser_.publish(newscan);

	  	obj90_r.clear();
	  	obj90_i.clear();
	  	obj180_r.clear();
	  	obj180_i.clear();
	  	obj270_r.clear();
	  	obj270_i.clear();
	  }
};



int main(int argc, char** argv) 
{
	
	intCent= 0.0;
	ros::init(argc, argv, "Pub_slave");
	ROS_INFO("Pub slave running...");
	Pub_slave ic;
	if(ros::isShuttingDown()==true){
		ROS_INFO("Fin del programa");
	}

	ros::spin();
	return 0;
	
}

