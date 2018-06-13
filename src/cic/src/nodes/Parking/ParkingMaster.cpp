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
#include <math.h>


std_msgs::Int16 vel, angDir;
float max90=0, max180=0, min180=100, min330=100, min180i, alfaR, dr, yaw, yaw2=0.01, yaw2a=0.01; 
int pos90, pos180, posmin180, posmin330;

float maxvel=500;
float alfaK=18.0;
float DF=33;
float k1=20.0;
float k2=50.0;
float k3=35.0;
float k4=1.0;
float k5=3.0;
float k6=50.0;
float k7=25.0;
float e1;
float e2;
float e3;
float e6;
float e7;

bool et0=false, et1=false, et2=false, et3=false, et4=false, et5=false, et6=false, et7=false, yawflag=false ;
float alfaE;
float a;
float d1;
float DeltaAlfa;
float AlfaD;
float yawinic;


class Master_parking
{
	ros::NodeHandle nh_;
	ros::Publisher pubDir,pubVel;
	ros::Subscriber scan_,yaw2_, pos90_,max90_,pos180_,max180_,posmin180_,posmin330_,min180_, min330_;
	public:
	  Master_parking() 
	  //: it_(nh_)  
	  {
			scan_ = nh_.subscribe("/scan", 1, &Master_parking::scan_callback, this);
			yaw2_ =nh_.subscribe("/model_car/yaw",1, &Master_parking::yawF_callback, this);
			max90_= nh_.subscribe("/parking/max90",1, &Master_parking::max90_callback, this);
			max180_= nh_.subscribe("/parking/max180",1, &Master_parking::max180_callback, this);
			min180_= nh_.subscribe("/parking/min180",1, &Master_parking::min180_callback, this);
			min330_= nh_.subscribe("/parking/min330",1, &Master_parking::min330_callback, this);
			pos90_= nh_.subscribe("/parking/pos90",1, &Master_parking::pos90_callback, this);
			pos180_= nh_.subscribe("/parking/pos180",1, &Master_parking::pos180_callback, this);
			posmin180_= nh_.subscribe("/parking/posmin180",1, &Master_parking::posmin180_callback, this);
			posmin330_= nh_.subscribe("/parking/posmin330",1, &Master_parking::posmin330_callback, this);

			pubDir= nh_.advertise<std_msgs::Int16>("/manual_control/steering",1);
			pubVel= nh_.advertise<std_msgs::Int16>("/manual_control/speed",1);
	  }


	void yawF_callback(const std_msgs::Float32 ros_dummie)
	{
		yaw2a=yaw2;
		yaw2=ros_dummie.data;
		if((yaw2>yaw2a) && (((yaw2/(yaw2a))>0)))
			yaw=yaw+(yaw2-yaw2a);
		else{
			if((yaw2<yaw2a) && (((yaw2/(yaw2a))>0)))
				yaw=yaw-(yaw2a-yaw2);
			else{
				if((yaw2>yaw2a) && (((yaw2/(yaw2a))<0)) &&(abs(yaw2)<90))
					yaw=yaw+(yaw2-yaw2a);
				else{
					if((yaw2<yaw2a) && (((yaw2/(yaw2a))<0)) &&(abs(yaw2)<90))
						yaw=yaw-(yaw2a-yaw2);
					else{
						if((yaw2>yaw2a) && (((yaw2/(yaw2a))<0)) &&(abs(yaw2)>90))
							yaw=yaw-((180-yaw2)+(180+yaw2a));
						else{
							if((yaw2<yaw2a) && (((yaw2/(yaw2a))<0)) &&(abs(yaw2)>90))
								yaw=yaw+((180-yaw2a)+(180+yaw2a));
						}
					}
				}
			}
		}
		
	}

	void max90_callback(const std_msgs::Float32 ros_dummie){
		max90=ros_dummie.data;
	}

	void max180_callback(const std_msgs::Float32 ros_dummie){
		max180=ros_dummie.data;
	}

	void min180_callback(const std_msgs::Float32 ros_dummie){
		min180=ros_dummie.data;
	}

	void min330_callback(const std_msgs::Float32 ros_dummie){
		min330=ros_dummie.data;
	}

	void pos90_callback(const std_msgs::Int16 ros_dummie){
		pos90=ros_dummie.data;
	}

	void pos180_callback(const std_msgs::Int16 ros_dummie){
		pos180=ros_dummie.data;
	}

	void posmin180_callback(const std_msgs::Int16 ros_dummie){
		posmin180=ros_dummie.data;
	}

	void posmin330_callback(const std_msgs::Int16 ros_dummie){
		posmin330=ros_dummie.data;
	}

	void scan_callback(const sensor_msgs::LaserScan scan) 
	{	
		if (min180!=100){ //Garantiza que ya se llego al espacio
			routine();
		}
		
	}

	void routine() 
	{
				

			
		if (et0==false){
			dr=sqrt(max180*max180-min180*min180);
			alfaE=90-posmin180;
			a=min180;
			d1=a*0.8+16; 
			DeltaAlfa=alfaK;
			

			ROS_INFO("dr: %f",dr);
			ROS_INFO("AlfaE: %f",alfaE);
			ROS_INFO("a: %f",a);
			ROS_INFO("d1: %f",d1);
			ROS_INFO("Entro a rutina");
			et0=true;
			vel.data=-maxvel*0.25;
			pubVel.publish(vel);
		}
		//Angle Adjustment

		if (et0==true && et1==false){
			e1=(90-posmin180)*k1;
			if (abs(e1)>90){
				e1=90*(e1/abs(e1));
			}
			angDir.data=e1+92;
			pubDir.publish(angDir);
			if(abs(e1)<2){
				et1=true;
				ROS_INFO("et1 done");
			}
		}
		
		//Vertical Adjustment
		if  (et1==true && et2==false){
			
			dr=sqrt(max180*max180-min180*min180);
			e2=(d1-dr);
			vel.data=-e2*k2;
			if (abs(vel.data)>maxvel){
				vel.data=maxvel*(vel.data/abs(vel.data));
			}
			pubVel.publish(vel);
			ROS_INFO("e2: %f",e2);
			if(abs(e2)<1.5){
				et2=true;
				ROS_INFO("et2 done");
				angDir.data=0;
				pubDir.publish(angDir);
				vel.data=400;
				pubVel.publish(vel);
				yawinic=yaw;
				AlfaD=yawinic-DeltaAlfa;
			}
		}

		//Turn Wheels right
		if  (et2==true && et3==false){
			for(unsigned long long i=0;i<100000000;i++){}
			e3=(AlfaD-yaw);
			
			vel.data=-e3*k3;
			if (abs(vel.data)>maxvel){
				vel.data=maxvel*(vel.data/abs(vel.data));
			} 
			
			pubVel.publish(vel);
			if(abs(e3)<3){
				et3=true;
				ROS_INFO("et3 done");
				angDir.data=93;
				pubDir.publish(angDir);
				vel.data=maxvel*0.7; //maxvel
				pubVel.publish(vel);
				min180i=min180;
				AlfaD=yawinic;
			}
		}

		//Backwards Adjustment
		if (et3==true && et4==false){
			if((posmin180<(90-alfaK*1.15-a/4))){ 
				et4=true;
				ROS_INFO("et4 done");
				angDir.data=179;
				pubDir.publish(angDir);
				et5=true;
				AlfaD=yaw+DeltaAlfa+3;
			}
		}

		//Turn Wheels left
		if  (et5==true && et6==false){
			e6=(AlfaD-yaw);
			vel.data=(e6*k6)+(e6*50/abs(e6));

			if (abs(vel.data)>maxvel){
				vel.data=maxvel*(vel.data/abs(vel.data));
			}
			pubVel.publish(vel);
			ROS_INFO("e6: %f",e6);
			if((abs(e6)<3)){
				et6=true;
				ROS_INFO("et6 done");
				angDir.data=95;
				pubDir.publish(angDir);
				vel.data=0;
				pubVel.publish(vel);
			}

			if(((posmin330<=2)||(posmin330>=357))&&(abs(e6)<10)){
				et6=true;
				ROS_INFO("et6 done");
				angDir.data=95;
				pubDir.publish(angDir);
				vel.data=-100;
				pubVel.publish(vel);
			}
		}
		
		//Final Adjustment
		if  (et6==true && et7==false){
			e7=(min330-DF);
			vel.data=-e7*k7;
			if (abs(vel.data)>maxvel)
				vel.data=maxvel*(vel.data/abs(vel.data));
			pubVel.publish(vel);
			ROS_INFO("e7: %f",e7);
			if((abs(e7)<1.5)){
				et7=true;
				ROS_INFO("et7 done");
				
				vel.data=0;
				pubVel.publish(vel);
				ROS_INFO("Done");
			}
		}
	}	
};

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "Master_parking");
	ROS_INFO("Master parking running.");
	Master_parking ic;
	if(ros::isShuttingDown()==true){
		ROS_INFO("Fin del programa");
	}

	ros::spin();
	return 0;
}

