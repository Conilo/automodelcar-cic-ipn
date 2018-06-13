//V5
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
float k1=20.0;
float k2=50.0;
float k3=35.0;
float k4=1.0;
float k5=3.0;
float k6=50.0;
float k7=25.0;
float DF=33;
float alfaK=18.0;
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
		

		//ROS_INFO("yaw: %f",yaw);
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
			a=min180;//ranges[int((pos180-pos90)/2+pos90)]*100;
			d1=a*0.8+16; //Parametro de ajuste vertical 
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

		if (et0==true && et1==false){
			e1=(90-posmin180)*k1;
			if (abs(e1)>90){
				e1=90*(e1/abs(e1));
			}
			angDir.data=e1+92;
			

			pubDir.publish(angDir);
			//ROS_INFO("e1: %f",e1);
			if(abs(e1)<2){
				et1=true;
				ROS_INFO("et1 done");
				//angDir.data=100;
				//pubDir.publish(angDir);
			}

		}
		
		//Ajuste vertical
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

		//Girar llantas a la derecha
		if  (et2==true && et3==false){
			//for(unsigned long long i=0;i<380000000;i++){}
			for(unsigned long long i=0;i<100000000;i++){}
			e3=(AlfaD-yaw);
			
			vel.data=-e3*k3;
			if (abs(vel.data)>maxvel){
				vel.data=maxvel*(vel.data/abs(vel.data));
			} 
			

			//vel.data=300; //Comentar esta linea para hacerlo por control kp
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

		
		
		//Ajuste Diagonal
		
		if (et3==true && et4==false){
			

			if((posmin180<(90-alfaK*1.15-a/4))){  //(min180>a*0.8)&&
				et4=true;
				ROS_INFO("et4 done");
				angDir.data=179;
				pubDir.publish(angDir);
				et5=true;
				AlfaD=yaw+DeltaAlfa+3;
			}
			
		}
		

		//Girar llantas a la Izquierda
		
		if  (et5==true && et6==false){
			
			e6=(AlfaD-yaw);
			vel.data=(e6*k6)+(e6*50/abs(e6));

			if (abs(vel.data)>maxvel){
				vel.data=maxvel*(vel.data/abs(vel.data));
			}
			pubVel.publish(vel);

			/* //Para hacerlo con el lidar
			if (posmin330>180&& posmin330<=360)
				e6=(360-(posmin330-360));
			if (posmin330<180&& posmin330>=0)
				e6=((posmin330+360)-360);
			e6=(180-(posmin330-180));
			vel.data=(e6*k6)//+(e6*50/abs(e6));
				*/
			

			//ROS_INFO("AlfaD: %f",AlfaD);
			//ROS_INFO("yaw: %f",yaw);
			

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

		
		//Ajuste Final hacia adelante
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


/*

//V4
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

float maxvel=350;
float k1=20.0;
float k2=50.0;
float k3=35.0;
float k4=1.0;
float k5=3.0;
float k6=50.0;
float k7=25.0;
float DF=33;
float alfaK=29.0;
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
		

		//ROS_INFO("yaw: %f",yaw);
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
			a=min180;//ranges[int((pos180-pos90)/2+pos90)]*100;
			d1=a*0.8+18; //Parametro de ajuste vertical 
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

		if (et0==true && et1==false){
			e1=(90-posmin180)*k1;
			if (abs(e1)>90){
				e1=90*(e1/abs(e1));
			}
			angDir.data=e1+92;
			

			pubDir.publish(angDir);
			//ROS_INFO("e1: %f",e1);
			if(abs(e1)<2){
				et1=true;
				ROS_INFO("et1 done");
				//angDir.data=100;
				//pubDir.publish(angDir);
			}

		}
		
		//Ajuste vertical
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

		//Girar llantas a la derecha
		if  (et2==true && et3==false){
			//for(unsigned long long i=0;i<380000000;i++){}
			e3=(AlfaD-yaw);
			vel.data=-e3*k3;
			if (abs(vel.data)>maxvel){
				vel.data=maxvel*(vel.data/abs(vel.data));
			}
			pubVel.publish(vel);
			//ROS_INFO("AlfaE: %f",alfaE);
			//ROS_INFO("AlfaD: %f",AlfaD);
			//ROS_INFO("yaw: %f",yaw);
			//ROS_INFO("e3: %f",e3);
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

		
		
		//Ajuste Diagonal
		
		if (et3==true && et4==false){
			//ROS_INFO("min180i: %f",min180i);
			//ROS_INFO("min180: %f",min180);
			//ROS_INFO("max90: %f",max90);
			

			if((posmin180<(90-alfaK*1.3-a/3))){  //(min180>a*0.8)&&
				et4=true;
				ROS_INFO("et4 done");
				angDir.data=180;
				pubDir.publish(angDir);
				et5=true;
				AlfaD=yaw+DeltaAlfa+3;
			}
			
		}
		

		//Girar llantas a la Izquierda
		
		if  (et5==true && et6==false){
			
			e6=(AlfaD-yaw);
			vel.data=(e6*k6)+(e6*50/abs(e6));

			if (abs(vel.data)>maxvel){
				vel.data=maxvel*(vel.data/abs(vel.data));
			}
			pubVel.publish(vel);

			/* //Para hacerlo con el lidar
			if (posmin330>180&& posmin330<=360)
				e6=(360-(posmin330-360));
			if (posmin330<180&& posmin330>=0)
				e6=((posmin330+360)-360);
			e6=(180-(posmin330-180));
			vel.data=(e6*k6)//+(e6*50/abs(e6));
			
			

			//ROS_INFO("AlfaD: %f",AlfaD);
			//ROS_INFO("yaw: %f",yaw);
			

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
				vel.data=0;
				pubVel.publish(vel);
			}
		}

		
		//Ajuste Final hacia adelante
		if  (et6==true && et7==false){
			e7=(min330-DF);
			vel.data=-e7*k7;
			if (abs(vel.data)>maxvel)
				vel.data=maxvel*(vel.data/abs(vel.data));
			pubVel.publish(vel);
			ROS_INFO("e7: %f",e7);
			if((abs(e7)<3)){
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


*/


/*

//V3
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
std_msgs::Float32 yaw;
float max90=0, max180=0, min180=100, alfaR, dr; 
int pos90, pos180, posmin180;

int cont=0, SpaceValidator=0, CarOneStartIndex=0, CarTwoStartIndex=0, ClearSpaceStartIndex=0;
std::vector<int> Space;
std::vector<float> ranges;
bool bandera=false, SpaceFoundFlag=false, CarOneFlag=false, CarTwoFlag=false, ClearSpaceFlag=false, routineFlag=false, PosDetectorFlag=false;

//Parametros
int NumOfDegres=7; //Dice el tamano del angulo a evaluar para validar si hay objeto o no
int NumOfValidations=2; //Dice cuantas validaciones correctas debe encontrar para determinar si hay espacio o no
int NumOfValidations2=3; //Tamano minimo de carone, cartwo, clearspace
int SpaceSize=20;// Dice el tamano minimo de validaciones para determinar que hay espacio en donde quepa el carro completo
float MaxDistance2Obj=25.0; //Distancia maxima del LIDAR al Objeto a detectar
int MaxRange=40; //Maximos centimetros de radio de búsqueda
int MinRange=10; //MinRange centimetros de radio de búsqueda


class Master_parking
{
	ros::NodeHandle nh_;
	ros::Publisher pubDir,pubVel;
	ros::Subscriber scan_,yaw2_;
	public:
	  Master_parking() 
	  //: it_(nh_)  
	  {
			scan_ = nh_.subscribe("/scan", 1, &Master_parking::scan_callback, this);
			yaw2_ =nh_.subscribe("/scan_follower/yaw2",1, &Master_parking::yawF_callback, this);

			pubDir= nh_.advertise<std_msgs::Int16>("/manual_control/steering",1);
			pubVel= nh_.advertise<std_msgs::Int16>("/manual_control/speed",1);
	  }

	void yawF_callback(const std_msgs::Float32 ros_dummie)
	{
		yaw.data=ros_dummie.data;
	}

	void routine() 
	{
		float k1=25.0;
		float k2=15.0;
		float k3=30.0;
		float k4=10.0;
		float k5=15.0;
		float k6=25.0;
		float DF=15.0;
		
		float alfaK=30.0;

		
		float alfaE=alfaR;
		float a=min180;//ranges[int((pos180-pos90)/2+pos90)]*100;
		float d1=a/2 +15;
		float DeltaAlfa=alfaE+alfaK;

		ROS_INFO("AlfaE: %f",alfaE);
		ROS_INFO("a: %f",a);
		
		ROS_INFO("d1: %f",d1);



		ROS_INFO("Entro a rutina");
		

		//Ajuste vertical
		float e=(d1-dr);
		while(abs(e>3)){
			e=(d1-dr);
			//vel.data=-e*k1;
			//pubVel.publish(vel);
			ROS_INFO("e: %f",e);
		}

		ROS_INFO("Muy bien");

		
		/*
		
		//Girar llantas a la derecha
		angDir.data=0;
		pubDir.publish(angDir);

		AlfaD=AlfaA-DeltaAlfa;
		e2=(AlfaA-AlfaD);
		while(abs(e2>5)){
			e2=(AlfaA-AlfaD);
			vel.data=e2*k2;
			pubVel.publish(vel);
		}

		//Ajuste Diagonal
		angDir.data=95;
		pubDir.publish(angDir);
		vel.data=300;
		pubVel.publish(vel);
		while(max90<k3){};
		while(min90>k4){};

		//Girar llantas a la Izquierda
		angDir.data=180;
		pubDir.publish(angDir);

		AlfaD=AlfaA+DeltaAlfa;
		e3=(AlfaA-AlfaD);
		while(abs(e3>5)){
			e3=(AlfaA-AlfaD);
			vel.data=e3*k5;
			pubVel.publish(vel);
		}

		//Ajuste Final hacia adelante
		e4=(scan.ranges[0]-DF);
		while(abs(e4>5)){
			e4=(scan.ranges[0]-DF);
			vel.data=e4*k6;
			pubVel.publish(vel);
		}

		vel.data=0;
		pubVel.publish(vel);
		ROS_INFO("Done");
		
	}

	void scan_callback(const sensor_msgs::LaserScan scan) 
	{	
		ranges=scan.ranges;
		if (bandera==false){
				for(unsigned long long i=0;i<50000000;i++){}
				angDir.data=0;
				pubDir.publish(angDir);
				for(unsigned long long i=0;i<30000000;i++){}
				angDir.data=92;
				vel.data=-400;
				pubVel.publish(vel);
				pubDir.publish(angDir);
				bandera=true;
			}
		if(CarTwoFlag==false){
			cont=0;
			SpaceValidator=0;

			//Escaneo de seccion de angulo, cuenta las veces que el laser choca a una distancia menor a 20
			for (int i=(90-NumOfDegres/2); i<=90+NumOfDegres/2; i++){
		  		if(scan.ranges[i]*100 < MaxDistance2Obj){
		  			cont++;
		  		}
		  	}
		  	if (cont>=NumOfValidations){ //Llena un primer vector con 0 si hay objeto y con 1 si hay espacio
		  		Space.push_back(0); //Signidica que hay objeto 
		  	}
		  	else{
		  		Space.push_back(1); //Significa que hay espacio
		  	}

		  	//ROS_INFO("%d",Space.back());

		  	if ((Space.size()>=SpaceSize)){ //Si ya se corrienron al menos las ejecuciones del tamano de spacesize
			  	if(((Space.back()!=Space[Space.size()-2]))&&(CarOneFlag==false)){
			  		CarOneFlag=true;
			  		CarOneStartIndex=Space.size();
			  		ROS_INFO("CARROUNO");
			  	}
			  	if(((Space.back()!=Space[Space.size()-2]))&&(CarOneFlag==true)&&(ClearSpaceFlag==false)&&(Space.size()>(CarOneStartIndex+NumOfValidations2))){
			  		ClearSpaceFlag=true;
			  		ClearSpaceStartIndex=Space.size();
			  		vel.data=-400;
					pubVel.publish(vel);
				  	ROS_INFO("CLEARSPACE");
			  	}
			  	if(((Space.back()!=Space[Space.size()-2]))&&(ClearSpaceFlag==true)&&(CarTwoFlag==false)&&(Space.size()>(ClearSpaceStartIndex+NumOfValidations2))){
			  		CarTwoFlag=true;
			  		CarTwoStartIndex=Space.size();
			  		ROS_INFO("CARTWO");
			  		ROS_INFO("%d, %d, %d",CarOneStartIndex,ClearSpaceStartIndex,CarTwoStartIndex);
			  		vel.data=0;
					pubVel.publish(vel);
			  		routineFlag=true;
			  	}
			}
  		}

  		routineFlag=true;
  		if(routineFlag==true) {
  			for(int i=0; i<360; i++) {
  				if((i<=90) &&((scan.ranges[i]*100<MaxRange)&&(scan.ranges[i]*100>MinRange))) {
  					if(scan.ranges[i]*100>max90){
  						max90=scan.ranges[i]*100;
  						pos90=i;
  					}
  				}
  				if(((i>90)&&(i<=180)) &&((scan.ranges[i]*100<MaxRange)&&(scan.ranges[i]*100>MinRange))) {
  					if(scan.ranges[i]*100>max180){
  						max180=scan.ranges[i]*100;
  						pos180=i;
  					}
  				}
  				if(((i>=0)&&(i<=180)) &&((scan.ranges[i]*100<MaxRange)&&(scan.ranges[i]*100>MinRange))) {
  					if(scan.ranges[i]*100<min180){
  						min180=scan.ranges[i]*100;
  						posmin180=i;
  					}
  				}
  					
  			}
  			
  			ROS_INFO("max90: %f",max90);
  			ROS_INFO("max180: %f",max180);
  			ROS_INFO("min180: %f",min180);
  			ROS_INFO("pos90: %d",pos90);
  			ROS_INFO("pos180: %d",pos180);
  			ROS_INFO("posmin180: %d",posmin180);
  			
  			


  			alfaR=90-posmin180;
  			dr=sqrt(max180*max180-min180*min180);
  			ROS_INFO("dr: %f",dr);
  			if(PosDetectorFlag==false){
  				PosDetectorFlag=true;
  				routine();
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

*/



/*
// V2
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


std_msgs::Int16 vel, angDir;
std_msgs::Float32 yaw;
float ranges[360];
int cont=0, SpaceValidator=0, CarOneStartIndex=0, CarTwoStartIndex=0, ClearSpaceStartIndex=0;
std::vector<int> Space;
bool bandera=false, SpaceFoundFlag=false, CarOneFlag=false, CarTwoFlag=false, ClearSpaceFlag=false;

//Parametros
int NumOfDegres=7; //Dice el tamano del angulo a evaluar para validar si hay objeto o no
int NumOfValidations=3; //Dice cuantas validaciones correctas debe encontrar para determinar si hay espacio o no
int NumOfValidations2=5; //Tamano minimo de carone, cartwo, clearspace
int SpaceSize=20;// Dice el tamano minimo de validaciones para determinar que hay espacio en donde quepa el carro completo
float MaxDistance2Obj=25.0; //Distancia mínima del LIDAR al Objeto a detectar


class Master_parking
{
	ros::NodeHandle nh_;
	ros::Publisher pubDir,pubVel;
	ros::Subscriber scan_,yaw2_;
	public:
	  Master_parking() 
	  //: it_(nh_)  
	  {
			scan_ = nh_.subscribe("/scan", 1, &Master_parking::scan_callback, this);
			yaw2_ =nh_.subscribe("/scan_follower/yaw2",1, &Master_parking::yawF_callback, this);

			pubDir= nh_.advertise<std_msgs::Int16>("/manual_control/steering",1);
			pubVel= nh_.advertise<std_msgs::Int16>("/manual_control/speed",1);
	  }

	void yawF_callback(const std_msgs::Float32 ros_dummie)
	{
		yaw.data=ros_dummie.data;
	}

	void routine() 
	{
		ROS_INFO("Entro a rutina");
		vel.data=-400;
		pubVel.publish(vel);
		for(unsigned long long i=0;i<160000000;i++){}
		angDir.data=95;
		vel.data=400;
		pubVel.publish(vel);
		pubDir.publish(angDir);
		angDir.data=0;
		pubDir.publish(angDir);
		for(unsigned long long i=0;i<300000000;i++){}
		angDir.data=90;
		pubDir.publish(angDir);
		for(double i=0;i<200000000;i++){}
		angDir.data=180;
		pubDir.publish(angDir);
		for(double i=0;i<200000000;i++){}
		vel.data=0;
		pubVel.publish(vel);
		vel.data=-200;
		pubVel.publish(vel);
		angDir.data=90;
		pubDir.publish(angDir);
		for(double i=0;i<250000000;i++){}
		vel.data=0;
		pubVel.publish(vel);
		ROS_INFO("Done");
	}

	void scan_callback(const sensor_msgs::LaserScan scan) 
	{	
		if (bandera==false){
				for(unsigned long long i=0;i<50000000;i++){}
				angDir.data=0;
				pubDir.publish(angDir);
				for(unsigned long long i=0;i<30000000;i++){}
				angDir.data=92;
				vel.data=-400;
				pubVel.publish(vel);
				pubDir.publish(angDir);
				bandera=true;
			}
		if(CarTwoFlag==false){
			cont=0;
			SpaceValidator=0;

			//Escaneo de seccion de angulo, cuenta las veces que el laser choca a una distancia menor a 20
			for (int i=(90-NumOfDegres/2); i<=90+NumOfDegres/2; i++){
		  		if(scan.ranges[i]*100 < MaxDistance2Obj){
		  			cont++;
		  		}
		  	}
		  	if (cont>=NumOfValidations){ //Llena un primer vector con 0 si hay objeto y con 1 si hay espacio
		  		Space.push_back(0); //Signidica que hay objeto 
		  	}
		  	else{
		  		Space.push_back(1); //Significa que hay espacio
		  	}

		  	//ROS_INFO("%d",Space.back());

		  	if ((Space.size()>=SpaceSize)){ //Si ya se corrienron al menos las ejecuciones del tamano de spacesize
			  	if(((Space.back()!=Space[Space.size()-2]))&&(CarOneFlag==false)){
			  		CarOneFlag=true;
			  		CarOneStartIndex=Space.size();
			  		ROS_INFO("CARROUNO");
			  	}
			  	if(((Space.back()!=Space[Space.size()-2]))&&(CarOneFlag==true)&&(ClearSpaceFlag==false)&&(Space.size()>(CarOneStartIndex+NumOfValidations2))){
			  		ClearSpaceFlag=true;
			  		ClearSpaceStartIndex=Space.size();
				  	ROS_INFO("CLEARSPACE");
			  	}
			  	if(((Space.back()!=Space[Space.size()-2]))&&(ClearSpaceFlag==true)&&(CarTwoFlag==false)&&(Space.size()>(ClearSpaceStartIndex+NumOfValidations2))){
			  		CarTwoFlag=true;
			  		CarTwoStartIndex=Space.size();
			  		ROS_INFO("CARTWO");
			  		ROS_INFO("%d, %d, %d",CarOneStartIndex,ClearSpaceStartIndex,CarTwoStartIndex);
			  		vel.data=0;
					pubVel.publish(vel);
			  		routine();
			  	}
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



*/








/* //V2
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


std_msgs::Int16 vel, angDir;
std_msgs::Float32 yaw;
float ranges[360], intensities[360];
int cont=0, SpaceValidator=0, CarOneStartIndex=0, CarTwoStartIndex=0, ClearSpaceStartIndex=0;
std::vector<int> Space, Valid1;
bool bandera=false, SpaceFoundFlag=false, CarOneFlag=false, CarTwoFlag=false, ClearSpaceFlag=false;

//Parametros
int NumOfDegres=5; //Dice el tamano del angulo a evaluar para validar si hay objeto o no
int NumOfValidations=3; //Dice cuantas validaciones correctas debe encontrar para determinar si hay espacio o no
int NumOfValidations2=5;
int SpaceSize=20;// Dice el tamano minimo de validaciones para determinar que hay espacio en donde quepa el carro completo


class Master_parking
{
	ros::NodeHandle nh_;
	ros::Publisher pubDir,pubVel;
	ros::Subscriber scan_,yaw2_;
	public:
	  Master_parking() 
	  //: it_(nh_)  
	  {
			scan_ = nh_.subscribe("/scan", 1, &Master_parking::scan_callback, this);
			yaw2_ =nh_.subscribe("/scan_follower/yaw2",1, &Master_parking::yawF_callback, this);

			pubDir= nh_.advertise<std_msgs::Int16>("/manual_control/steering",1);
			pubVel= nh_.advertise<std_msgs::Int16>("/manual_control/speed",1);
	  }

	void yawF_callback(const std_msgs::Float32 ros_dummie)
	{
		yaw.data=ros_dummie.data;
	}

	void routine() //espacio ya esta definida como variable global
	{
		ROS_INFO("Entro a rutina");
		angDir.data=100;
		vel.data=20;
		pubVel.publish(vel);
		pubDir.publish(angDir);
		usleep(1000);
		angDir.data=150;
		pubDir.publish(angDir);
		usleep(2000);
		angDir.data=97;
		pubDir.publish(angDir);
		usleep(5000);
		angDir.data=30;
		pubDir.publish(angDir);
		usleep(2000);
		vel.data=0;
		pubVel.publish(vel);
		usleep(1000);
		vel.data=-30;
		pubVel.publish(vel);
		usleep(500);
		vel.data=0;
		pubVel.publish(vel);
	}

	void scan_callback(const sensor_msgs::LaserScan scan) 
	{
		if (bandera==false){
				angDir.data=100;
				vel.data=-200;
				pubVel.publish(vel);
				pubDir.publish(angDir);
				bandera=true;
			}
		if(CarTwoFlag==false){
			cont=0;
			SpaceValidator=0;

			//Escaneo de seccion de angulo, cuenta las veces que el laser choca a una distancia menor a 20
			for (int i=(90-NumOfDegres/2); i<=90+NumOfDegres/2; i++){
		  		if(scan.ranges[i]*100 < 20.0){
		  			cont++;
		  		}
		  	}
		  	if (cont>=NumOfValidations){ //Llena un primer vector con 0 si hay objeto y con 1 si hay espacio
		  		Space.push_back(0); //Signidica que hay objeto 
		  	}
		  	else{
		  		Space.push_back(1); //Significa que hay espacio
		  	}

		  	//ROS_INFO("%d",Space.back());

		  	if ((Space.size()>=SpaceSize)){ //Si ya se corrienron al menos las ejecuciones del tamano de spacesize
			  	if(((Space.back()!=Space[Space.size()-2]))&&(CarOneFlag==false)){
			  		CarOneFlag=true;
			  		CarOneStartIndex=Space.size();
			  		ROS_INFO("CARROUNO, Ultimo: %d Antepenultimo: %d",Space.back(),Space[Space.size()-2]);
			  	}
			  	if(((Space.back()!=Space[Space.size()-2]))&&(CarOneFlag==true)&&(ClearSpaceFlag==false)&&(Space.size()>(CarOneStartIndex+NumOfValidations2))){
			  		ClearSpaceFlag=true;
			  		ClearSpaceStartIndex=Space.size();
				  	ROS_INFO("CLEARSPACE");
			  	}
			  	if(((Space.back()!=Space[Space.size()-2]))&&(ClearSpaceFlag==true)&&(CarTwoFlag==false)&&(Space.size()>(ClearSpaceStartIndex+NumOfValidations2))){
			  		CarTwoFlag=true;
			  		CarTwoStartIndex=Space.size();
			  		ROS_INFO("CARTWO");
			  		ROS_INFO("%d, %d, %d",CarOneStartIndex,CarTwoStartIndex,ClearSpaceStartIndex);
			  		//routine();
			  	}
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
	//usleep(10000);
	ros::spin();
	return 0;
}

*/