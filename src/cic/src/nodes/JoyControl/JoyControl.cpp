#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>
#include <iostream>


float y = 100, j=0;
int x = 0, z=90;
int xmin = -400, xmax = 400;
int zmin = 80, zmax = 100;  //20 a 160

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	j=joy->axes[2];
	y = (j*140)+90;
	z=(int)y;
	//ROS_INFO("%f --- %f",y,j);

   if (joy->buttons[2])   //Velocidad
      x += -50;
   else if (joy->buttons[3])
      x += 50;

   if(joy->buttons[4] || joy->buttons[5] || joy->buttons[6] || joy->buttons[7])
   {
      x = 0;
      z = 100;
   }

   if (x > xmax) x = xmax;
   if (x < xmin) x = xmin;
   if (z > zmax) z = zmax;
   if (z < zmin) z = zmin;
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "JoyControl");
   ros::NodeHandle nh_;
   ros::Subscriber information_joy = nh_.subscribe<sensor_msgs::Joy>("joy", 1, joyCallback);
   ros::Publisher speedpub_ = nh_.advertise<std_msgs::Int16>("/manual_control/speed", 1);
   ros::Publisher steeringpub_ = nh_.advertise<std_msgs::Int16>("/manual_control/steering", 1);
   std_msgs::Int16 speed;
   std_msgs::Int16 steering;
   ros::Rate loop_rate(10);

   while(ros::ok()) {
      speed.data = x;
      speedpub_.publish(speed);
      steering.data = z;
      steeringpub_.publish(steering);
      ros::spinOnce();
      loop_rate.sleep();
   }
}
