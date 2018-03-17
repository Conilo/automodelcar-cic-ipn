#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>  
#include <sensor_msgs/image_encodings.h>
#include <ros/console.h>
#include <vector>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

// Global Parameters
bool DEBUG = true;
float MAX_DIST = 1.1;
int AMP_ANG_FIZQ = 60;
int AMP_ANG_FDER = 210;

// Global variables
int start_time, end_time;
int ang;
float dist;
std::vector<int> ROI;
std::vector<float> ranges;
std::vector<std::pair<int,float> > obj;  
std_msgs::Int16 pa;
std_msgs::Float32 pd;     

class ObstacleDetection
{
    ros::NodeHandle nh;
    ros::Publisher laser_pub, pubdist, pubang;	
    ros::Subscriber scan_sub;

public:

ObstacleDetection()
	: nh()
{
    scan_sub = 
        nh.subscribe("/scan", 360, 
            &ObstacleDetection::laser_msg_Callback, this); 

    pubdist = 
        nh.advertise<std_msgs::Float32>("/scan_followc/dist",1);
    pubang = 
        nh.advertise<std_msgs::Int16>("/scan_followc/angle",1); 

    if (DEBUG)
    {
        laser_pub = 
            nh.advertise<sensor_msgs::LaserScan>(
                "/obstacle_detection_scan", 1);
    }
     
}

void laser_msg_Callback(
    const sensor_msgs::LaserScan::ConstPtr& scan)
{
    // Starts counting time
    start_time = cv::getTickCount();

    // Local variables
    float elapsed_time;
    
    bool objFlag = false;
    bool f1 = false;
    bool f2 = false;
    sensor_msgs::LaserScan newscan;

    if (DEBUG)
    {
        // Creates a scan vector array for debug purpuses
        for (float i=0.0;i<=360.0;i++)
        {
            ranges.push_back(std::numeric_limits<float>::infinity()*(i));  
        }
    }

    //Rango
    for (int j=0;j<=60;j++)
        {
            ROI.push_back(j);
            f1=true; 
        }

    if (f1==true)
    {
        for (int k=340;k<=359;k++)
        {
            ROI.push_back(k);
            f2=true;
        }
    }
    else if(f2==true)
    {
        for (int l=61;l<=210;l++)
        {
            ROI.push_back(l);
        }
    }

    int cnt=0;
    for (int m=ROI.front();m<=ROI.back();m++)
     { 
        if(scan->ranges[m]<MAX_DIST)
           {
              ranges[m]=scan->ranges[m];
              obj.push_back(std::make_pair(m,scan->ranges[m]));
              cnt++;
           }
         else 
         {
             if(cnt>7)
             {
                ang=obj.front().first;
                dist=obj.front().second;
                objFlag=true;
                break;
             }
         }
    }
    obj.clear();
    if (objFlag==true)
    {
       ang=obj.front().first;
       dist=obj.front().second;
       pa.data=obj.front().first;
       pd.data=obj.front().second;
       pubang.publish(pa);
       pubdist.publish(pd);
      
       if (ang>300 || ang<=60)
       {ROS_INFO("Front");}
       else if (ang>60 && ang<140)
       {ROS_INFO("Lat");}
       else 
       {ROS_INFO("Back");}	 
    }
    else 
    {
        ang=220;
        dist=0.1;
        pa.data=220;
        pd.data=0.1;
        pubang.publish(pa);
        pubdist.publish(pd);
        newscan.ranges = ranges;
        //newscan.intensities = intensities
        laser_pub.publish(newscan);
        
       // newscan.ranges= ranges;
    }

    // Finish counting time
	end_time = cv::getTickCount();	
    elapsed_time = 
        (end_time - start_time)/cv::getTickFrequency();
    ROS_INFO("Ang: %i     |Dist: %f",ang,dist);
    ROS_INFO("Time elapsed:%f.........end block", elapsed_time);
}
};


int main(int argc,char **argv)
{
    ros::init(argc, argv, "ObstacleDetection");
    ROS_INFO("ObstacleDetection node running...");

    // Get parameters from launch


    ObstacleDetection od;
    ros::spin();
    return 0;
}