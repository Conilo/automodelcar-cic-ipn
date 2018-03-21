#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/console.h>
#include <vector>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include "cic/Obstacles.h"

typedef std::vector<cv::Point> vector_point;
typedef std::vector<vector_point> obstacles;

// Global Parameters
bool DEBUG = true;
float MAX_DIST = 1.0;
int INITIAL_RANGE_ANGLE = 300;
int END_RANGE_ANGLE = 200;
int MIN_POINTS = 5;

//Global constants
float MAX_DIST_ALLOWED = 5.0;

// Global variables
int start_time, end_time;
int angle;
float distance;    

class ObstacleDetection
{
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Publisher pubMsg, laser_pub;

public:

ObstacleDetection()
	: nh()
{
    scan_sub = 
        nh.subscribe("/scan", 360, 
            &ObstacleDetection::laser_msg_Callback, this); 

    pubMsg = nh.advertise<cic::Obstacles>(
        "/obstacle_detection", 1);

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
    bool detected_object = false;
    std::vector<int> ROI;
    std::vector<float> ranges;
    std::vector<float> intensities;
    vector_point scan_points, obstacle;
    obstacles detected_obstacles;

    // scan/ranges vector for debug purpuses
    sensor_msgs::LaserScan debug_scan;

    // Fill the ranges vector
    for (float i=0.0; i<=360.0; i++)
    {
        ranges.push_back(
            std::numeric_limits<float>::infinity());
        intensities.push_back(
            std::numeric_limits<float>::infinity());  
    }

    // Range to be considered
    if (INITIAL_RANGE_ANGLE >= END_RANGE_ANGLE)
    {
        int i = INITIAL_RANGE_ANGLE;;
        while (i<360)
        {
            ROI.push_back(i);
            i += 1;
        }
        i = 0;
        while (i < END_RANGE_ANGLE)
        {
            ROI.push_back(i);
            i += 1;
        }
    }
    else
    {
        int i = INITIAL_RANGE_ANGLE;
        while (i < END_RANGE_ANGLE)
        {
            ROI.push_back(i);
            i += 1;
        }
    }

    // Takes only points within a certian distance 
    // (< MAX_DIST)
    for (std::vector<int>::iterator i=ROI.begin(); 
		 i != ROI.end(); ++i)
     { 
        if(scan->ranges[*i] <= MAX_DIST)
           {
              ranges[*i] = scan->ranges[*i];
              intensities[*i] = scan->intensities[*i];
              scan_points.push_back(
                  cv::Point(*i, scan->ranges[*i]));
                  
           }
    }

    // BDSCAN variation algorithm
    cv::Point last_point = scan_points.front();
    int cnt = 0;

	for (int i = 0; i <= scan_points.size(); ++i)
    {
        cv::Point current_point = scan_points[i];

        // Unifies the 360 range with 0 range
        if ((current_point.x < 5) && (last_point.x > 300))
        {
            last_point.x = 0;
        }
        
        float dist_norm = cv::norm(current_point - last_point);
        
        if (dist_norm < MAX_DIST_ALLOWED)
        {
            obstacle.push_back(current_point);
            cnt += 1;
        }
        else
        {
            if (cnt > MIN_POINTS)
            {
                detected_obstacles.push_back(obstacle);
            }
            obstacle.clear();
            cnt = 0;
        }
        last_point = current_point;
    }

    ROS_INFO("Detected obstacles: %lu", detected_obstacles.size());	

    // Message publication
    cic::Obstacles obstacles_msg;
    obstacles_msg.header.stamp = ros::Time::now();
	obstacles_msg.angle = angle;
	obstacles_msg.distance = distance;
	pubMsg.publish(obstacles_msg);

    // Publish debug info
    if (DEBUG)
    {
        debug_scan = *scan;
        debug_scan.header.stamp = ros::Time::now();
        debug_scan.header.frame_id = scan->header.frame_id;
        debug_scan.ranges = ranges;
        debug_scan.intensities = intensities;

        laser_pub.publish(debug_scan);

        ROS_INFO("Angle: %i  |Distance: %f", 
                 angle, 
                 distance);
    }
    
    // Finish counting time
	end_time = cv::getTickCount();
    elapsed_time = 
        (end_time - start_time)/cv::getTickFrequency();
    ROS_INFO("Time elapsed:%f....... end block", elapsed_time);
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