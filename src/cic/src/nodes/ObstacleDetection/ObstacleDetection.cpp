#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/console.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Point.h"
#include "cic/Obstacles.h"

typedef std::vector<cv::Point2f> vector_point;
typedef std::vector<vector_point> obstacles;

struct Sort_Y {
    bool operator() (cv::Point pt1, cv::Point pt2) 
    { return (pt1.y < pt2.y); }
} sort_y;

// Global Parameters
bool DEBUG = true;
float RANGE = 1.0;
float INITIAL_RANGE_ANGLE = 300.0;
float END_RANGE_ANGLE = 200.0;

// BDSCAN global parameters
int MIN_OBSTACLE_POINTS = 10;
float MAX_DISTANCE_POINTS = 5.0; 


// Global variables
int start_time, end_time;

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
    int cnt, detected_obstacles;
    float elapsed_time, dist_norm;
    std::vector<float> ROI;
    std::vector<float> ranges;
    std::vector<float> intensities;
    vector_point scan_points, obstacle;
    obstacles detected_obstacles_vector;
    cv::Point2f last_point, current_point;

    // scan/ranges vector for debug purpuses
    sensor_msgs::LaserScan debug_scan;

    // Fill the ranges vector
    for (float i = 0.0; i <= 360.0; i++)
    {
        ranges.push_back(
            std::numeric_limits<float>::infinity());
        intensities.push_back(
            std::numeric_limits<float>::infinity());  
    }

    // Range to be considered
    if (INITIAL_RANGE_ANGLE >= END_RANGE_ANGLE)
    {
        float i = INITIAL_RANGE_ANGLE;;
        while (i<360)
        {
            ROI.push_back(i);
            i += 1.0;
        }
        i = 0.0;
        while (i < END_RANGE_ANGLE)
        {
            ROI.push_back(i);
            i += 1.0;
        }
    }
    else
    {
        float i = INITIAL_RANGE_ANGLE;
        while (i < END_RANGE_ANGLE)
        {
            ROI.push_back(i);
            i += 1.0;
        }
    }

    // Takes points within a certian distance range
    for (std::vector<float>::iterator i = ROI.begin(); 
		 i != ROI.end(); i++)
     { 
        if(scan->ranges[*i] <= RANGE)
           {
                ranges[*i] = scan->ranges[*i];
                intensities[*i] = scan->intensities[*i];
                scan_points.push_back(
                    cv::Point2f(*i, scan->ranges[*i]));
           }
    }

    // BDSCAN variation algorithm
    if (!scan_points.empty())
    {
        cnt = 0;
        last_point = scan_points.front();
	    for (int i = 0; i <= scan_points.size(); ++i)
        {
            current_point = scan_points[i];

            // Unifies the 360 range with 0 range
            if ((current_point.x < 5) && (last_point.x > 300))
            {
                last_point.x = 0;
            }
        
            // Distance between last and current points
            dist_norm = cv::norm(current_point - last_point);
        
            // Verifies if the distance is lower than Epsylon
            if (dist_norm < MAX_DISTANCE_POINTS)
            {
                // Append current point to the object
                obstacle.push_back(current_point);
                cnt += 1;
            }
            else
            {
                // verifies the object's number of points
                if (cnt > MIN_OBSTACLE_POINTS)
                {
                    // append object to object vetor
                    detected_obstacles_vector.push_back(obstacle);
                }
                // Clear the current object points
                obstacle.clear();
                cnt = 0;
            }

            last_point = current_point;
        }
    }

    // Get the obstacles closest vertex
    std::vector<geometry_msgs::Point> obstacle_vertices;
    geometry_msgs::Point vertix;
    if (!detected_obstacles_vector.empty())
    {
        detected_obstacles = detected_obstacles_vector.size();

        for (int obstacle_index = 0; 
             obstacle_index < detected_obstacles; 
             obstacle_index++)
        {
            // Sort each obstacle points
            /*std::sort(
                detected_obstacles_vector[obstacle_index].begin(), 
                detected_obstacles_vector[obstacle_index].end(),
                sort_y);*/
            
            // Take the closest point
            vertix.x = 
                detected_obstacles_vector[obstacle_index].front().x;
            vertix.y = 
                100.0 *detected_obstacles_vector[obstacle_index].front().y;
            vertix.z = 0.0;
            obstacle_vertices.push_back(vertix);
            }

        // Message publication
        cic::Obstacles obstacles_msg;
        obstacles_msg.header.stamp = ros::Time::now();
        obstacles_msg.detected_obstacles = detected_obstacles;
        obstacles_msg.obstacle_vertices = obstacle_vertices;
	    pubMsg.publish(obstacles_msg);
    }

    // Publish debug info
    if (DEBUG)
    {
        debug_scan = *scan;
        debug_scan.header.stamp = ros::Time::now();
        debug_scan.header.frame_id = scan->header.frame_id;
        debug_scan.ranges = ranges;
        debug_scan.intensities = intensities;

        laser_pub.publish(debug_scan);
    }
    
    // Finish counting time
	end_time = cv::getTickCount();
    elapsed_time = 
        (end_time - start_time)/cv::getTickFrequency();

    ROS_INFO("Detected obstacles: %i", detected_obstacles);
    ROS_INFO("Time elapsed:%f....... end block", elapsed_time);
}
};


int main(int argc,char **argv)
{
    ros::init(argc, argv, "ObstacleDetection");
    ROS_INFO("ObstacleDetection node running...");

    // Get parameters from launch
    ros::param::get("/debug_mode", DEBUG);
	ros::param::get("~initial_range_angle", INITIAL_RANGE_ANGLE);
    ros::param::get("~end_range_angle", END_RANGE_ANGLE);
    ros::param::get("~range", RANGE);
    ros::param::get("~min_points", MIN_OBSTACLE_POINTS);
    ros::param::get("~max_distance", MAX_DISTANCE_POINTS);

    ObstacleDetection od;
    ros::spin();
    return 0;
}