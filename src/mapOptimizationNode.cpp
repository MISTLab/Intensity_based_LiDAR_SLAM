#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <ros/ros.h>
#include <ceres/ceres.h>
#include "lidarFeaturePointsFunction.hpp"
#include "mapOptimization.hpp"
#include <iostream>
#include <omp.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "map_optimization");
    ros::NodeHandle nh;
    std::string CLOUD_TOPIC; // full cloud topic name
    nh.getParam("/intensity_feature_tracker/cloud_topic", CLOUD_TOPIC);     
    mapOptimization mapOptimization_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, CLOUD_TOPIC, 20);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/laser_odom_to_init", 20);
    message_filters::Subscriber<sensor_msgs::PointCloud2> plane_sub(nh, "/laser_cloud_less_flat", 20);
    message_filters::Subscriber<sensor_msgs::PointCloud2> plane_search_sub(nh, "/laser_cloud_less_sharp", 20); 
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, odom_sub, plane_sub, plane_search_sub);
    sync.registerCallback(boost::bind(&mapOptimization::mapOptimizationCallback, &mapOptimization_, _1, _2, _3, _4)); 
    ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 20, &mapOptimization::laserOdometryHandler, &mapOptimization_); 
    ros::Publisher pub_aft_mapped_to_init = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);
    ros::MultiThreadedSpinner spinner(8);
    spinner.spin(); 

    return 0;
}