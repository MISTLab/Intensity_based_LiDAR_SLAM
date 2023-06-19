#include "intensity_feature_tracker.h"
#include "image_handler.h"
#include "tic_toc.h"
#include <opencv2/opencv.hpp>  
#include <iostream>
#include <omp.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <ros/ros.h>

intensity_slam::ImageHandler *image_handler;
intensity_slam::feature_tracker *feature_tracker;
std::mutex mapOdomMutex;
void map_odom_callback(const nav_msgs::Odometry::ConstPtr &mapOdometry){  
    mapOdomMutex.lock();        
    feature_tracker->mapOdomQueue_.push(mapOdometry); 
    mapOdomMutex.unlock(); 
}

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    TicToc covert_pointcloud2Intensity_time;

    #pragma omp parallel sections num_threads(4)
    {
        #pragma omp section
        image_handler->cloud_handler(cloud_msg); //5ms
    }
    ros::Time cloud_time = cloud_msg->header.stamp; 
    TicToc detectFeatures_time; 
    feature_tracker->detectfeatures(cloud_time, image_handler->image_intensity, image_handler->cloud_track, image_handler->GroundPointOut);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "intensity_feature_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    feature_tracker = new intensity_slam::feature_tracker(n);
    
    // Load params
    std::cout << "read parameters" << std::endl; 
    feature_tracker->readParameters();

    image_handler = new intensity_slam::ImageHandler(feature_tracker->IMAGE_HEIGHT, 
                                                     feature_tracker->IMAGE_WIDTH, 
                                                     feature_tracker->NUM_THREADS);
    if(true){
        std::cout << "start subscribe topic: "<< feature_tracker->CLOUD_TOPIC << std::endl;
        ros::Subscriber sub_point = n.subscribe(feature_tracker->CLOUD_TOPIC, 1, cloud_callback);
        ros::Subscriber sub_map_path = n.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100, map_odom_callback); 
        ros::MultiThreadedSpinner spinner(8);
        spinner.spin();
    }

    return 0;
}

