#pragma once
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/Image.h>
// #include <DBoW2/DBoW2.h>
#include <DBoW3/DBoW3.h>

// #include <opencv2/features2d.hpp>
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
// #include <opencv2/core/eigen.hpp>
// #include <opencv/cv.h>
#include <opencv2/opencv.hpp>

#include "keyframe.h"
#include "parameters.h"
#include "Scancontext.h"
// #include "ouster_ros/ros.h"

class loopClosureHandler
{
private:
    /* data */
    
public:
    loopClosureHandler(/* args */);
    ~loopClosureHandler();
    void keyframeProcessor(loopClosureProcessor::Keyframe keyframe); 
    void keyframeProcessor(loopClosureProcessor::Keyframe keyframe, pcl::PointCloud<PointType>::Ptr cloud_keyPose3D);
    int detectLoop(cv::Mat & image, cv::Mat& descriptors, int frame_index);
    sensor_msgs::ImagePtr cvMat2Image(std_msgs::Header header, cv::Mat & image);
    SCManager scManager;
    


public:   
    DBoW3::Database db_;
    DBoW3::Vocabulary* voc_;
    std::vector<cv::Mat> bow_descriptors_;
    std::map<int, cv::Mat> image_pool_;
    std::map<size_t, loopClosureProcessor::Keyframe> keyframe_pool_; 
    int loop_index_;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeKeyframeNearSearch_;
    std::mutex kdtreeKeyframeNearSearch_mutex_;
    // cv::Mat MASK;
};

