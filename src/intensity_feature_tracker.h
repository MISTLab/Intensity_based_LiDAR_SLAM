#pragma once
#include "parameters.h"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "tic_toc.h"
#include <omp.h>
#include <pcl/visualization/cloud_viewer.h>
#include "keyframe.h"
#include "loop_closure_handler.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/nonlinear/ISAM2.h>

#include <deque>
#include <queue>

#include <iostream>
#include <fstream>
#include <stdlib.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>


namespace intensity_slam{

class feature_tracker{
public:
    feature_tracker(ros::NodeHandle& n);
    ~feature_tracker();
    void readParameters();
    void detectfeatures(ros::Time &time, 
                        const cv::Mat &image_intensity, 
                        const pcl::PointCloud<PointType>::Ptr cloud_track,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr groundPoint_);
    void detectcorners( ros::Time &time, 
                        const cv::Mat &image_intensity, 
                        const pcl::PointCloud<PointType>::Ptr cloud_track);
    void setMask();
    void keypoint2uv();
    std::vector<cv::Point2f> keypoint2uv(std::vector<cv::KeyPoint> cur_keypoints);
    void extractPointsAndFilterZeroValue();
    void extractPointsAndFilterZeroValue(std::vector<cv::Point2f> cur_orb_point_2d_uv, pcl::PointCloud<pcl::PointXYZI>::Ptr cloudTrack, std::vector<cv::Point3f> &cur_out_point3d, std::vector<uchar> &status);

    void image_show(std::vector<cv::DMatch> &matches, std::string& detectTime);
    void image_show(std::vector<cv::DMatch> &matches, std::string& detectTime, cv::Mat prev_img_, cv::Mat cur_img_, std::vector<cv::Point2f> cur_orb_point_2d_uv_, std::vector<cv::Point2f> prev_orb_point_2d_uv_);
    void extractMatchedPoints(std::vector<cv::DMatch> &matches, std::vector<cv::Point3f> &prev_points, std::vector<cv::Point3f> &cur_points, std::vector<cv::Point3f> &prev_out_point3d, std::vector<cv::Point3f> &cur_out_point3d);
    Eigen::Matrix4d p2p_calculateRandT(std::vector<cv::Point3f> &src_cloud, std::vector<cv::Point3f> &dst_cloud);
    void tfBroadcast();
    void getKeyframe(double time, std::vector<cv::Point3f> &featurMatchedpoints);
    void loopClosureThread();
    void mapOdomHandle(); 
    void factorGraphThread();
    void icpThread(loopClosureProcessor::Keyframe keyframeNew); 
    void updatePoses();
    void writeTF2file(ros::Time &time); 
    void getSubmapOfhistory(pcl::PointCloud<PointType>::Ptr cloud_submap); 
    void tranformCurrentScanToMap(pcl::PointCloud<PointType>::Ptr cloud_current_scan, size_t index, loopClosureProcessor::Keyframe keyframe_new); 
    Eigen::Matrix4d getTransformMatrix(size_t index);
     

    std::string PROJECT_NAME;
    std::string CLOUD_TOPIC;
    int IMAGE_WIDTH;
    int IMAGE_HEIGHT;
    int IMAGE_CROP;
    int USE_ORB;
    int NUM_ORB_FEATURES;
    double SKIP_TIME;
    int NUM_THREADS;
    bool HAND_HELD_FLAG;
    bool USE_TEASER; 
    bool USE_PNPRANSAC; 
    bool USE_ICP; 
    bool USE_CROP;
    bool USE_DOWNSAMPLE; 
    double CROP_SIZE; 
    double VOXEL_SIZE; 
    double FITNESS_SCORE; 
    double KEYFRAME_TIME_INTERVAL, KEYFRAME_DISTANCE_INTERVAL;

    cv::Mat MASK;
    std::vector<cv::Point2f> cur_orb_point_2d_uv_, prev_orb_point_2d_uv_;
    pcl::PointCloud<PointType>::Ptr cloudTrack_;
    pcl::PointCloud<PointType> cur_cloud_, prev_cloud_;
    pcl::PointCloud<pcl::PointXYZ> ground_cloud_;
    std::vector<uchar> status_;
    std::vector<cv::Point3f> cur_out_point3d_, prev_out_point3d_;
    double first_frame_time_; 
    ros::Time time_stamp_; 
    int frequency_;
    size_t keyframeId_; 
    loopClosureHandler loopClosureProcessor_; 
    std::thread icpProcessThread_; 
    bool getLoop_; 
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D_;
    pcl::PointCloud<PointXYZIRPYT>::Ptr cloudKeyPoses6D_;
    loopClosureProcessor::Keyframe keyframe_;
    // std::deque<loopClosureProcessor::FactorGraphNode> factorGraphNode_; 
    // std::deque<loopClosureProcessor::Keyframe> keyframesQueue_; 

    std::deque<std::shared_ptr<loopClosureProcessor::FactorGraphNode>> factorGraphNode_;
    std::deque<std::shared_ptr<loopClosureProcessor::Keyframe>> keyframesQueue_; 
    std::queue<nav_msgs::Odometry::ConstPtr> mapOdomQueue_; 
    ros::Publisher pubLaserOdometry_;
    ros::Publisher matched_keypoints_img_pub_front_end; 
    

    // std::deque<std::shared_ptr<std::string>> mavros_to_batman_msg_;
    pcl::PointCloud<PointType>::Ptr current_scan_;
    pcl::PointCloud<PointType>::Ptr loop_scan_;
    bool skip_intensity_; 
    double feature_extraction_time_;
    double scan_matching_time_;






private:
    ros::NodeHandle nh_;
    ros::Publisher pubKeyPoses_;
    ros::Publisher pubLoopScan_, pubCurrentScan_;
    cv::Mat prev_img_, cur_img_;//intensity image
    std::vector<cv::KeyPoint> prev_keypoints_, cur_keypoints_;
    cv::Mat prev_descriptors_, cur_descriptors_;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    Eigen::Matrix4d T_s2s_, T_s2m_, T_s2pgo_;//T_s2s_: local scan to scan transform R+T; T_s2m_: global transformation matrix; T_s2pgo_: global transformation matrix after pgo
    Eigen::Vector3d cur_position_, prev_position_;
    Eigen::Matrix3d cur_rotation_, prev_rotation_;
    std::vector<std::pair<int, int>> corres_;

    gtsam::NonlinearFactorGraph gtSAMgraph_;
    gtsam::Values initialEstimate_;
    gtsam::Values optimizedEstimate_;
    gtsam::ISAM2 *isam_;
    gtsam::Values isamCurrentEstimate_;

    gtsam::noiseModel::Diagonal::shared_ptr priorNoise_;
    gtsam::noiseModel::Diagonal::shared_ptr odometryNoise_;
    gtsam::noiseModel::Diagonal::shared_ptr constraintNoise_;
    gtsam::noiseModel::Base::shared_ptr robustNoiseModel_;
    std::mutex factorMutex;

    Eigen::Quaterniond graph_prev_rotation_;
    Eigen::Vector3d graph_prev_translation_;
    int loop_index_kdtree_;
    bool skip_flag_; 
    double pgo_time_; 
    double loop_time_;
    
    


};
}