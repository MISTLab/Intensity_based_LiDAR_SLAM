//include sensor_msgs pointcloud2
#pragma once
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
// include eigen library
// #include <Eigen/Dense>
#include <eigen3/Eigen/Dense>
#include <omp.h>
#include "image_handler.h"
#include "tic_toc.h"
#include "keyframe.h"
#include <geometry_msgs/Point.h>
#include <deque>
#include "ikd-Tree/ikd_Tree.h"
#define PCL_NO_PRECOMPILE

template <typename Derived>
static void reduceVector(std::vector<Derived> &v, std::vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

class mapOptimization
{
public:
    mapOptimization();
    ~mapOptimization();
    void mapOptimizationCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, const nav_msgs::Odometry::ConstPtr& odom_msg, const sensor_msgs::PointCloud2::ConstPtr& plane_cloud_msg, const sensor_msgs::PointCloud2::ConstPtr& pc_corner_msg);
    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry);
    cv::Mat setMask(int height, int width, int crop);
    void keypoint2uv(const std::vector<cv::KeyPoint>& keypoints, std::vector<cv::Point2f>& uv);
    void extractPointsAndFilterZeroValue(const std::vector<cv::Point2f>& cur_orb_point_2d_uv_,  const pcl::PointCloud<PointType>::Ptr& cloudTrack_, std::vector<cv::Point3f>& cur_out_point3d_, std::vector<uchar>& status_);
    void extractMatchedPoints(std::vector<cv::DMatch> &matches, std::vector<cv::Point3f> &prev_matched_points3d, Eigen::Quaterniond prev_q, Eigen::Vector3d prev_t, std::vector<cv::Point3f> &cur_matched_points3d, Eigen::Quaterniond cur_q, Eigen::Vector3d cur_t, std::vector<cv::Point3f> &prev_out_point3d, std::vector<cv::Point3f> &cur_out_point3d, std_msgs::Header header);
    void extractMatchedPoints(std::vector<cv::DMatch> &matches, std::vector<cv::Point3f> &prev_matched_points3d, std::vector<cv::Point3f> &cur_matched_points3d,  std::vector<cv::Point3f> &prev_out_point3d, std::vector<cv::Point3f> &cur_out_point3d);

    void transformUpdate();
    void transformAssociateToMap();
    void calculateAverageDistance(double &avg_distance, std::vector<cv::Point3f> prev_matched_points3d, Eigen::Quaterniond prev_q, Eigen::Vector3d prev_t, std::vector<cv::Point3f> cur_matched_points3d, Eigen::Quaterniond cur_q, Eigen::Vector3d cur_t);
    void appendLines(visualization_msgs::Marker &line_list, geometry_msgs::Point p1, geometry_msgs::Point p2);
    void initialLineList(visualization_msgs::Marker &line_list, std_msgs::Header header);
    void image_show(std::vector<cv::DMatch> &matches, std::string& detectTime, cv::Mat prev_img, cv::Mat cur_img, std::vector<cv::Point2f> cur_orb_point_2d_uv_, std::vector<cv::Point2f> prev_orb_point_2d_uv_);
    void filterNaNPoints();
    intensity_slam::ImageHandler *image_handler_;
    std::map<size_t, mapProcessor::Keyframe> keyframe_pool_; 
    std::deque<std::shared_ptr<mapProcessor::SlideWindowKeyframe>> keyframe_sliding_window_;
    mapProcessor::Keyframe prev_keyframe, cur_keyframe;

private:
    size_t keyframeId_;
    int IMAGE_WIDTH;
    int IMAGE_HEIGHT;
    int IMAGE_CROP; 
    int NUM_THREADS;
    int NUM_ORB_FEATURES;
    bool HAND_HELD_FLAG;
    int SLIDING_WINDOW_SIZE;
    int GROUND_PLANE_WINDOW_SIZE;
    Eigen::Quaterniond q_wmap_wodom;
    Eigen::Vector3d t_wmap_wodom;
    Eigen::Quaterniond q_wodom_curr;
    Eigen::Vector3d t_wodom_curr;
    ros::Publisher pubOdomAftMappedHighFrec;
    ros::Publisher robot_marker_pub;
    ros::Publisher matched_points_pub;
    std::vector<cv::KeyPoint> prev_keypoints_, cur_keypoints_, cur_keypoints_tmp_;
    std::vector<cv::Point2f>  prev_orb_point_2d_uv_, cur_orb_point_2d_uv_, cur_orb_point_2d_uv_tmp_;
    std::vector<cv::Point3f>  prev_out_point3d_, cur_out_point3d_, cur_out_point3d_tmp_;
    std::vector<uchar> status_;
    cv::Mat MASK;
    cv::Mat prev_descriptors_, cur_descriptors_, cur_descriptors_tmp_;
    cv::Mat prev_keyframe_img, cur_keyframe_img;
    cv::BFMatcher matcher_;
    bool keyframe_flag_;
    double parameters_[7] = {0, 0, 0, 1, 0, 0, 0}; 
    std::vector<cv::Point3f> prev_matched_points3d_, cur_matched_points3d_;
    visualization_msgs::Marker line_list;
    ros::Publisher matched_lines_pub; 
    ros::Publisher matched_keypoints_img_pub, ground_points_pub;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_, corner_voxel_grid_;
    pcl::PointXYZ ground_point_sensor_, ground_point_world_;
    std::mutex map_mutex_;
    KD_TREE<GroundPlanePointType>::Ptr ikdtree, corner_ikdtree_;
    pcl::PointCloud<pcl::PointXYZ> ground_plane_cloud_;
    

    
    
    


    





};

