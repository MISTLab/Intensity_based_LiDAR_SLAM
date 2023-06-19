#pragma once
#include <opencv2/core/core.hpp>
#include <vector>
#include <map>
#include <eigen3/Eigen/Dense>
#include "parameters.h"

namespace mapProcessor{
struct Keyframe
{
    
    size_t keyframeId;
    pcl::PointCloud<PointType> cloud_track_; 
    pcl::PointCloud<pcl::PointXYZ> ground_cloud_; 
    Eigen::Quaterniond q_map_cur_k_;
    Eigen::Vector3d t_map_cur_k_;


    Keyframe(){}

    Keyframe(   
                size_t keyframeIdTmp,
                pcl::PointCloud<PointType> cloud,
                pcl::PointCloud<pcl::PointXYZ> cloud_ground,
                Eigen::Quaterniond q_map_cur_k,
                Eigen::Vector3d t_map_cur_k):                
                keyframeId{keyframeIdTmp},
                cloud_track_{cloud}, ground_cloud_{cloud_ground}, q_map_cur_k_{q_map_cur_k}, t_map_cur_k_{t_map_cur_k}
                {}
};
struct SlideWindowKeyframe
{
    
    cv::Mat descriptors; // for detecting matches between two keyframes
    cv::Mat image_intensity; // for visualization
    std::vector<cv::Point2f>  orb_point_2d_uv;// for visualization
    Eigen::Quaterniond q_map_cur_tk; // for convert point cloud to map frame
    Eigen::Vector3d t_map_cur_tk; // for convert point cloud to map frame
    std::vector<cv::Point3f>  cur_point3d_wrt_orb_features; // 3D point cloud in lidar frame
    
    SlideWindowKeyframe(){}
    SlideWindowKeyframe(cv::Mat descriptorsTmp, cv::Mat image_intensityTmp, std::vector<cv::Point2f> orb_point_2d_uvTmp, Eigen::Quaterniond q_map_cur_tkTmp, Eigen::Vector3d t_map_cur_tkTmp, std::vector<cv::Point3f> cur_point3d_wrt_orb_featuresTmp):
        descriptors{descriptorsTmp}, image_intensity{image_intensityTmp}, orb_point_2d_uv{orb_point_2d_uvTmp}, q_map_cur_tk{q_map_cur_tkTmp}, t_map_cur_tk{t_map_cur_tkTmp}, cur_point3d_wrt_orb_features{cur_point3d_wrt_orb_featuresTmp}
        {}

};    
}


namespace loopClosureProcessor{
struct Keyframe
{
    double time;
    size_t keyframeId;
    cv::Mat image;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::vector<cv::Point3f> featureMatched3DPoints;
    Eigen::Vector3d cur_position_;
    Eigen::Vector3d prev_position_;
    Eigen::Matrix3d cur_rotation_;
    Eigen::Matrix3d prev_rotation_;
    pcl::PointCloud<PointType> cloud_track_; 

    Keyframe(){}

    Keyframe(   double timeTmp,
                size_t keyframeIdTmp,
                cv::Mat imageTmp,
                std::vector<cv::KeyPoint> keypointsTmp,
                cv::Mat descriptorsTmp,
                std::vector<cv::Point3f> featureMatched3DPointsTmp,
                Eigen::Vector3d cur_position_Tmp,
                Eigen::Vector3d prev_position_Tmp,
                Eigen::Matrix3d cur_rotation_Tmp,
                Eigen::Matrix3d prev_rotation_Tmp,
                pcl::PointCloud<PointType> cloud):                
                time{timeTmp}, keyframeId{keyframeIdTmp}, image{imageTmp}, keypoints{keypointsTmp}, descriptors{descriptorsTmp}, featureMatched3DPoints{featureMatched3DPointsTmp}, cur_position_{cur_position_Tmp}, prev_position_{prev_position_Tmp},  cur_rotation_{cur_rotation_Tmp}, prev_rotation_{prev_rotation_Tmp},
                cloud_track_{cloud}
                {}



};

struct FactorGraphNode
{
    double time;
    size_t keyframeId;
    Eigen::Vector3d cur_position_;
    Eigen::Vector3d prev_position_;
    Eigen::Matrix3d cur_rotation_;
    Eigen::Matrix3d prev_rotation_;

    // construction function
    FactorGraphNode(    double timeTmp,
                        size_t keyframeIdTmp,
                        Eigen::Vector3d cur_position_Tmp,
                        Eigen::Vector3d prev_position_Tmp,
                        Eigen::Matrix3d cur_rotation_Tmp,
                        Eigen::Matrix3d prev_rotation_Tmp):

                        time{timeTmp}, keyframeId{keyframeIdTmp}, cur_position_{cur_position_Tmp},prev_position_{prev_position_Tmp},  cur_rotation_{cur_rotation_Tmp}, prev_rotation_{prev_rotation_Tmp}
                        {}
    FactorGraphNode(){}




};


}