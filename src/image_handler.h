// Modified from https://github.com/TixiaoShan/imaging_lidar_place_recognition
#pragma once
#include "parameters.h"
#include <omp.h>
#include <math.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>

namespace intensity_slam{
class ImageHandler
{
public:

    ros::NodeHandle nh;

    ros::Publisher pub_image;
    ros::Publisher groundPoint_pub;

    cv::Mat image_range;
    cv::Mat image_ambient;
    cv::Mat image_intensity;
    int IMAGE_HEIGHT, IMAGE_WIDTH, NUM_THREADS;
    
    pcl::PointCloud<PointType>::Ptr cloud_track;
    pcl::PointCloud<pcl::PointXYZ>::Ptr GroundPointOut;
    ImageHandler(int height=128, int width=1024, int threadNum=6)
    {
        IMAGE_HEIGHT = height;
        IMAGE_WIDTH = width;
        NUM_THREADS = threadNum;
        cloud_track.reset(new pcl::PointCloud<PointType>());
        cloud_track->resize(IMAGE_HEIGHT * IMAGE_WIDTH);
        GroundPointOut.reset(new pcl::PointCloud<pcl::PointXYZ>);
        groundPoint_pub = nh.advertise<sensor_msgs::PointCloud2>("/ransac_groundPoint",1000);
        
    }

    void groundPlaneExtraction(const sensor_msgs::PointCloud2ConstPtr &cloud_msg){
        GroundPointOut->points.clear();
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_original_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *point_cloud_original_ptr);

        assert((int)point_cloud_original_ptr->size() % IMAGE_HEIGHT * IMAGE_WIDTH == 0);
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_screening_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        #pragma omp parallel for num_threads(NUM_THREADS)
        for (long unsigned int i = 0; i < point_cloud_original_ptr->points.size(); i++)
        {
            
            
            if (point_cloud_original_ptr->points[i].z >= -2.0 && point_cloud_original_ptr->points[i].z <= -0.45)
            {
                point_cloud_screening_ptr->points.push_back(point_cloud_original_ptr->points[i]);
            }
        }
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);
        seg.setAxis(Eigen::Vector3f(0,0,1));
        seg.setEpsAngle(15 *M_PI/180);
        seg.setNumberOfThreads(2*NUM_THREADS);
        seg.setInputCloud (point_cloud_screening_ptr);
        seg.segment (*inliers, *coefficients);
        pcl::PointCloud<pcl::PointXYZ>::Ptr GroundPoint_ptr(new pcl::PointCloud<pcl::PointXYZ>);

        double A = coefficients->values[0];
        double B = coefficients->values[1];
        double C = coefficients->values[2];
        double D = coefficients->values[3];

        Eigen::Vector3f plane_normal(A, B, C);
        Eigen::Vector3f z_normal(0, 0, 1);
        if (plane_normal.dot(z_normal) > cos(15 *M_PI/180)) 
        {
           #pragma omp parallel for num_threads(NUM_THREADS)
            for (long unsigned int i = 0; i < point_cloud_original_ptr->points.size(); i++)
            {
                double X = point_cloud_original_ptr->points[i].x;
                double Y = point_cloud_original_ptr->points[i].y;
                double Z = point_cloud_original_ptr->points[i].z;
                double height = abs(A*X + B*Y + C*Z + D) / sqrt(A*A + B*B + C*C); 

                if ((height <= 0.03) && (point_cloud_original_ptr->points[i].z < -0.0) )
                {
                    GroundPoint_ptr->points.push_back(point_cloud_original_ptr->points[i]);
                }
            }

            GroundPointOut = GroundPoint_ptr;
            if(groundPoint_pub.getNumSubscribers() != 0){
                sensor_msgs::PointCloud2 groundPoint_cloud_msg;
                pcl::toROSMsg(*GroundPoint_ptr, groundPoint_cloud_msg);
                groundPoint_cloud_msg.header = cloud_msg->header;
                groundPoint_pub.publish(groundPoint_cloud_msg);
            }
        }
    }
    void cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
    {       
        pcl::PointCloud<PointOuster>::Ptr laser_cloud(new pcl::PointCloud<PointOuster>());
        pcl::fromROSMsg(*cloud_msg, *laser_cloud);
        assert((int)laser_cloud->size() % IMAGE_HEIGHT * IMAGE_WIDTH == 0);
        image_range = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, cv::Scalar(0));
        image_ambient = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, cv::Scalar(0));
        image_intensity = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, cv::Scalar(0));

        #pragma omp parallel for num_threads(NUM_THREADS)
        for (int u = 0; u < IMAGE_HEIGHT; u++) 
        {
            for (int v = 0; v < IMAGE_WIDTH; v++) 
            {
                const auto& pt = laser_cloud->points[u * IMAGE_WIDTH + v];
                float range = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
                float intensity = pt.intensity;

                intensity = std::min(intensity, 255.0f);
                image_range.at<uint8_t>(u, v) = std::min(range * 20, 255.0f);
                image_intensity.at<uint8_t>(u, v) = intensity;
                PointType* p = &cloud_track->points[u * IMAGE_WIDTH + v];

                if (range >= 0.1)
                {
                    p->x = pt.x;
                    p->y = pt.y;
                    p->z = pt.z;
                    p->intensity = intensity;
                    
                }
                else
                {
                    p->x = p->y = p->z =  0;
                    
                    p->intensity = 0;
                }
            }
        }

    }


};

}//namespace
