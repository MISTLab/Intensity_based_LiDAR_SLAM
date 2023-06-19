#include "intensity_feature_tracker.h"
#include "tic_toc.h"
#include <tf2_ros/transform_broadcaster.h>
#include <ceres/ceres.h>
#include "lidarFeaturePointsFunction.hpp"




using namespace intensity_slam;



template <typename Derived>
static void reduceVector(std::vector<Derived> &v, std::vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

feature_tracker::feature_tracker(ros::NodeHandle &n)
                    :nh_(n)                
{   
    pubKeyPoses_ = nh_.advertise<sensor_msgs::PointCloud2>("/key_pose_origin", 2);
    pubLoopScan_ = nh_.advertise<sensor_msgs::PointCloud2>("/loop_scan", 2);
    pubCurrentScan_ = nh_.advertise<sensor_msgs::PointCloud2>("/current_scan", 2);
    T_s2s_ = Eigen::Matrix4d::Identity();
    T_s2m_ = Eigen::Matrix4d::Identity();
    T_s2pgo_ = Eigen::Matrix4d::Identity();
    prev_position_ << 0.0, 0.0, 0.0;
    prev_rotation_ <<   0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0;
    keyframeId_ = 0; 
    cloudKeyPoses3D_.reset(new pcl::PointCloud<PointType>());
    cloudKeyPoses6D_.reset(new pcl::PointCloud<PointTypePose>());

    gtsam::Vector Vector6(6);
    Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
    priorNoise_ = gtsam::noiseModel::Diagonal::Variances(Vector6);
    odometryNoise_ = gtsam::noiseModel::Diagonal::Variances(Vector6);
    gtsam::Vector Vector6_loop(6);
    Vector6_loop << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    constraintNoise_ = gtsam::noiseModel::Diagonal::Variances(Vector6_loop);

    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam_ = new gtsam::ISAM2(parameters);
    getLoop_ = false; 
    pubLaserOdometry_ = nh_.advertise<nav_msgs::Odometry>("/laser_odom_to_init_intensity", 100);
    matched_keypoints_img_pub_front_end = nh_.advertise<sensor_msgs::Image>("/front_matched_keypoints_img", 10);
    current_scan_.reset(new pcl::PointCloud<PointType>());
    loop_scan_.reset(new pcl::PointCloud<PointType>());

    std::string PROJECT_NAME("intensity_feature_tracker");
    std::string pkg_path = ros::package::getPath(PROJECT_NAME);
    skip_flag_ = false;
    skip_intensity_ = false; 
    
}

feature_tracker::~feature_tracker(){}

void feature_tracker::detectcorners( ros::Time &time, 
                        const cv::Mat &image_intensity, 
                        const pcl::PointCloud<PointType>::Ptr cloud_track)
{

    //



}
void feature_tracker::writeTF2file(ros::Time &time){
    // tf2_ros::Buffer tfBuffer;
    // tf2_ros::TransformListener tfListener(tfBuffer);
    // geometry_msgs::TransformStamped transformStamped;
    // try{
    //     transformStamped = tfBuffer.lookupTransform("map","base_link",time);
    //     if(out_gtposes_.is_open()){
    //         out_gtposes_    << std::fixed << time.toSec() << " " 
    //                         << transformStamped.transform.translation.x << " " 
    //                         << transformStamped.transform.translation.y << " " 
    //                         << transformStamped.transform.translation.z << " " 
    //                         << transformStamped.transform.rotation.x << " " 
    //                         << transformStamped.transform.rotation.y << " " 
    //                         << transformStamped.transform.rotation.z << " " 
    //                         << transformStamped.transform.rotation.w << "\n"; 
                        
    //     }
    //     // transformStamped.transform.rotation.w; 
    //     // time.toSec();
    // }
    // catch(tf2::TransformException &ex){
    //     ROS_ERROR("%s", ex.what());
    //     //return false;
    // }
    
}
void feature_tracker::updatePoses(){
    gtsam::Pose3 tempEstimate;
    
    for (size_t i = 0; i < isamCurrentEstimate_.size(); ++i){
        const std::lock_guard<std::mutex> lock(factorMutex);
        
        tempEstimate = isamCurrentEstimate_.at<gtsam::Pose3>(i);

        cloudKeyPoses3D_->points[i].x = tempEstimate.translation().x(); 
        cloudKeyPoses3D_->points[i].y = tempEstimate.translation().y();
        cloudKeyPoses3D_->points[i].z = tempEstimate.translation().z();

        cloudKeyPoses6D_->points[i].x = tempEstimate.translation().x();
        cloudKeyPoses6D_->points[i].y = tempEstimate.translation().y();
        cloudKeyPoses6D_->points[i].z = tempEstimate.translation().z();
        cloudKeyPoses6D_->points[i].roll = tempEstimate.rotation().roll();
        cloudKeyPoses6D_->points[i].pitch = tempEstimate.rotation().pitch();
        cloudKeyPoses6D_->points[i].yaw = tempEstimate.rotation().yaw();
        

    }

    ROS_DEBUG("updating poses: %li", isamCurrentEstimate_.size());
    {
        const std::lock_guard<std::mutex> lock(factorMutex);
        getLoop_ = false; 
    }  
    
    
}
void feature_tracker::icpThread(loopClosureProcessor::Keyframe keyframeNew){
    std::cout << "\n\nicp thread," <<loopClosureProcessor_.loop_index_ << ", keyframe id:" << keyframeNew.keyframeId << std::endl; 

}

Eigen::Matrix4d feature_tracker::getTransformMatrix(size_t index){
    std::lock_guard<std::mutex> lock(factorMutex);
    ROS_INFO("getTransformMatrix, index: %li, size: %li", index, cloudKeyPoses6D_->points.size());
    auto p_tmp = cloudKeyPoses6D_->points[index];
    Eigen::Matrix4d T_tmp =  Eigen::Matrix4d::Identity();
    gtsam::Rot3 rot_temp = gtsam::Rot3::RzRyRx(p_tmp.yaw, p_tmp.pitch, p_tmp.roll);
    Eigen::Matrix3d R = rot_temp.toQuaternion().normalized().toRotationMatrix();// 3*3
    Eigen::Vector3d T(p_tmp.x, p_tmp.y, p_tmp.z);//3*1
    T_tmp.block(0,0,3,3) = R; 
    T_tmp.block(0,3,3,1) = T; 
    T_tmp.block(3,0,1,4) << 0, 0, 0, 1;
    return T_tmp;

}
void feature_tracker::tranformCurrentScanToMap(pcl::PointCloud<PointType>::Ptr cloud_current_scan, size_t index, loopClosureProcessor::Keyframe keyframe_new){
    
    Eigen::Matrix4d T_tmp = getTransformMatrix(index);
    std::cout << "T_tmp:"<< T_tmp << std::endl;
    pcl::transformPointCloud(keyframe_new.cloud_track_, *cloud_current_scan, T_tmp);

}

void feature_tracker::getSubmapOfhistory(pcl::PointCloud<PointType>::Ptr cloud_submap){
    size_t submap_window_size = 1;
    size_t i; 

    if(loopClosureProcessor_.loop_index_ < (int)submap_window_size){
        i = 0;
    }
    else{
        i = loopClosureProcessor_.loop_index_ - submap_window_size;
    }
    for(; i < loopClosureProcessor_.loop_index_ + submap_window_size + 1; i++){
        if(i < loopClosureProcessor_.keyframe_pool_.size()){
            Eigen::Matrix4d T_tmp =  getTransformMatrix(i);
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(loopClosureProcessor_.keyframe_pool_[i].cloud_track_, *cloudOut, T_tmp);
            *cloud_submap += *cloudOut;
        }
        
    }

}



void feature_tracker::loopClosureThread(){
    ros::Rate rate(10);
    while (ros::ok()){  
        size_t dequeSize = 0;
        TicToc loop_tic;  
        {
            const std::lock_guard<std::mutex> lock(factorMutex);
            dequeSize = keyframesQueue_.size();
        }

        if(dequeSize > 0){
            std::shared_ptr<loopClosureProcessor::Keyframe> keyframeNewArriveTmp;
            loopClosureProcessor::Keyframe keyframeNewArrive;
            {
                const std::lock_guard<std::mutex> lock(factorMutex);
                keyframeNewArriveTmp = keyframesQueue_.front();
                keyframesQueue_.pop_front();
            }
            keyframeNewArrive = *keyframeNewArriveTmp;
            loopClosureProcessor_.keyframeProcessor(keyframeNewArrive);

            if(loopClosureProcessor_.loop_index_ >= 0){                
                if(USE_ICP){
                    pcl::IterativeClosestPoint<PointType, PointType> icp;
                    icp.setMaxCorrespondenceDistance(100);
                    icp.setMaximumIterations(100);
                    icp.setTransformationEpsilon(1e-6);
                    icp.setEuclideanFitnessEpsilon(1e-6);
                    icp.setRANSACIterations(0);
                    
                    pcl::CropBox<PointType> crop;
                    pcl::VoxelGrid<PointType> vf_scan;
                    crop.setNegative(false); 
                    crop.setMin(Eigen::Vector4f(-CROP_SIZE, -CROP_SIZE, -CROP_SIZE, 1.0));
                    crop.setMax(Eigen::Vector4f(CROP_SIZE, CROP_SIZE, CROP_SIZE, 1.0));

                    vf_scan.setLeafSize(VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
                    current_scan_->clear();
                    loop_scan_->clear();

                    {
                        ROS_INFO("current id: %li, size of keypose3d: %li", keyframeNewArrive.keyframeId, cloudKeyPoses3D_->points.size());
                        for(int i = 0; i < 40 && keyframeNewArrive.keyframeId>= cloudKeyPoses3D_->points.size(); i++){
                            
                            std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        }
                        if(keyframeNewArrive.keyframeId>= cloudKeyPoses3D_->points.size()) {
                            ROS_ERROR("factor node is not updated in the end! return!");
                            continue;
                        }
                        auto t_display = getTransformMatrix(keyframeNewArrive.keyframeId);
                        std::cout << "original pose:" << t_display << std::endl;
                        

                    }
                    tranformCurrentScanToMap(current_scan_, keyframeNewArrive.keyframeId, keyframeNewArrive);     
                    getSubmapOfhistory(loop_scan_);
                    if(loop_scan_->size() == 0){
                        ROS_ERROR("loop_scan_ is empty");
                        continue;
                    }
                    ROS_INFO("loop_scan_ size: %li", loop_scan_->size());
                    ROS_INFO("current_scan_ size: %li", current_scan_->size());

                    // Remove NaNs
                    std::vector<int> idx;
                    current_scan_->is_dense = false;
                    pcl::removeNaNFromPointCloud(*current_scan_, *current_scan_, idx);
                    
                    loop_scan_->is_dense = false;
                    pcl::removeNaNFromPointCloud(*loop_scan_, *loop_scan_, idx);

                    if (USE_CROP) {
                        crop.setInputCloud(current_scan_);
                        crop.filter(*current_scan_);
                    }
                    if (USE_DOWNSAMPLE) {
                        vf_scan.setInputCloud(current_scan_);
                        vf_scan.filter(*current_scan_);
                    }

                    // Crop Box Filter for loop cloud
                    if (USE_CROP) {
                        crop.setInputCloud(loop_scan_);
                        crop.filter(*loop_scan_);
                    }
                    if (USE_DOWNSAMPLE) {
                        vf_scan.setInputCloud(loop_scan_);
                        vf_scan.filter(*loop_scan_);
                    }
                    // publish loop_scan_ and current_scan_
                    sensor_msgs::PointCloud2 cloudMsgTemp;
                    pcl::toROSMsg(*loop_scan_, cloudMsgTemp);
                    cloudMsgTemp.header.stamp = ros::Time().now();
                    cloudMsgTemp.header.frame_id = "map";
                    pubLoopScan_.publish(cloudMsgTemp);
                    
                    pcl::toROSMsg(*current_scan_, cloudMsgTemp);
                    cloudMsgTemp.header.stamp = ros::Time().now();
                    cloudMsgTemp.header.frame_id = "map";
                    pubCurrentScan_.publish(cloudMsgTemp);
                    




                    ROS_INFO("loop_scan_DS size: %li", loop_scan_->size());
                    ROS_INFO("current_scan_DS size: %li", current_scan_->size());


                    // Align clouds
                    if(loop_scan_->points.size()>10 && current_scan_->points.size()>10){
                        icp.setInputSource(current_scan_);
                        icp.setInputTarget(loop_scan_);
                        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
                        icp.align(*unused_result);
                        
                        
                        if (icp.hasConverged() != false){
                            std::cout << "\n icp is converged and fitnessScore is: " << icp.getFitnessScore();
                        }
                        else{
                            std::cout << "\n icp is not converged";
                        }
                        
                        if (icp.hasConverged() != false && icp.getFitnessScore() <= FITNESS_SCORE){
                            // add between factor
                            Eigen::Matrix4d T_before, T_after, T_temp, T_cur2map, T_loop2map, T_cur2map_gt;
                            
                            auto T_icp_cur2loop_bias = icp.getFinalTransformation();
                            std::cout << "\n icp result:\n" << T_icp_cur2loop_bias << std::endl;
                            T_temp = T_icp_cur2loop_bias.cast<double>();
                            
                            T_cur2map = getTransformMatrix(keyframeNewArrive.keyframeId);
                            T_loop2map = getTransformMatrix(loopClosureProcessor_.loop_index_);
                            T_cur2map_gt = T_temp * T_cur2map;

                            Eigen::Matrix3d rot_cur = T_cur2map_gt.block(0,0,3,3);
                            Eigen::Quaterniond q_cur(rot_cur);
                            Eigen::Vector3d t_cur(T_cur2map_gt.block(0,3,3,1));                            
                            Eigen::Vector3d cur_euler = q_cur.toRotationMatrix().eulerAngles(2, 1, 0);
                            
                            gtsam::Pose3 pose_from = gtsam::Pose3(gtsam::Rot3::RzRyRx(cur_euler[0], cur_euler[1], cur_euler[2]), gtsam::Point3(t_cur[0], t_cur[1], t_cur[2]));
                            Eigen::Matrix3d after_rotation;

                            Eigen::Matrix3d rot_loop = T_loop2map.block(0,0,3,3);
                            Eigen::Quaterniond q_loop(rot_loop);        
                            Eigen::Vector3d t_loop(T_loop2map.block(0,3,3,1));                       
                            Eigen::Vector3d loop_euler = q_loop.toRotationMatrix().eulerAngles(2, 1, 0);

                            gtsam::Pose3 pose_to = gtsam::Pose3(gtsam::Rot3::RzRyRx(loop_euler[0], loop_euler[1], loop_euler[2]), gtsam::Point3(  t_loop[0], t_loop[1], t_loop[2]));
                            {
                                
                                gtsam::Vector Vector6_loop(6);
                                float noiseScore = icp.getFitnessScore();
                                Vector6_loop << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
                                constraintNoise_ = gtsam::noiseModel::Diagonal::Variances(Vector6_loop);
                                
                                
                                robustNoiseModel_ = gtsam::noiseModel::Robust::Create(
                                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), 
                                    gtsam::noiseModel::Diagonal::Variances(Vector6_loop)
                                ); 

                                const std::lock_guard<std::mutex> lock(factorMutex);                                
                                gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(                            
                                        keyframeNewArrive.keyframeId,   
                                        loopClosureProcessor_.loop_index_,                         
                                        pose_from.between(pose_to),
                                        constraintNoise_));// TODO: check pose3
                                ROS_INFO("icp method: graph between factor of loop added");                                
                                getLoop_ = true;                                

                            }
                            
                        }
                    }
                    ROS_INFO("time of ICP: %i ms", (int)loop_tic.toc());

                }  

            } 

            if(getLoop_ == true){
                updatePoses();
                sensor_msgs::PointCloud2 cloudMsgTemp;
                pcl::toROSMsg(*cloudKeyPoses3D_, cloudMsgTemp);
                cloudMsgTemp.header.stamp = ros::Time().now();
                cloudMsgTemp.header.frame_id = "map";
                pubKeyPoses_.publish(cloudMsgTemp);
            }

        }
        ROS_INFO("loop time: %lf ms", loop_tic.toc());
        loop_time_ = loop_tic.toc();
        
        rate.sleep();

        
        
    }

}
void feature_tracker::mapOdomHandle(){
    
    while (!mapOdomQueue_.empty() && !factorGraphNode_.empty() && mapOdomQueue_.back()->header.stamp.toSec() >= factorGraphNode_.front()->time)
    {
        factorMutex.lock();
        while(!mapOdomQueue_.empty() && mapOdomQueue_.front()->header.stamp.toSec() < factorGraphNode_.front()->time){  
            mapOdomQueue_.pop();             
        }
        factorMutex.unlock();
        
        if (mapOdomQueue_.empty())
        {
            continue;
        }
        // adding factor
        loopClosureProcessor::FactorGraphNode factorNewArrive;
        std::shared_ptr<loopClosureProcessor::FactorGraphNode> factorNewArriveTmp;
        // lock
        {
            const std::lock_guard<std::mutex> lock(factorMutex);
            factorNewArriveTmp = factorGraphNode_.front();
            factorGraphNode_.pop_front();
        }
        factorNewArrive = *factorNewArriveTmp; 

        factorMutex.lock();
        auto odom_curr = mapOdomQueue_.front(); 
        mapOdomQueue_.pop(); 
        factorMutex.unlock();

        Eigen::Quaterniond q_tmp;
        q_tmp.x() = odom_curr->pose.pose.orientation.x;
        q_tmp.y() = odom_curr->pose.pose.orientation.y;
        q_tmp.z() = odom_curr->pose.pose.orientation.z;
        q_tmp.w() = odom_curr->pose.pose.orientation.w;
        Eigen::Vector3d euler = q_tmp.toRotationMatrix().eulerAngles(2, 1, 0); 

        Eigen::Vector3d t_wodom_curr;
        t_wodom_curr.x() = odom_curr->pose.pose.position.x;
        t_wodom_curr.y() = odom_curr->pose.pose.position.y;
        t_wodom_curr.z() = odom_curr->pose.pose.position.z;

        
        
        
        if(cloudKeyPoses3D_->points.empty()){// add prior factor               
            const std::lock_guard<std::mutex> lock(factorMutex);
            gtSAMgraph_.add(gtsam::PriorFactor<gtsam::Pose3>(factorNewArrive.keyframeId, 
                                                            gtsam::Pose3(   gtsam::Rot3::RzRyRx(euler[0], euler[1], euler[2]),
                                                                            gtsam::Point3(  t_wodom_curr.x(), 
                                                                                            t_wodom_curr.y(), 
                                                                                            t_wodom_curr.z())), 
                                                            priorNoise_
                                                            )                                
                            );
            initialEstimate_.insert(factorNewArrive.keyframeId, 
                                    gtsam::Pose3(   gtsam::Rot3::RzRyRx(euler[0], euler[1], euler[2]),
                                                                            gtsam::Point3(  t_wodom_curr.x(), 
                                                                                            t_wodom_curr.y(), 
                                                                                            t_wodom_curr.z())));

        }
        else{// add between factor
            // ROS_INFO("adding between factor");
            Eigen::Quaterniond q;
            q = graph_prev_rotation_;
            Eigen::Vector3d prev_euler = q.toRotationMatrix().eulerAngles(2, 1, 0);

            gtsam::Pose3 pose_from = gtsam::Pose3( gtsam::Rot3::RzRyRx(prev_euler[0], prev_euler[1], prev_euler[2]),
                                                    gtsam::Point3(  graph_prev_translation_.x(), 
                                                                    graph_prev_translation_.y(), 
                                                                    graph_prev_translation_.z()));
            

            gtsam::Pose3 pose_to = gtsam::Pose3(    gtsam::Rot3::RzRyRx(euler[0], euler[1], euler[2]),
                                                    gtsam::Point3(  t_wodom_curr.x(), 
                                                                    t_wodom_curr.y(), 
                                                                    t_wodom_curr.z()));
        
            const std::lock_guard<std::mutex> lock(factorMutex);
            gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(factorNewArrive.keyframeId-1, 
                                                                factorNewArrive.keyframeId, 
                                                                pose_from.between(pose_to), 
                                                                odometryNoise_));
            
            initialEstimate_.insert(factorNewArrive.keyframeId,  
                                    gtsam::Pose3(   gtsam::Rot3::RzRyRx(euler[0], euler[1], euler[2]),
                                                    gtsam::Point3(  t_wodom_curr.x(), 
                                                                    t_wodom_curr.y(), 
                                                                    t_wodom_curr.z())
                                                )
                                    );
            
                        

        }
        graph_prev_rotation_ = q_tmp;
        graph_prev_translation_ = t_wodom_curr;
       
        {
            const std::lock_guard<std::mutex> lock(factorMutex);
            isam_->update(gtSAMgraph_, initialEstimate_);
            isam_->update();
        
        
            gtSAMgraph_.resize(0);
            initialEstimate_.clear();
            
            PointType thisPose3D;
            PointTypePose thisPose6D;
            

            gtsam::Pose3 latestEstimate;
            isamCurrentEstimate_ = isam_->calculateEstimate();
            latestEstimate = isamCurrentEstimate_.at<gtsam::Pose3>(isamCurrentEstimate_.size()-1);

            thisPose3D.x = latestEstimate.translation().x();
            thisPose3D.y = latestEstimate.translation().y();
            thisPose3D.z = latestEstimate.translation().z();
            thisPose3D.intensity = cloudKeyPoses3D_->points.size(); 
            cloudKeyPoses3D_->push_back(thisPose3D);

            thisPose6D.x = thisPose3D.x;
            thisPose6D.y = thisPose3D.y;
            thisPose6D.z = thisPose3D.z;
            thisPose6D.intensity = thisPose3D.intensity; 
            thisPose6D.roll  = latestEstimate.rotation().roll();
            thisPose6D.pitch = latestEstimate.rotation().pitch();
            thisPose6D.yaw   = latestEstimate.rotation().yaw(); 
            thisPose6D.time = factorNewArrive.time; //1597969461.995467 
            cloudKeyPoses6D_->push_back(thisPose6D);
        }

        if(getLoop_ == true){
            updatePoses();
        }

        // tf broadcast
        gtsam::Pose3 latestEstimate;
        {
            const std::lock_guard<std::mutex> lock(factorMutex);                
            isamCurrentEstimate_ = isam_->calculateEstimate();
            latestEstimate = isamCurrentEstimate_.at<gtsam::Pose3>(isamCurrentEstimate_.size()-1);
            
        }



        auto q = latestEstimate.rotation().toQuaternion();
        static tf2_ros::TransformBroadcaster br; 
        geometry_msgs::TransformStamped transformStamped; 
        transformStamped.header.stamp = ros::Time().fromSec(factorNewArrive.time);
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "pgo_odom";          
        transformStamped.transform.translation.x = latestEstimate.translation().x();
        transformStamped.transform.translation.y = latestEstimate.translation().y();
        transformStamped.transform.translation.z = latestEstimate.translation().z();
        transformStamped.transform.rotation.w = q.w();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        br.sendTransform(transformStamped);

        //publish keypose
        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(*cloudKeyPoses3D_, cloudMsgTemp);
        cloudMsgTemp.header.stamp = ros::Time().fromSec(factorNewArrive.time);
        cloudMsgTemp.header.frame_id = "map";
        pubKeyPoses_.publish(cloudMsgTemp);
    }
    
}

void feature_tracker::factorGraphThread(){
    ros::Rate rate(100);
    while (ros::ok()){     
        if(true){
            TicToc pgo_time;
            mapOdomHandle(); 
            ROS_INFO("pgo_time: %f", pgo_time.toc());
            pgo_time_ = pgo_time.toc();
        }  
        rate.sleep(); 
        
    }
}

void feature_tracker::detectfeatures(ros::Time &time, 
                                     const cv::Mat &image_intensity, 
                                     const pcl::PointCloud<PointType>::Ptr _cloud_track,
                                     const pcl::PointCloud<pcl::PointXYZ>::Ptr groundPoint_)
{
    // new keyframe
    TicToc intensity_odom_time;
    TicToc intensity_feature_extraction_time;
    static int global_frame_index = 0;
    if(global_frame_index == 0) first_frame_time_ = time.toSec();
    time_stamp_ = time; 
    status_.clear(); 
    cv::Ptr<cv::ORB> detector = cv::ORB::create(NUM_ORB_FEATURES, 1.2f, 8, 1);

    cv::Mat image = image_intensity;
    cloudTrack_ = _cloud_track;
    cur_cloud_ = *_cloud_track;
    ground_cloud_ = *groundPoint_;
    
    cur_img_ = image;

    //feature points extract
    TicToc detectOrbFeaturesTime; //time start
    detector->detect(cur_img_, cur_keypoints_, MASK);
    
    cur_orb_point_2d_uv_ = keypoint2uv(cur_keypoints_);
    extractPointsAndFilterZeroValue(cur_orb_point_2d_uv_, cloudTrack_, cur_out_point3d_, status_);
    reduceVector(cur_out_point3d_, status_);
    reduceVector(cur_orb_point_2d_uv_, status_);
    reduceVector(cur_keypoints_, status_);

    detector->compute(cur_img_, cur_keypoints_, cur_descriptors_);
    


    std::vector<cv::DMatch> matches, good_matches; 
    cv::BFMatcher matcher = cv::BFMatcher(cv::NORM_HAMMING, true);
    if(!prev_img_.empty()){ //if it's not the first frame, then match features

        TicToc matchTime;                 
        matcher.match(cur_descriptors_, prev_descriptors_, matches);
        ROS_DEBUG("cur_keyPoints num:%ld", cur_keypoints_.size());
        ROS_DEBUG("prev_keyPoints num:%ld", prev_keypoints_.size());
        ROS_DEBUG("Matches num:%ld, matched time:%f", matches.size(), matchTime.toc());
        
        std::sort(matches.begin(), matches.end());
        //remove bad matches
        for (size_t i = 0; i < matches.size()*0.3; ++i)
        {
            good_matches.push_back(matches[i]);
        }
        ROS_DEBUG("good_matches num:%ld", good_matches.size());

        std::string detectTime = std::to_string(detectOrbFeaturesTime.toc()); //time count end
        //match again
        if(!(prev_keypoints_.size() != cur_keypoints_.size() && good_matches.size() >=4 && good_matches.size()!= matches.size())){
            // re-detect for cur and prev frames. 
            TicToc reDetectTime;
            cv::Ptr<cv::ORB> detector2 = cv::ORB::create(NUM_ORB_FEATURES*2, 1.2f, 8, 1);

            //current frame
            detector2->detect(cur_img_, cur_keypoints_, MASK);
            cur_orb_point_2d_uv_ = keypoint2uv(cur_keypoints_);
            extractPointsAndFilterZeroValue(cur_orb_point_2d_uv_, cloudTrack_, cur_out_point3d_, status_);
            reduceVector(cur_out_point3d_, status_);
            reduceVector(cur_orb_point_2d_uv_, status_);
            reduceVector(cur_keypoints_, status_);
            detector2->compute(cur_img_, cur_keypoints_, cur_descriptors_);


            // previous frame
            detector2->detect(prev_img_, prev_keypoints_, MASK);
            prev_orb_point_2d_uv_ = keypoint2uv(prev_keypoints_);
            pcl::PointCloud<PointType>::Ptr prev_cloud_ptr_(new pcl::PointCloud<PointType>(prev_cloud_));

            extractPointsAndFilterZeroValue(prev_orb_point_2d_uv_, prev_cloud_ptr_, prev_out_point3d_, status_);
            reduceVector(prev_out_point3d_, status_);
            reduceVector(prev_orb_point_2d_uv_, status_);
            reduceVector(prev_keypoints_, status_);
            detector2->compute(prev_img_, prev_keypoints_, prev_descriptors_); 

            matcher.match(cur_descriptors_, prev_descriptors_, matches);
            
            std::sort(matches.begin(), matches.end());
            //remove bad matches
            good_matches.clear();
            for (size_t i = 0; i < matches.size()*0.2; ++i)
            {
                good_matches.push_back(matches[i]);
            }
            ROS_DEBUG("good_matches num:%ld", good_matches.size());

            detectTime = "re-detect"+std::to_string(detectOrbFeaturesTime.toc());
            // std::cout << "re-detect time:" << reDetectTime.toc() << " ms" << std::endl;

        }

        

        if(prev_keypoints_.size() != cur_keypoints_.size() && good_matches.size() >=4 && good_matches.size()!= matches.size()){
            global_frame_index++;
            ROS_DEBUG("global frame:%d", global_frame_index);
            frequency_ = round(1.0 * global_frame_index/(time.toSec() - first_frame_time_));
            std::string frequency_s = std::to_string(frequency_);
            std::string name(", Hz:");
            detectTime += name; 
            detectTime += frequency_s; 
            // show image with matched feature points
            image_show(good_matches, detectTime);
            
            //extract points from pointcloud according to the index of feature points
            std::vector<cv::Point3f> prev_matched_points3d, cur_matched_points3d; 
            extractMatchedPoints(good_matches, prev_matched_points3d, cur_matched_points3d, prev_out_point3d_, cur_out_point3d_);
            ROS_INFO("features detectation time:%lf ms", intensity_feature_extraction_time.toc()); //odom node
            feature_extraction_time_ = intensity_feature_extraction_time.toc();


            //calculate rotation and translation matrix
            TicToc scan_registration_time;
            T_s2s_ = p2p_calculateRandT(cur_matched_points3d, prev_matched_points3d);
            // std::cout << "intensity scan registration time:" << scan_registration_time.toc() << " ms" << std::endl;
            scan_matching_time_ = scan_registration_time.toc();
            tfBroadcast();

            getKeyframe(time_stamp_.toSec(), cur_out_point3d_);

        }
        else{// in case of bad frame, skip this frame and use geometry features to calculate the transformation matrix, usually only 1 or 2 frames. 
            static size_t skip_count = 0;
            skip_count++;
            ROS_ERROR("skip this bad frame:%ld ", skip_count); 
            skip_flag_ = true;
            skip_intensity_ = true; 
            T_s2s_ = Eigen::Matrix4d::Identity();
            tfBroadcast();
        }
        
    }

    prev_cloud_ =cur_cloud_;
    prev_img_ = cur_img_;
    prev_keypoints_ = cur_keypoints_;
    prev_descriptors_ = cur_descriptors_;
    prev_orb_point_2d_uv_ = cur_orb_point_2d_uv_;
    prev_out_point3d_ = cur_out_point3d_;

}
void feature_tracker::getKeyframe(double time, std::vector<cv::Point3f> &featurMatched3Dpoints){
    cur_position_ = T_s2m_.block(0,3,3,1); 
    cur_rotation_ = T_s2m_.block(0,0,3,3);

    double image_time = time;
    static double last_skip_time = -1; 
    if(cloudKeyPoses3D_->points.empty()){
        {
                const std::lock_guard<std::mutex> lock(factorMutex);

                std::shared_ptr<loopClosureProcessor::FactorGraphNode> factorNew(new loopClosureProcessor::FactorGraphNode(time, keyframeId_, cur_position_, prev_position_, cur_rotation_, prev_rotation_)); 
                factorGraphNode_.emplace_back(factorNew);

            }
            {
                const std::lock_guard<std::mutex> lock(factorMutex);
                std::shared_ptr<loopClosureProcessor::Keyframe> keyframeNew(new loopClosureProcessor::Keyframe(time, keyframeId_, cur_img_, cur_keypoints_, cur_descriptors_, featurMatched3Dpoints, cur_position_, prev_position_, cur_rotation_, prev_rotation_, *cloudTrack_));
                keyframesQueue_.emplace_back(keyframeNew);

            }

            // update parameters
            last_skip_time = image_time; 
            prev_position_ = cur_position_;
            prev_rotation_ = cur_rotation_;  
            keyframeId_++;
            return;

    }
    //update other keyframes
    if(image_time - last_skip_time > KEYFRAME_TIME_INTERVAL){
        double distance = (cur_position_ - prev_position_).norm();
        if(distance > KEYFRAME_DISTANCE_INTERVAL){
            keyframe_={time, keyframeId_, cur_img_, cur_keypoints_, cur_descriptors_, featurMatched3Dpoints, cur_position_, prev_position_, cur_rotation_, prev_rotation_, *cloudTrack_};

            {
                const std::lock_guard<std::mutex> lock(factorMutex);

                std::shared_ptr<loopClosureProcessor::FactorGraphNode> factorNew(new loopClosureProcessor::FactorGraphNode(time, keyframeId_, cur_position_, prev_position_, cur_rotation_, prev_rotation_)); 
                factorGraphNode_.emplace_back(factorNew);

            }
            {
                const std::lock_guard<std::mutex> lock(factorMutex);
                std::shared_ptr<loopClosureProcessor::Keyframe> keyframeNew(new loopClosureProcessor::Keyframe(time, keyframeId_, cur_img_, cur_keypoints_, cur_descriptors_, featurMatched3Dpoints, cur_position_, prev_position_, cur_rotation_, prev_rotation_, *cloudTrack_));
                keyframesQueue_.emplace_back(keyframeNew);

            }

            // update parameters
            last_skip_time = image_time; 
            prev_position_ = cur_position_;
            prev_rotation_ = cur_rotation_;  
            keyframeId_++;
            
        }

         

    }     
    
    

}
void feature_tracker::tfBroadcast(){
    //global transformation matrix
    T_s2m_ = T_s2m_ * T_s2s_;
    {
        const std::lock_guard<std::mutex> lock(factorMutex);
        T_s2pgo_ = T_s2pgo_ * T_s2s_;

    }
    

    //global rotation
    Eigen::Matrix3d rot; 
    rot = T_s2m_.block(0,0,3,3);
    Eigen::Quaterniond q(rot);    
    q = q.normalized();

    // global position 
    Eigen::Vector3d p(T_s2m_.block(0,3,3,1));

    //broadcast
    static tf2_ros::TransformBroadcaster br; 
    geometry_msgs::TransformStamped transformStamped; 

    transformStamped.header.stamp = time_stamp_;

    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "intensity_odom";

    transformStamped.transform.translation.x = p[0];
    transformStamped.transform.translation.y = p[1];
    transformStamped.transform.translation.z = p[2];

    transformStamped.transform.rotation.w = q.w();
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();

    br.sendTransform(transformStamped);

    // publish laser odometry
    nav_msgs::Odometry laserOdometry;
    laserOdometry.header.frame_id = "/map";
    if(skip_flag_){
        skip_flag_ = false;
        laserOdometry.child_frame_id = "/odom_skip";
    }
    else{
        laserOdometry.child_frame_id = "/laser_odom";
    }
    // laserOdometry.child_frame_id = "/laser_odom";
    laserOdometry.header.stamp = time_stamp_;
    laserOdometry.pose.pose.orientation.x = q.x();
    laserOdometry.pose.pose.orientation.y = q.y();
    laserOdometry.pose.pose.orientation.z = q.z();
    laserOdometry.pose.pose.orientation.w = q.w();
    laserOdometry.pose.pose.position.x = p[0];
    laserOdometry.pose.pose.position.y = p[1];
    laserOdometry.pose.pose.position.z = p[2];
    pubLaserOdometry_.publish(laserOdometry);
}
Eigen::Matrix4d  feature_tracker::p2p_calculateRandT(std::vector<cv::Point3f> &src_cloud, std::vector<cv::Point3f> &dst_cloud){
    //initial ceres solver parameters
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    ceres::LocalParameterization *q_parameterization =
        new ceres::EigenQuaternionParameterization();
    ceres::Problem::Options problem_options;

    ceres::Problem problem(problem_options);
    double fe_parameters_[7] = {0, 0, 0, 1, 0, 0, 0}; //front end rotation and translation.
    problem.AddParameterBlock(fe_parameters_, 4, q_parameterization);
    problem.AddParameterBlock(fe_parameters_ + 4, 3);

    // iteration each point in the src_cloud and dst_cloud
    for (size_t i = 0; i < src_cloud.size(); i++)
    {
        Eigen::Vector3d src_point = Eigen::Vector3d(src_cloud[i].x, src_cloud[i].y, src_cloud[i].z);
        Eigen::Vector3d dst_point = Eigen::Vector3d(dst_cloud[i].x, dst_cloud[i].y, dst_cloud[i].z);        
        ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<front_end_residual, 3, 4, 3>(
                    new front_end_residual(src_point, dst_point));
        problem.AddResidualBlock(cost_function, loss_function, fe_parameters_, fe_parameters_ + 4);
    }

    // solve the problem
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 20;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // std::cout << summary.BriefReport() << "\n";

    //update the q and t
    Eigen::Map<Eigen::Quaterniond> q_w_curr(fe_parameters_);
    Eigen::Map<Eigen::Vector3d> t_w_curr(fe_parameters_ + 4);

    //convert q to rotation matrix
    Eigen::Matrix3d R_w_curr = q_w_curr.toRotationMatrix();

    // covert R and T to T_s2s_
    Eigen::Matrix3d R = R_w_curr;// 3*3
    Eigen::Vector3d T = t_w_curr;//3*1

    Eigen::Matrix4d T_s2s_temp; 
    T_s2s_temp.block(0,0,3,3) = R; 
    T_s2s_temp.block(0,3,3,1) = T; 
    T_s2s_temp.block(3,0,1,4) << 0, 0, 0, 1;

    return T_s2s_temp; 
    // return Eigen::Matrix4d::Identity();

}

void feature_tracker::extractMatchedPoints(std::vector<cv::DMatch> &matches, std::vector<cv::Point3f> &prev_points, std::vector<cv::Point3f> &cur_points, std::vector<cv::Point3f> &prev_out_point3d, std::vector<cv::Point3f> &cur_out_point3d){
    prev_points.resize(matches.size());
    cur_points.resize(matches.size());

    #pragma omp parallel for num_threads(NUM_THREADS)
    for(size_t i=0; i<matches.size(); i++){
        int prev_point_index = matches[i].trainIdx; 
        int cur_point_index  = matches[i].queryIdx;
        prev_points[i] = prev_out_point3d_[prev_point_index];
        cur_points[i] = cur_out_point3d_[cur_point_index];
    }

}
void feature_tracker::image_show(std::vector<cv::DMatch> &matches, std::string& detectTime, cv::Mat prev_img_, cv::Mat cur_img_, std::vector<cv::Point2f> cur_orb_point_2d_uv_, std::vector<cv::Point2f> prev_orb_point_2d_uv_){
    int gap =10; 
    cv::Mat gap_image(gap, prev_img_.size().width, CV_8UC1, cv::Scalar(255,255,255));
    cv::Mat img_show;

    cv::vconcat(cur_img_, gap_image, img_show); 
    cv::vconcat(img_show, prev_img_, img_show);  
    cv::cvtColor(img_show, img_show, cv::COLOR_GRAY2RGB);

    // draw keypoints in current frame 
    for(size_t i = 0; i< (size_t)cur_orb_point_2d_uv_.size(); i++){
        cv::Point2f cur_pt = cur_orb_point_2d_uv_[i];
        cv::circle(img_show, cur_pt, 5, cv::Scalar(0,255,0));
    }
    //  draw keypoints in previous frame
    for(size_t i = 0; i< (size_t)prev_orb_point_2d_uv_.size(); i++){
        cv::Point2f prev_pt = prev_orb_point_2d_uv_[i];
        prev_pt.y += cur_img_.size().height + gap;
        cv::circle(img_show, prev_pt, 5, cv::Scalar(0,0,255));
    }

    //draw lines for matches
    for(size_t i = 0; i< (size_t)matches.size(); i++){
        int cur_pt_index = matches[i].queryIdx;
        cv::Point2f cur_pt = cur_orb_point_2d_uv_[cur_pt_index];
        int prev_pt_index = matches[i].trainIdx;
        cv::Point2f prev_pt = prev_orb_point_2d_uv_[prev_pt_index]; 
        prev_pt.y += cur_img_.size().height + gap;

        cv::line(img_show, cur_pt, prev_pt, cv::Scalar(0,255,0), 2, 8, 0);
    }

    std::string keypoint_cur_img_text("cur_img, time cost ms:");
    keypoint_cur_img_text.append(detectTime);

    std::string match_num("Match num:");
    int match_size = (int)matches.size();
    match_num += std::to_string(match_size);


    cv::putText(img_show, keypoint_cur_img_text,   cv::Point2f(5, 20 + IMAGE_HEIGHT*0), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);

    cv::putText(img_show, match_num,   cv::Point2f(5, 60 + IMAGE_HEIGHT*0), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);

    cv::putText(img_show, "prev_img",   cv::Point2f(5, 20 + IMAGE_HEIGHT*1 + gap), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);

    if(matched_keypoints_img_pub_front_end.getNumSubscribers() > 0){
        cv_bridge::CvImage output_image;
        output_image.header.frame_id = "map";
        output_image.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
        output_image.image = img_show;
        matched_keypoints_img_pub_front_end.publish(output_image);
    }

}

void feature_tracker::image_show(std::vector<cv::DMatch> &matches, std::string& detectTime){
    int gap =10; 
    cv::Mat gap_image(gap, prev_img_.size().width, CV_8UC1, cv::Scalar(255,255,255));
    cv::Mat img_show;

    cv::vconcat(cur_img_, gap_image, img_show); 
    cv::vconcat(img_show, prev_img_, img_show);  
    cv::cvtColor(img_show, img_show, cv::COLOR_GRAY2RGB);

    // draw keypoints in current frame 
    for(size_t i = 0; i< (size_t)cur_orb_point_2d_uv_.size(); i++){
        cv::Point2f cur_pt = cur_orb_point_2d_uv_[i];
        cv::circle(img_show, cur_pt, 5, cv::Scalar(0,255,0));
    }
    //  draw keypoints in previous frame
    for(size_t i = 0; i< (size_t)prev_orb_point_2d_uv_.size(); i++){
        cv::Point2f prev_pt = prev_orb_point_2d_uv_[i];
        prev_pt.y += cur_img_.size().height + gap;
        cv::circle(img_show, prev_pt, 5, cv::Scalar(0,0,255));
    }

    //draw lines for matches
    for(size_t i = 0; i< (size_t)matches.size(); i++){
        int cur_pt_index = matches[i].queryIdx;
        cv::Point2f cur_pt = cur_orb_point_2d_uv_[cur_pt_index];
        int prev_pt_index = matches[i].trainIdx;
        cv::Point2f prev_pt = prev_orb_point_2d_uv_[prev_pt_index]; 
        prev_pt.y += cur_img_.size().height + gap;

        cv::line(img_show, cur_pt, prev_pt, cv::Scalar(0,255,0), 2, 8, 0);
    }

    std::string keypoint_cur_img_text("cur_img, time cost ms:");
    keypoint_cur_img_text.append(detectTime);

    std::string match_num("Match num:");
    int match_size = (int)matches.size();
    match_num += std::to_string(match_size);


    cv::putText(img_show, keypoint_cur_img_text,   cv::Point2f(5, 20 + IMAGE_HEIGHT*0), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);

    cv::putText(img_show, match_num,   cv::Point2f(5, 60 + IMAGE_HEIGHT*0), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);

    cv::putText(img_show, "prev_img",   cv::Point2f(5, 20 + IMAGE_HEIGHT*1 + gap), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);

    if(matched_keypoints_img_pub_front_end.getNumSubscribers() > 0){
        cv_bridge::CvImage output_image;
        output_image.header.frame_id = "map";
        output_image.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
        output_image.image = img_show;
        matched_keypoints_img_pub_front_end.publish(output_image);
    }


}

void feature_tracker::keypoint2uv(){
    cur_orb_point_2d_uv_.resize(cur_keypoints_.size());
    for (size_t i = 0; i < cur_keypoints_.size(); i++)
    {
        cur_orb_point_2d_uv_[i] = cur_keypoints_[i].pt;
    } 
}

std::vector<cv::Point2f> feature_tracker::keypoint2uv(std::vector<cv::KeyPoint> cur_keypoints){
    std::vector<cv::Point2f> cur_orb_point_2d_uv;
    cur_orb_point_2d_uv.resize(cur_keypoints.size());
    for (size_t i = 0; i < cur_keypoints.size(); i++)
    {
        cur_orb_point_2d_uv[i] = cur_keypoints[i].pt;
    } 
    return cur_orb_point_2d_uv;
}

void feature_tracker::extractPointsAndFilterZeroValue(std::vector<cv::Point2f> cur_orb_point_2d_uv, pcl::PointCloud<pcl::PointXYZI>::Ptr cloudTrack, std::vector<cv::Point3f> &cur_out_point3d, std::vector<uchar> &status){
    ROS_DEBUG("Extract Points and filter zero value"); 
    assert(cloudTrack->size()>0);
    cur_out_point3d.resize(cur_orb_point_2d_uv.size());
    status.resize(cur_orb_point_2d_uv.size());

    #pragma omp parallel for num_threads(NUM_THREADS)
    for(size_t i=0; i< cur_orb_point_2d_uv.size(); i++){
        int col_id = cvRound(cur_orb_point_2d_uv[i].x);
        int row_id = cvRound(cur_orb_point_2d_uv[i].y);
        int index = row_id * IMAGE_WIDTH + col_id;

        pcl::PointXYZI *point_i = &cloudTrack->points[index];

        cv::Point3f p_3d(0.0f, 0.0f, 0.0f);

        if(abs(point_i->x) < 0.01){
            status[i] = 0; //filter points if there are no x, y, z values for this pixel.
        }
        else{
            status[i] = 1; 
            p_3d.x = point_i->x;
            p_3d.y = point_i->y; 
            p_3d.z = point_i->z; 
        } 
        cur_out_point3d[i] = p_3d; 
    } 
}

void feature_tracker::extractPointsAndFilterZeroValue(){
    ROS_DEBUG("Extract Points and filter zero value"); 
    assert(cloudTrack_->size()>0);
    cur_out_point3d_.resize(cur_orb_point_2d_uv_.size());
    status_.resize(cur_orb_point_2d_uv_.size());

    #pragma omp parallel for num_threads(NUM_THREADS)
    for(size_t i=0; i< cur_orb_point_2d_uv_.size(); i++){
        int col_id = cvRound(cur_orb_point_2d_uv_[i].x);
        int row_id = cvRound(cur_orb_point_2d_uv_[i].y);
        int index = row_id * IMAGE_WIDTH + col_id;

        pcl::PointXYZI *point_i = &cloudTrack_->points[index];

        cv::Point3f p_3d(0.0f, 0.0f, 0.0f);

        if(abs(point_i->x) < 0.01){
            status_[i] = 0; //filter points if there are no x, y, z values for this pixel.
        }
        else{
            status_[i] = 1; 
            p_3d.x = point_i->x;
            p_3d.y = point_i->y; 
            p_3d.z = point_i->z; 
        } 
        cur_out_point3d_[i] = p_3d;  
    } 
}

void feature_tracker::readParameters(){    
    nh_.getParam("/intensity_feature_tracker/project_name", PROJECT_NAME);     
    nh_.getParam("/intensity_feature_tracker/cloud_topic", CLOUD_TOPIC); 
    nh_.getParam("/intensity_feature_tracker/image_width", IMAGE_WIDTH);
    nh_.getParam("/intensity_feature_tracker/image_height", IMAGE_HEIGHT);
    nh_.getParam("/intensity_feature_tracker/image_crop", IMAGE_CROP);
    nh_.getParam("/intensity_feature_tracker/use_orb", USE_ORB);
    nh_.getParam("/intensity_feature_tracker/num_orb_features", NUM_ORB_FEATURES);    
    nh_.getParam("/intensity_feature_tracker/skip_time", SKIP_TIME);
    nh_.getParam("/intensity_feature_tracker/num_threads", NUM_THREADS);
    nh_.getParam("/intensity_feature_tracker/hand_held_flag", HAND_HELD_FLAG);
    nh_.getParam("/intensity_feature_tracker/use_teaser", USE_TEASER);
    nh_.getParam("/intensity_feature_tracker/use_pnpransac", USE_PNPRANSAC);
    nh_.param<bool>("/intensity_feature_tracker/use_icp", USE_ICP, true); 
    nh_.param<bool>("/loop_closure_parameters/use_crop", USE_CROP, true); 
    nh_.param<bool>("/loop_closure_parameters/use_voxel_downsample", USE_DOWNSAMPLE, true);
    nh_.param<double>("/loop_closure_parameters/crop_size", CROP_SIZE, 1.0);
    nh_.param<double>("/loop_closure_parameters/vf_scan_res", VOXEL_SIZE, 0.25);
    nh_.param<double>("/loop_closure_parameters/icp_fitness_score", FITNESS_SCORE, 0.3);
    nh_.param<double>("/loop_closure_parameters/keyframe_time_intervals", KEYFRAME_TIME_INTERVAL, 15);
    nh_.param<double>("/loop_closure_parameters/keyframe_distance_intervals", KEYFRAME_DISTANCE_INTERVAL, 0.3);
    if(HAND_HELD_FLAG) setMask();
}

void feature_tracker::setMask(){
    MASK = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, cv::Scalar(255));
    for (int i = 0; i < IMAGE_HEIGHT; ++i)
        for (int j = 0; j < IMAGE_WIDTH; ++j)
            if (j < IMAGE_CROP || j > IMAGE_WIDTH - IMAGE_CROP)
                MASK.at<uchar>(i,j) = 0;

}