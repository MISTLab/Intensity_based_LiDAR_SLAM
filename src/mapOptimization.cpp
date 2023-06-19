#include "mapOptimization.hpp"
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <ceres/ceres.h>
#include "lidarFeaturePointsFunction.hpp"
void mapOptimization::laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
    TicToc laserOdomHandler_time; 
	Eigen::Quaterniond q_wodom_curr;
	Eigen::Vector3d t_wodom_curr;
	q_wodom_curr.x() = laserOdometry->pose.pose.orientation.x;
	q_wodom_curr.y() = laserOdometry->pose.pose.orientation.y;
	q_wodom_curr.z() = laserOdometry->pose.pose.orientation.z;
	q_wodom_curr.w() = laserOdometry->pose.pose.orientation.w;
	t_wodom_curr.x() = laserOdometry->pose.pose.position.x;
	t_wodom_curr.y() = laserOdometry->pose.pose.position.y;
	t_wodom_curr.z() = laserOdometry->pose.pose.position.z;

	Eigen::Quaterniond q_w_curr = q_wmap_wodom * q_wodom_curr;
	Eigen::Vector3d t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom; 

	nav_msgs::Odometry odomAftMapped;
	odomAftMapped.header.frame_id = "map";
	odomAftMapped.child_frame_id = "high_freq_odom";
	odomAftMapped.header.stamp = laserOdometry->header.stamp;
	odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
	odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
	odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
	odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
	odomAftMapped.pose.pose.position.x = t_w_curr.x();
	odomAftMapped.pose.pose.position.y = t_w_curr.y();
	odomAftMapped.pose.pose.position.z = t_w_curr.z();
	pubOdomAftMappedHighFrec.publish(odomAftMapped);

    //robot model
    // Create a mesh marker for the spot
    visualization_msgs::Marker mesh_marker;
    mesh_marker.header.frame_id = "map";
    mesh_marker.header.stamp = laserOdometry->header.stamp;
    mesh_marker.ns = "spot";
    mesh_marker.id = 0;
    mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh_marker.action = visualization_msgs::Marker::ADD;
    mesh_marker.pose.position.x = t_w_curr.x();
    mesh_marker.pose.position.y = t_w_curr.y();
    mesh_marker.pose.position.z = t_w_curr.z();
    mesh_marker.pose.orientation.x = q_w_curr.x();
    mesh_marker.pose.orientation.y = q_w_curr.y();
    mesh_marker.pose.orientation.z = q_w_curr.z();
    mesh_marker.pose.orientation.w = q_w_curr.w();
    mesh_marker.scale.x = 1;
    mesh_marker.scale.y = 1;
    mesh_marker.scale.z = 1;
    mesh_marker.color.r = 1;
    mesh_marker.color.g = 1;
    mesh_marker.color.b = 1;
    mesh_marker.color.a = 1.0;
    mesh_marker.lifetime = ros::Duration(0.5);
    std::string PROJECT_NAME("intensity_feature_tracker");
    std::string pkg_path = ros::package::getPath(PROJECT_NAME);
    std::string spot_mesh_file("/config/spot.stl");
    std::string prefix_str("package://");
    std::string mesh_path_str = prefix_str + PROJECT_NAME + spot_mesh_file;
    mesh_marker.mesh_resource = mesh_path_str;
    mesh_marker.mesh_use_embedded_materials = true;
    robot_marker_pub.publish(mesh_marker); 
    
}
void mapOptimization::filterNaNPoints(){
    
}

void mapOptimization::mapOptimizationCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, const nav_msgs::Odometry::ConstPtr& odom_msg, const sensor_msgs::PointCloud2::ConstPtr& plane_cloud_msg, const sensor_msgs::PointCloud2::ConstPtr& pc_corner_msg){
    TicToc map_optimization_time;
    q_wodom_curr.x() = odom_msg->pose.pose.orientation.x;
    q_wodom_curr.y() = odom_msg->pose.pose.orientation.y;
    q_wodom_curr.z() = odom_msg->pose.pose.orientation.z;
    q_wodom_curr.w() = odom_msg->pose.pose.orientation.w;
    t_wodom_curr.x() = odom_msg->pose.pose.position.x;
    t_wodom_curr.y() = odom_msg->pose.pose.position.y;
    t_wodom_curr.z() = odom_msg->pose.pose.position.z;
    transformAssociateToMap(); 
    
    Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters_);
    Eigen::Map<Eigen::Vector3d> t_w_curr(parameters_ + 4);

    Eigen::Quaterniond q_w_curr_original(q_w_curr);
    Eigen::Vector3d t_w_curr_original(t_w_curr);

    TicToc tic_toc_pc_processing;
    pcl::PointCloud<GroundPlanePointType> pc_plane;
    pcl::PointCloud<GroundPlanePointType> pc_corner;

    image_handler_->GroundPointOut->points.clear();
    #pragma omp parallel sections num_threads(4) //~20ms
    {
        #pragma omp section
        image_handler_->groundPlaneExtraction(cloud_msg);

        #pragma omp section
        pcl::fromROSMsg(*plane_cloud_msg,  pc_plane); // read the plane pointcloud

        #pragma omp section
        pcl::fromROSMsg(*pc_corner_msg,  pc_corner); // read edge points
        
        #pragma omp section
        image_handler_->cloud_handler(cloud_msg); //~5ms processing intensity image
    }
    

    *image_handler_->GroundPointOut += pc_plane;
    std::vector<int> idx;
    pcl::removeNaNFromPointCloud(*image_handler_->GroundPointOut, *image_handler_->GroundPointOut, idx);
    pcl::removeNaNFromPointCloud(pc_corner, pc_corner, idx);

    cv::Mat image_intensity = image_handler_->image_intensity;
    cv::Ptr<cv::ORB> detector = cv::ORB::create(NUM_ORB_FEATURES*2, 1.2f, 8, 1);
    detector->detect(image_intensity, cur_keypoints_, MASK);
    keypoint2uv(cur_keypoints_, cur_orb_point_2d_uv_);
    extractPointsAndFilterZeroValue(cur_orb_point_2d_uv_, image_handler_->cloud_track, cur_out_point3d_, status_);
    reduceVector(cur_out_point3d_, status_);
    reduceVector(cur_orb_point_2d_uv_, status_);
    reduceVector(cur_keypoints_, status_);
    detector->compute(image_intensity, cur_keypoints_, cur_descriptors_);

    std::shared_ptr<mapProcessor::SlideWindowKeyframe> mapKeyframeNew( new mapProcessor::SlideWindowKeyframe(cur_descriptors_, image_intensity, cur_orb_point_2d_uv_, q_w_curr, t_w_curr, cur_out_point3d_));
    


    //match the features with previous keyframe, initialize parameters
    std::vector<cv::DMatch> matches, good_matches; 
    if(prev_keyframe_img.empty()){
        prev_keyframe_img = image_intensity;
        prev_keypoints_ = cur_keypoints_;
        prev_descriptors_ = cur_descriptors_;
        prev_orb_point_2d_uv_ = cur_orb_point_2d_uv_;
        prev_out_point3d_ = cur_out_point3d_;
        keyframeId_ = 0; 
         
        std::shared_ptr<mapProcessor::Keyframe> keyframetmp (new mapProcessor::Keyframe(keyframeId_, *image_handler_->cloud_track, *image_handler_->GroundPointOut, q_w_curr, t_w_curr)); 
        prev_keyframe = *keyframetmp;
        
        ground_plane_cloud_ = *image_handler_->GroundPointOut;
        Eigen::Matrix4d cur_transform = Eigen::Matrix4d::Identity();
        cur_transform.block<3,3>(0,0) = q_w_curr.toRotationMatrix();
        cur_transform.block<3,1>(0,3) = t_w_curr;
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(ground_plane_cloud_, *tmp_cloud, cur_transform);
        ikdtree->Build((*tmp_cloud).points);
        tmp_cloud->clear();
        pcl::transformPointCloud(pc_corner, *tmp_cloud, cur_transform);
        corner_ikdtree_->Build((*tmp_cloud).points);
           

    }
    else{
        matcher_.match(cur_descriptors_, prev_descriptors_, matches);
        std::sort(matches.begin(), matches.end());
        for (size_t i = 0; i < matches.size()*0.2; ++i)
        {
            good_matches.push_back(matches[i]);
        }
        
        
        
        if(1){ 
            keyframe_flag_ = true; 
            keyframeId_++; 
            extractMatchedPoints(good_matches, prev_matched_points3d_, prev_keyframe.q_map_cur_k_, prev_keyframe.t_map_cur_k_, cur_matched_points3d_, q_w_curr, t_w_curr, prev_out_point3d_, cur_out_point3d_, cloud_msg->header);
        }
        std::shared_ptr<mapProcessor::Keyframe> keyframetmp (new mapProcessor::Keyframe(keyframeId_, *image_handler_->cloud_track, *image_handler_->GroundPointOut, q_w_curr, t_w_curr));
        cur_keyframe = *keyframetmp;
         
        
    }

    if(keyframe_flag_ == true){
        keyframe_flag_ = false;
        if(ground_points_pub.getNumSubscribers() != 0){
            pcl::PointCloud<pcl::PointXYZ>::Ptr ikdtree_points(new pcl::PointCloud<pcl::PointXYZ>());
            corner_ikdtree_->flatten(ikdtree->Root_Node, ikdtree->PCL_Storage, NOT_RECORD);
            ikdtree_points->points = ikdtree->PCL_Storage;
            sensor_msgs::PointCloud2 groundPlaneMapCloud_msg;
            pcl::toROSMsg(*ikdtree_points, groundPlaneMapCloud_msg);
            groundPlaneMapCloud_msg.header.frame_id = "map";
            groundPlaneMapCloud_msg.header.stamp = cloud_msg->header.stamp;
            ground_points_pub.publish(groundPlaneMapCloud_msg);
        }
        
        
        
        
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        ceres::LocalParameterization *q_parameterization =
            new ceres::EigenQuaternionParameterization();
        ceres::Problem::Options problem_options;

        ceres::Problem problem(problem_options);
        problem.AddParameterBlock(parameters_, 4, q_parameterization);
        problem.AddParameterBlock(parameters_ + 4, 3);

        size_t num_good_matches = 0; 
        
        //publish line list for visualization
        initialLineList(line_list ,cloud_msg->header);
        if(prev_keypoints_.size() != cur_keypoints_.size() && good_matches.size() >=4 && good_matches.size()!= matches.size()){
            for(size_t i = 0; i < prev_matched_points3d_.size() && false; i++){
                Eigen::Vector3d prev_point3d = prev_keyframe.q_map_cur_k_ * Eigen::Vector3d(prev_matched_points3d_[i].x, prev_matched_points3d_[i].y, prev_matched_points3d_[i].z) + prev_keyframe.t_map_cur_k_;
                Eigen::Vector3d cur_point3d = cur_keyframe.q_map_cur_k_ * Eigen::Vector3d(cur_matched_points3d_[i].x, cur_matched_points3d_[i].y, cur_matched_points3d_[i].z) + cur_keyframe.t_map_cur_k_;
                geometry_msgs::Point prev_point, cur_point;
                prev_point.x = prev_point3d(0);
                prev_point.y = prev_point3d(1);
                prev_point.z = prev_point3d(2);
                cur_point.x = cur_point3d(0);
                cur_point.y = cur_point3d(1);
                cur_point.z = cur_point3d(2);
                {
                    std::lock_guard<std::mutex> lock(map_mutex_);
                    appendLines(line_list, prev_point, cur_point);
                }
                
                double distance = (prev_point3d - cur_point3d).norm();
                if(distance > 0.5){
                    continue;
                }
                num_good_matches++;

                Eigen::Vector3d cur_point3d_vector3d(cur_matched_points3d_[i].x, cur_matched_points3d_[i].y, cur_matched_points3d_[i].z);               
                Eigen::Vector3d point_w;
                point_w = q_w_curr * cur_point3d_vector3d + t_w_curr;
                // BA residual
                ceres::CostFunction* cost_function =
                    new ceres::AutoDiffCostFunction<FeatureMatchingResidual, 3, 4, 3>(
                        new FeatureMatchingResidual(cur_point3d_vector3d, prev_point3d));
                {
                    std::lock_guard<std::mutex> lock(map_mutex_);
                    problem.AddResidualBlock(cost_function, loss_function, parameters_, parameters_ + 4);
                }
            }
        }
        if(matched_lines_pub.getNumSubscribers() !=0){
            matched_lines_pub.publish(line_list);
        }
        if(keyframe_sliding_window_.size() >= 1 && true)
        {
            
            auto cur_keyframe_from_SW = mapKeyframeNew;
            #pragma omp parallel for num_threads(NUM_THREADS)
            for (auto it = keyframe_sliding_window_.rbegin(); it != keyframe_sliding_window_.rend(); ++it){
                auto prev_keyframe_from_SW = *it;
                cv::BFMatcher matcher_tmp; 
                std::vector<cv::DMatch> matches_tmp, good_matches_tmp; 
                matcher_tmp.match(cur_keyframe_from_SW->descriptors, prev_keyframe_from_SW->descriptors, matches_tmp);
                if(matches_tmp.size() > 100 ){
                    std::sort(matches_tmp.begin(), matches_tmp.end());
                    for (size_t i = 0; i < matches_tmp.size()*0.2; ++i) 
                    {
                        good_matches_tmp.push_back(matches_tmp[i]);
                    }
                    {
                        std::string detectTime = std::to_string(tic_toc_pc_processing.toc());
                        image_show(good_matches_tmp, detectTime, prev_keyframe_from_SW->image_intensity, cur_keyframe_from_SW->image_intensity, cur_keyframe_from_SW->orb_point_2d_uv, prev_keyframe_from_SW->orb_point_2d_uv);
                    }
                    std::vector<cv::Point3f> prev_matched_points3d_tmp, cur_matched_points3d_tmp;
                    if(good_matches_tmp.size() > 50 && good_matches_tmp.size() != matches_tmp.size() && prev_keyframe_from_SW->descriptors.size() != cur_keyframe_from_SW->descriptors.size()){
                        extractMatchedPoints(good_matches_tmp, prev_matched_points3d_tmp, cur_matched_points3d_tmp,  prev_keyframe_from_SW->cur_point3d_wrt_orb_features, cur_keyframe_from_SW->cur_point3d_wrt_orb_features); 
                        if(prev_matched_points3d_tmp.size() > 0 && cur_matched_points3d_tmp.size() > 0){
                            #pragma omp parallel for num_threads(NUM_THREADS)
                            for(size_t i = 0; i < prev_matched_points3d_tmp.size(); i++){
                                Eigen::Vector3d prev_point3d = prev_keyframe_from_SW->q_map_cur_tk * Eigen::Vector3d(prev_matched_points3d_tmp[i].x, prev_matched_points3d_tmp[i].y, prev_matched_points3d_tmp[i].z) + prev_keyframe_from_SW->t_map_cur_tk;
                                Eigen::Vector3d cur_point3d = cur_keyframe_from_SW->q_map_cur_tk * Eigen::Vector3d(cur_matched_points3d_tmp[i].x, cur_matched_points3d_tmp[i].y, cur_matched_points3d_tmp[i].z) + cur_keyframe_from_SW->t_map_cur_tk;

                                double distance = (prev_point3d - cur_point3d).norm();
                                if(distance > 0.3){
                                    continue;
                                }
                                Eigen::Vector3d cur_point3d_vector3d = Eigen::Vector3d(cur_matched_points3d_tmp[i].x, cur_matched_points3d_tmp[i].y, cur_matched_points3d_tmp[i].z);
                                
                                ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<FeatureMatchingResidual, 3, 4, 3>( new FeatureMatchingResidual(cur_point3d_vector3d, prev_point3d));
                                {
                                    std::lock_guard<std::mutex> lock(map_mutex_);
                                    problem.AddResidualBlock(cost_function, loss_function, parameters_, parameters_ + 4); 
                                }                              
                                
                            }
                        }
                    }
                }
            }
        }
        if(true){
            pcl::PointCloud<pcl::PointXYZ>::Ptr PlaneCloudDS(new pcl::PointCloud<pcl::PointXYZ>());
            voxel_grid_.setInputCloud(image_handler_->GroundPointOut);        
            voxel_grid_.filter(*image_handler_->GroundPointOut);
            
            
            for(size_t i=0; i < image_handler_->GroundPointOut->points.size(); i++){
                ground_point_sensor_ = image_handler_->GroundPointOut->points[i];
                Eigen::Vector3d point_curr(ground_point_sensor_.x, ground_point_sensor_.y, ground_point_sensor_.z);
                Eigen::Vector3d point_w_tmp = q_w_curr * point_curr + t_w_curr;
                ground_point_world_.x = point_w_tmp.x();
                ground_point_world_.y = point_w_tmp.y();
                ground_point_world_.z = point_w_tmp.z();
                KD_TREE<GroundPlanePointType>::PointVector searchResults; 
                std::vector<float> pointSearchSqDis;
                ikdtree->Nearest_Search(ground_point_world_, 5, searchResults, pointSearchSqDis);
                Eigen::Matrix<double, 5, 3> matA0;
                Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
                if (pointSearchSqDis[4] < 1.0)
                {
                    
                    for (int j = 0; j < 5; j++)
                    {
                        matA0(j, 0) = searchResults[j].x;
                        matA0(j, 1) = searchResults[j].y;
                        matA0(j, 2) = searchResults[j].z;
                    }
                    Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                    double negative_OA_dot_norm = 1 / norm.norm();
                    norm.normalize();
                    bool planeValid = true;
                    for (int j = 0; j < 5; j++)
                    {
                        if (fabs(norm(0) * searchResults[j].x +
                                    norm(1) * searchResults[j].y +
                                    norm(2) * searchResults[j].z + negative_OA_dot_norm) > 0.2)
                        {
                            planeValid = false;
                            break;
                        }
                    }
                    Eigen::Vector3d curr_point(ground_point_sensor_.x, ground_point_sensor_.y, ground_point_sensor_.z);
                    if (planeValid)
                    {
                        ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
                        problem.AddResidualBlock(cost_function, loss_function, parameters_, parameters_ + 4);
                        
                    }
                }
            }

        }
                
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 10;
        options.minimizer_progress_to_stdout = false;
        options.check_gradients = false;
        options.gradient_check_relative_precision = 1e-4;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        ROS_INFO("%s", summary.BriefReport().c_str());
        if(summary.termination_type == ceres::CONVERGENCE){
            transformUpdate();
        }

        if(true){
            if(summary.termination_type == ceres::CONVERGENCE){
                cur_keyframe.q_map_cur_k_ = q_w_curr;
                cur_keyframe.t_map_cur_k_ = t_w_curr;
            }
                
            prev_keyframe = cur_keyframe;
            prev_keyframe_img = image_intensity;
            prev_keypoints_ = cur_keypoints_;
            prev_descriptors_ = cur_descriptors_;
            prev_orb_point_2d_uv_ = cur_orb_point_2d_uv_;
            prev_out_point3d_ = cur_out_point3d_;
            pcl::PointCloud<pcl::PointXYZ> ground_plane_cloud_;
            ground_plane_cloud_ = *image_handler_->GroundPointOut;
            Eigen::Matrix4d cur_transform = Eigen::Matrix4d::Identity();
            cur_transform.block<3,3>(0,0) = cur_keyframe.q_map_cur_k_.toRotationMatrix();
            cur_transform.block<3,1>(0,3) = cur_keyframe.t_map_cur_k_;
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::transformPointCloud(ground_plane_cloud_, *tmp_cloud, cur_transform);
            ikdtree->Add_Points((*tmp_cloud).points, true);

            tmp_cloud->clear();
            pcl::transformPointCloud(pc_corner, *tmp_cloud, cur_transform);
            corner_ikdtree_->Add_Points((*tmp_cloud).points, true);
            if(summary.termination_type == ceres::CONVERGENCE){
                mapKeyframeNew->q_map_cur_tk = q_w_curr;
                mapKeyframeNew->t_map_cur_tk = t_w_curr;               

            }
            keyframe_sliding_window_.emplace_back(mapKeyframeNew);

            if(keyframe_sliding_window_.size() > (size_t)SLIDING_WINDOW_SIZE){
                keyframe_sliding_window_.pop_front();
            }
        }
    }


    
    


    
    










    


    

    


    


    
    
}

//mapOptimization() function
mapOptimization::mapOptimization():
    ikdtree(new KD_TREE<GroundPlanePointType>(0.3, 0.6, 0.4)),//0.4 best
    corner_ikdtree_(new KD_TREE<GroundPlanePointType>(0.3, 0.6, 0.8))
{
    Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters_);
    Eigen::Map<Eigen::Vector3d> t_w_curr(parameters_ + 4);
    //print q_w_curr and t_w_curr
    // ROS_INFO("q_w_curr: %f, %f, %f, %f", q_w_curr.x(), q_w_curr.y(), q_w_curr.z(), q_w_curr.w());
    // ROS_INFO("t_w_curr: %f, %f, %f", t_w_curr.x(), t_w_curr.y(), t_w_curr.z());
    //print the address of parameters_
    // ROS_INFO("parameters_ address: %p", parameters_);
    // ROS_INFO("parameters_+4 address: %p", parameters_+4);

    //ros node handle
    ros::NodeHandle nh;
    nh.getParam("/intensity_feature_tracker/image_width", IMAGE_WIDTH);
    nh.getParam("/intensity_feature_tracker/image_height", IMAGE_HEIGHT);
    //IMAGE_CROP
    nh.getParam("/intensity_feature_tracker/image_crop", IMAGE_CROP);
    //num_threads
    nh.getParam("/intensity_feature_tracker/num_threads", NUM_THREADS);
    // NUM_ORB_FEATURES
    nh.getParam("/intensity_feature_tracker/num_orb_features", NUM_ORB_FEATURES);
    nh.getParam("/intensity_feature_tracker/hand_held_flag", HAND_HELD_FLAG);
    nh.getParam("/map_optimization_parameters/sliding_window_size", SLIDING_WINDOW_SIZE);
    // GROUND_PLANE_WINDOW_SIZE; ground_plane_window_size
    nh.getParam("/map_optimization_parameters/ground_plane_window_size", GROUND_PLANE_WINDOW_SIZE);


    image_handler_ = new intensity_slam::ImageHandler(IMAGE_HEIGHT, 
                                                     IMAGE_WIDTH, 
                                                     NUM_THREADS);
    //initial MASK    
    if(HAND_HELD_FLAG) MASK = setMask(IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CROP);
    else MASK = setMask(IMAGE_HEIGHT, IMAGE_WIDTH, 0);
    matcher_ = cv::BFMatcher(cv::NORM_HAMMING, true);
    keyframe_flag_ = false;




    q_wmap_wodom = Eigen::Quaterniond(1, 0, 0, 0);
    t_wmap_wodom = Eigen::Vector3d(0, 0, 0);
    pubOdomAftMappedHighFrec = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100);//disable factor node
    robot_marker_pub = nh.advertise<visualization_msgs::Marker>("/car_model_Marker_array", 10);
    matched_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/matched_points", 10);
    ground_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/ground_points", 10);
    //pub_matched_lines
    matched_lines_pub = nh.advertise<visualization_msgs::Marker>("/matched_lines", 10);
    // publish image
    matched_keypoints_img_pub = nh.advertise<sensor_msgs::Image>("/matched_keypoints_img", 10);
    voxel_grid_.setLeafSize(0.8, 0.8, 0.8);
    corner_voxel_grid_.setLeafSize(0.4, 0.4, 0.4);

}
mapOptimization::~mapOptimization(){}

// setmask
cv::Mat mapOptimization::setMask(int height, int width, int crop){
    auto mask = cv::Mat(height, width, CV_8UC1, cv::Scalar(255));
    for (int i = 0; i < height; ++i)
        for (int j = 0; j < width; ++j)
            if (j < crop || j > width - crop)
                mask.at<uchar>(i,j) = 0;
    return mask;
}
//void keypoint2uv(const std::vector<cv::KeyPoint>& keypoints, std::vector<cv::Point2f>& uv);
void mapOptimization::keypoint2uv(const std::vector<cv::KeyPoint>& keypoints, std::vector<cv::Point2f>& uv){
    uv.clear();
    for (long unsigned int i = 0; i < keypoints.size(); ++i)
        uv.push_back(keypoints[i].pt);
}

// void extractPointsAndFilterZeroValue
void mapOptimization::extractPointsAndFilterZeroValue(const std::vector<cv::Point2f>& cur_orb_point_2d_uv, const pcl::PointCloud<PointType>::Ptr& cloudTrack, std::vector<cv::Point3f>& cur_out_point3d, std::vector<uchar>& status){
    cur_out_point3d.clear();
    status.clear();
    cur_out_point3d.resize(cur_orb_point_2d_uv.size());
    status.resize(cur_orb_point_2d_uv.size());

    #pragma omp parallel for num_threads(NUM_THREADS)
    for(size_t i=0; i< cur_orb_point_2d_uv.size(); i++){
        int col_id = cvRound(cur_orb_point_2d_uv[i].x);
        int row_id = cvRound(cur_orb_point_2d_uv[i].y);
        int index = row_id * IMAGE_WIDTH + col_id;

        PointType *point_i = &cloudTrack->points[index];

        cv::Point3f p_3d(0.0f, 0.0f, 0.0f);

        if(abs(point_i->x) < 0.01 && abs(point_i->y) < 0.01 && abs(point_i->z) < 0.01){
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


// @in: prev_out_point3d_, cur_out_point3d_
// @out: prev_matched_points3d_, cur_matched_points3d_
void mapOptimization::extractMatchedPoints(std::vector<cv::DMatch> &matches, std::vector<cv::Point3f> &prev_matched_points3d, Eigen::Quaterniond prev_q, Eigen::Vector3d prev_t, std::vector<cv::Point3f> &cur_matched_points3d, Eigen::Quaterniond cur_q, Eigen::Vector3d cur_t, std::vector<cv::Point3f> &prev_out_point3d, std::vector<cv::Point3f> &cur_out_point3d, std_msgs::Header header){
    prev_matched_points3d.clear();
    cur_matched_points3d.clear();
    prev_matched_points3d.resize(matches.size());
    cur_matched_points3d.resize(matches.size());

    #pragma omp parallel for num_threads(NUM_THREADS)
    for(size_t i=0; i<matches.size(); i++){
        int prev_point_index = matches[i].trainIdx; 
        int cur_point_index  = matches[i].queryIdx;
        prev_matched_points3d[i] = prev_out_point3d[prev_point_index];
        cur_matched_points3d[i] = cur_out_point3d[cur_point_index];
    }

    //publish matched points
    if(prev_matched_points3d.size() > 0 && cur_matched_points3d.size() > 0){
        // create PointcloudXYZRGB for cur_matched_points3d and prev_matched_points3d, set the color to red and green respectively.
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cur_matched_points3d_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_matched_points3d_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        cur_matched_points3d_cloud->points.resize(cur_matched_points3d.size());
        prev_matched_points3d_cloud->points.resize(prev_matched_points3d.size());
        #pragma omp parallel for num_threads(NUM_THREADS)
        for(size_t i=0; i<cur_matched_points3d.size(); i++){
            cur_matched_points3d_cloud->points[i].x = cur_matched_points3d[i].x;
            cur_matched_points3d_cloud->points[i].y = cur_matched_points3d[i].y;
            cur_matched_points3d_cloud->points[i].z = cur_matched_points3d[i].z;
            cur_matched_points3d_cloud->points[i].r = 255;
            cur_matched_points3d_cloud->points[i].g = 0;
            cur_matched_points3d_cloud->points[i].b = 0;
        }
        #pragma omp parallel for num_threads(NUM_THREADS)
        for(size_t i=0; i<prev_matched_points3d.size(); i++){
            prev_matched_points3d_cloud->points[i].x = prev_matched_points3d[i].x;
            prev_matched_points3d_cloud->points[i].y = prev_matched_points3d[i].y;
            prev_matched_points3d_cloud->points[i].z = prev_matched_points3d[i].z;
            prev_matched_points3d_cloud->points[i].r = 0;
            prev_matched_points3d_cloud->points[i].g = 255;
            prev_matched_points3d_cloud->points[i].b = 0;
        }

        //transform cur_matched_points3d_cloud to map
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cur_matched(new pcl::PointCloud<pcl::PointXYZRGB>());
        Eigen::Matrix4d cur_transform = Eigen::Matrix4d::Identity();
        cur_transform.block<3,3>(0,0) = cur_q.toRotationMatrix();
        cur_transform.block<3,1>(0,3) = cur_t;
        pcl::transformPointCloud(*cur_matched_points3d_cloud, *cur_matched, cur_transform);


        //transform prev_matched_points3d_cloud to map
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_matched(new pcl::PointCloud<pcl::PointXYZRGB>);
        Eigen::Matrix4d prev_transform = Eigen::Matrix4d::Identity();
        prev_transform.block<3,3>(0,0) = prev_q.toRotationMatrix();
        prev_transform.block<3,1>(0,3) = prev_t;
        pcl::transformPointCloud(*prev_matched_points3d_cloud, *prev_matched, prev_transform);

        // put cur_matched_points3d_cloud and prev_matched_points3d_cloud together.
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr matched_points3d_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        *matched_points3d_cloud = *cur_matched + *prev_matched;
        // publish the matched points3d_cloud.
        if(matched_points_pub.getNumSubscribers() != 0){
            sensor_msgs::PointCloud2 matched_points3d_cloud_msg;
            pcl::toROSMsg(*matched_points3d_cloud, matched_points3d_cloud_msg);
            matched_points3d_cloud_msg.header = header;
            matched_points3d_cloud_msg.header.frame_id = "map";
            matched_points_pub.publish(matched_points3d_cloud_msg);
        }
        
    }
}

// @in: prev_out_point3d_, cur_out_point3d_
// @out: prev_matched_points3d_, cur_matched_points3d_
void mapOptimization::extractMatchedPoints(std::vector<cv::DMatch> &matches, std::vector<cv::Point3f> &prev_matched_points3d, std::vector<cv::Point3f> &cur_matched_points3d,  std::vector<cv::Point3f> &prev_out_point3d, std::vector<cv::Point3f> &cur_out_point3d){
    prev_matched_points3d.clear();
    cur_matched_points3d.clear();
    prev_matched_points3d.resize(matches.size());
    cur_matched_points3d.resize(matches.size());

    #pragma omp parallel for num_threads(NUM_THREADS)
    for(size_t i=0; i<matches.size(); i++){
        int prev_point_index = matches[i].trainIdx; 
        int cur_point_index  = matches[i].queryIdx;
        prev_matched_points3d[i] = prev_out_point3d[prev_point_index];
        cur_matched_points3d[i] = cur_out_point3d[cur_point_index];
    }
    
}

// set initial guess
void mapOptimization::transformAssociateToMap()
{
    Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters_);
    Eigen::Map<Eigen::Vector3d> t_w_curr(parameters_ + 4);
	q_w_curr = q_wmap_wodom * q_wodom_curr;
	t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
}

void mapOptimization::transformUpdate()
{
    Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters_);
    Eigen::Map<Eigen::Vector3d> t_w_curr(parameters_ + 4);
	q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
	t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
}
void mapOptimization::calculateAverageDistance(double &avg_distance, std::vector<cv::Point3f> prev_matched_points3d, Eigen::Quaterniond prev_q, Eigen::Vector3d prev_t, std::vector<cv::Point3f> cur_matched_points3d, Eigen::Quaterniond cur_q, Eigen::Vector3d cur_t){
    double sum_distance = 0;
    double min_distance = std::numeric_limits<double>::max();
    double max_distance = std::numeric_limits<double>::min();
    for(size_t i=0; i<prev_matched_points3d.size(); i++){
        Eigen::Vector3d prev_point3d = prev_q * Eigen::Vector3d(prev_matched_points3d[i].x, prev_matched_points3d[i].y, prev_matched_points3d[i].z) + prev_t;
        Eigen::Vector3d cur_point3d = cur_q * Eigen::Vector3d(cur_matched_points3d[i].x, cur_matched_points3d[i].y, cur_matched_points3d[i].z) + cur_t;
        double distance = (prev_point3d - cur_point3d).norm();
        sum_distance += distance;
        if(distance < min_distance)
            min_distance = distance;
        if(distance > max_distance)
            max_distance = distance;
    }
    avg_distance = sum_distance / prev_matched_points3d.size();

   
    // //ros info the min and max distance
    // ROS_INFO("min_distance: %f", min_distance);
    // ROS_INFO("max_distance: %f", max_distance);
    // ROS_INFO("avg_distance: %f", avg_distance);

}

void mapOptimization::appendLines(visualization_msgs::Marker &line_list, geometry_msgs::Point p1, geometry_msgs::Point p2){
    line_list.points.push_back(p1);
    line_list.points.push_back(p2);
}
void mapOptimization::initialLineList(visualization_msgs::Marker &line_list, std_msgs::Header header){
    // clear line_list
    line_list.points.clear();
    line_list.header = header;
    line_list.header.frame_id = "map";
    line_list.ns = "basic_line";
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.scale.x = 0.005;
    line_list.scale.y = 0.005;
    line_list.scale.z = 0.005;
    line_list.color.a = 1.0;
    line_list.color.r = 0.0;
    line_list.color.g = 0.0;
    line_list.color.b = 1.0;
    line_list.pose.orientation.w = 1.0;


}
void mapOptimization::image_show(std::vector<cv::DMatch> &matches, std::string& detectTime, cv::Mat prev_img, cv::Mat cur_img, std::vector<cv::Point2f> cur_orb_point_2d_uv_, std::vector<cv::Point2f> prev_orb_point_2d_uv_){
    int gap =10; 
    cv::Mat gap_image(gap, prev_img.size().width, CV_8UC1, cv::Scalar(255,255,255));
    cv::Mat img_show;

    cv::vconcat(cur_img, gap_image, img_show); 
    cv::vconcat(img_show, prev_img, img_show);  
    cv::cvtColor(img_show, img_show, cv::COLOR_GRAY2RGB);

    // draw keypoints in current frame 
    for(size_t i = 0; i< (size_t)cur_orb_point_2d_uv_.size(); i++){
        cv::Point2f cur_pt = cur_orb_point_2d_uv_[i];
        cv::circle(img_show, cur_pt, 5, cv::Scalar(0,255,0));
    }
    //  draw keypoints in previous frame
    for(size_t i = 0; i< (size_t)prev_orb_point_2d_uv_.size(); i++){
        cv::Point2f prev_pt = prev_orb_point_2d_uv_[i];
        prev_pt.y += cur_img.size().height + gap;
        cv::circle(img_show, prev_pt, 5, cv::Scalar(0,0,255));
    }

    //draw lines for matches
    for(size_t i = 0; i< (size_t)matches.size(); i++){
        int cur_pt_index = matches[i].queryIdx;
        cv::Point2f cur_pt = cur_orb_point_2d_uv_[cur_pt_index];
        int prev_pt_index = matches[i].trainIdx;
        cv::Point2f prev_pt = prev_orb_point_2d_uv_[prev_pt_index]; 
        prev_pt.y += cur_img.size().height + gap;

        cv::line(img_show, cur_pt, prev_pt, cv::Scalar(0,255,0), 2, 8, 0);
    }

    std::string keypoint_cur_imgtext("cur_img, time cost ms:");
    keypoint_cur_imgtext.append(detectTime);

    std::string match_num("Match num:");
    int match_size = (int)matches.size();
    match_num += std::to_string(match_size);


    cv::putText(img_show, keypoint_cur_imgtext,   cv::Point2f(5, 20 + IMAGE_HEIGHT*0), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);

    cv::putText(img_show, match_num,   cv::Point2f(5, 60 + IMAGE_HEIGHT*0), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);

    cv::putText(img_show, "prev_img",   cv::Point2f(5, 20 + IMAGE_HEIGHT*1 + gap), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);

    if(matched_keypoints_img_pub.getNumSubscribers() > 0){
        cv_bridge::CvImage output_image;
        output_image.header.frame_id = "map";
        output_image.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
        output_image.image = img_show;
        matched_keypoints_img_pub.publish(output_image);
    }
}