#include "loop_closure_handler.h"

loopClosureHandler::loopClosureHandler(/* args */)
{
    std::string PROJECT_NAME("intensity_feature_tracker");
    std::string pkg_path = ros::package::getPath(PROJECT_NAME);
    std::string vocabulary_file("/config/orbvoc.dbow3");
    vocabulary_file = pkg_path + vocabulary_file;
    voc_= new DBoW3::Vocabulary(vocabulary_file); 
    db_.setVocabulary(*voc_, false, 0);
    loop_index_ = -1;
    kdtreeKeyframeNearSearch_.reset(new pcl::KdTreeFLANN<PointType>());
}

loopClosureHandler::~loopClosureHandler()
{
}

sensor_msgs::ImagePtr loopClosureHandler::cvMat2Image(std_msgs::Header header, cv::Mat & image){
    static cv_bridge::CvImage outImg; 
    outImg.header = header;
    outImg.encoding = "bgr8";
    outImg.image = image;
    auto imageOut = outImg.toImageMsg();
    return imageOut; 
}

void loopClosureHandler::keyframeProcessor(loopClosureProcessor::Keyframe keyframe, pcl::PointCloud<PointType>::Ptr cloud_keyPose3D){
    keyframe_pool_[keyframe.keyframeId] = keyframe; 
    loop_index_ = -1;
    // ROS_INFO cloud_keyPose3D->size(), cloud_keyPose6D->size();
    if(cloud_keyPose3D->size() == 0){
        return;
    }
    // ROS_INFO("size of keyframe: %li", cloud_keyPose3D->size());
    PointType cur_pose;
    {
        std::lock_guard<std::mutex> lock(kdtreeKeyframeNearSearch_mutex_);
        kdtreeKeyframeNearSearch_->setInputCloud(cloud_keyPose3D); 
        cur_pose = cloud_keyPose3D->points[cloud_keyPose3D->size()-1];
    }
        
    
    std::vector<int> keyframe_near_search_indices;
    std::vector<float> keyframe_near_search_distances;
    kdtreeKeyframeNearSearch_->radiusSearch(cur_pose, 7, keyframe_near_search_indices, keyframe_near_search_distances);
    // ROS_INFO("size of keyframe_near_search_indices: %li", keyframe_near_search_indices.size());
    for(size_t i = 0; i< keyframe_near_search_indices.size(); i++){        
        int id_tmp = keyframe_near_search_indices[i];
        auto loop_keyframe = keyframe_pool_.find(id_tmp);
        // ROS_INFO("index: %i, distance: %f, interval time: %f", keyframe_near_search_indices[i], keyframe_near_search_distances[i], loop_keyframe->second.time - keyframe.time);
        if(loop_keyframe != keyframe_pool_.end()){
            if(abs(loop_keyframe->second.time - keyframe.time) > 40){ // loop closure time threshold 15 seconds
                loop_index_ = id_tmp;
                ROS_INFO("loop index: %i, cur index: %li", loop_index_, keyframe.keyframeId);
                break; 
            }
        }

    }


}

void loopClosureHandler::keyframeProcessor(loopClosureProcessor::Keyframe keyframe){
    keyframe_pool_[keyframe.keyframeId] = keyframe; 
    loop_index_ = -1;

    bool USE_SCANCONTEXT = false;
    bool USE_KDTREE = false; 
    bool USE_ORBLOOP = true;
    // scan context
    if(USE_SCANCONTEXT){
        scManager.makeAndSaveScancontextAndKeys(keyframe.cloud_track_);
        auto detectResult = scManager.detectLoopClosureID();
        loop_index_ = detectResult.first;
    }

    if(USE_ORBLOOP){
        loop_index_ = detectLoop(keyframe.image, keyframe.descriptors, keyframe.keyframeId);//false: -1; detect loop: loop_index >=1
    }

    if(USE_KDTREE){
        // kdtree based loop detection
        
        
    }


        
    
    
    
    if(loop_index_ >= 0){
        std::cout << "\n Current id:" << keyframe.keyframeId << ", candidate id: " << loop_index_ << std::endl; 
    }


}


int loopClosureHandler::detectLoop(cv::Mat & image, cv::Mat & descriptors, int frame_index){
    DBoW3::QueryResults ret;
    int MIN_LOOP_SEARCH_GAP = 50;
    double MIN_LOOP_BOW_TH = 0.015;
    int SKIPED_FRAMES = 5;

    ros::NodeHandle nh; 
    nh.param<double>("/loop_closure_parameters/min_loop_bow_threshold", MIN_LOOP_BOW_TH, 0.0155);
    nh.param<int>("/loop_closure_parameters/min_loop_search_gap", MIN_LOOP_SEARCH_GAP, 50);
    nh.param<int>("/loop_closure_parameters/skiped_frames", SKIPED_FRAMES, 5);
    
    db_.query(descriptors, ret, 4, frame_index - MIN_LOOP_SEARCH_GAP);
    db_.add(descriptors);

    image_pool_[frame_index] = image.clone();
    cv::Mat bow_images = image.clone();

    if (frame_index - MIN_LOOP_SEARCH_GAP < 0)
        return -1;

    bool find_loop = false;
    if (ret.size() >= 1 && ret[0].Score > MIN_LOOP_BOW_TH)
    {
        for (unsigned int i = 1; i < ret.size(); i++)
        {
            ROS_INFO("ret [%i] score is %f", i, ret[i].Score);
            if (ret[i].Score > MIN_LOOP_BOW_TH)
            {          
                find_loop = true;
            }
        }
    }
    
    if (find_loop && frame_index > 5)
    {
        int min_index = -1;
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            if (min_index == -1 || ((int)ret[i].Id < min_index && ret[i].Score > 0.015 ))
                min_index = ret[i].Id;
        }
        ROS_INFO("find loop: %i", min_index);
        if(min_index < 6){
            return min_index;
        }
        else{
            return -1; 
        }
        
        
    }
    else
        return -1;

}