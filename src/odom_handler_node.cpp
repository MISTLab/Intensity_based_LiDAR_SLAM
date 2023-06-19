#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h> 
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <eigen3/Eigen/Dense>

ros::Publisher pubMergedOdometry;
Eigen::Quaterniond aloam_q_wodom_prev, intensity_q_wodom_prev, q_odom;
Eigen::Vector3d aloam_t_wodom_prev, intensity_t_wodom_prev, t_odom;
Eigen::Matrix4d odom_cur, aloam_odom_cur, intensity_odom_cur, aloam_prev, intensity_prev;
bool aloam_odom_init = false, intensity_odom_init = false;
std::string skip_flag; 

void callback(const nav_msgs::Odometry::ConstPtr& aloam_odom, const nav_msgs::Odometry::ConstPtr& intensity_odom)
{
    Eigen::Quaterniond aloam_q_wodom_curr, intensity_q_wodom_curr;
    Eigen::Vector3d aloam_t_wodom_curr, intensity_t_wodom_curr;
	aloam_q_wodom_curr.x() = aloam_odom->pose.pose.orientation.x;
	aloam_q_wodom_curr.y() = aloam_odom->pose.pose.orientation.y;
	aloam_q_wodom_curr.z() = aloam_odom->pose.pose.orientation.z;
	aloam_q_wodom_curr.w() = aloam_odom->pose.pose.orientation.w;
	aloam_t_wodom_curr.x() = aloam_odom->pose.pose.position.x;
	aloam_t_wodom_curr.y() = aloam_odom->pose.pose.position.y;
	aloam_t_wodom_curr.z() = aloam_odom->pose.pose.position.z;
    aloam_odom_cur.block<3,3>(0,0) = aloam_q_wodom_curr.toRotationMatrix();
    aloam_odom_cur.block<3,1>(0,3) = aloam_t_wodom_curr;
    aloam_odom_cur.block(3,0,1,4) << 0, 0, 0, 1;

    intensity_q_wodom_curr.x() = intensity_odom->pose.pose.orientation.x;
	intensity_q_wodom_curr.y() = intensity_odom->pose.pose.orientation.y;
	intensity_q_wodom_curr.z() = intensity_odom->pose.pose.orientation.z;
	intensity_q_wodom_curr.w() = intensity_odom->pose.pose.orientation.w;
	intensity_t_wodom_curr.x() = intensity_odom->pose.pose.position.x;
	intensity_t_wodom_curr.y() = intensity_odom->pose.pose.position.y;
	intensity_t_wodom_curr.z() = intensity_odom->pose.pose.position.z;
    skip_flag = intensity_odom->child_frame_id;
    intensity_odom_cur.block<3,3>(0,0) = intensity_q_wodom_curr.toRotationMatrix();
    intensity_odom_cur.block<3,1>(0,3) = intensity_t_wodom_curr;
    intensity_odom_cur.block(3,0,1,4) << 0, 0, 0, 1;

    if(!aloam_odom_init && !intensity_odom_init){
        aloam_prev = aloam_odom_cur;
        intensity_prev = intensity_odom_cur;
        odom_cur = intensity_odom_cur;
        aloam_odom_init = true;
        intensity_odom_init = true;
    }
    else{
        Eigen::Matrix4d intensity_odom_diff = intensity_prev.inverse() * intensity_odom_cur;
        Eigen::Matrix4d aloam_odom_diff = aloam_prev.inverse() * aloam_odom_cur; 
        if( skip_flag == "/odom_skip"){
            std::cout << "intensity_odom_diff: " << intensity_odom_diff << std::endl;
            odom_cur = odom_cur * aloam_odom_diff;
        }
        else{
            odom_cur = odom_cur * intensity_odom_diff;
        } 
        aloam_prev = aloam_odom_cur;
        intensity_prev = intensity_odom_cur;
        
    }
    Eigen::Matrix3d rot_cur = odom_cur.block(0,0,3,3);
    Eigen::Vector3d t_cur = odom_cur.block(0,3,3,1);
    Eigen::Quaterniond q_cur(rot_cur); 
    nav_msgs::Odometry odom;
    odom.header.stamp = aloam_odom->header.stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = "laser_odom";
    odom.pose.pose.orientation.x = q_cur.x();
    odom.pose.pose.orientation.y = q_cur.y();
    odom.pose.pose.orientation.z = q_cur.z();
    odom.pose.pose.orientation.w = q_cur.w();
    odom.pose.pose.position.x = t_cur.x();
    odom.pose.pose.position.y = t_cur.y();
    odom.pose.pose.position.z = t_cur.z();
    pubMergedOdometry.publish(odom); 
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "map_optimization");
    ros::NodeHandle nh;
    pubMergedOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);

    message_filters::Subscriber<nav_msgs::Odometry> aloam_odom_sub(nh, "/laser_odom_to_init_aloam", 10);
    message_filters::Subscriber<nav_msgs::Odometry> intensity_odom_sub(nh, "/laser_odom_to_init_intensity", 10);

    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), aloam_odom_sub, intensity_odom_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2)); 
    ros::spin();
    return 0;  
}