//#ifndef IMAGE_PROCESSING_TIMING_H
//#define IMAGE_PROCESSING_TIMING_H

#include <cmath>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Path.h>

class ImageProcessingTiming
{
private:
    ros::NodeHandle nh_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tfListener_;
    image_transport::ImageTransport it_;
    ros::Publisher orig_pub_, proc_pub_;
  
  
    void orig_msg_cb(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::CameraInfoConstPtr& cam_info)
    {
        ros::Time rostime = ros::Time::now();
        ros::WallTime walltime = ros::WallTime::now();
        publishTimeInfo(image->header, rostime, walltime, orig_pub_, "orig");
    }


    void proc_msg_cb(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::CameraInfoConstPtr& cam_info)
    {
        ros::Time rostime = ros::Time::now();
        ros::WallTime walltime = ros::WallTime::now();
        publishTimeInfo(image->header, rostime, walltime, proc_pub_, "proc");
    }


    void publishTimeInfo(const std_msgs::Header& header, const ros::Time& rostime, const ros::WallTime& walltime, ros::Publisher& pub, const std::string& source)
    {
        ROS_INFO_STREAM("Received callback from " << source);
        
        nav_msgs::Path path;
        path.header = header;
        
        geometry_msgs::PoseStamped rostimePose;
        rostimePose.header.stamp = rostime;
        
        geometry_msgs::PoseStamped walltimePose;
        walltimePose.header.stamp = ros::Time(walltime.sec, walltime.nsec);
        
        path.poses.push_back(rostimePose);
        path.poses.push_back(walltimePose);
        
        nav_msgs::PathConstPtr path_ptr = boost::make_shared<nav_msgs::Path>(path);
        
        pub.publish(path);
    }

    void results_msg_cb(const nav_msgs::PathConstPtr &orig_result, const nav_msgs::PathConstPtr &proc_result)
    {
        ros::Time baseline = orig_result->header.stamp;
        
        ros::Time orig_ros = orig_result->poses[0].header.stamp;
        ros::Time orig_wall = orig_result->poses[1].header.stamp;
        
        ros::Time proc_ros = proc_result->poses[0].header.stamp;
        ros::Time proc_wall = proc_result->poses[1].header.stamp;
        
        double ros_dt = (proc_ros - orig_ros).toNSec()/1E6;
        double wall_dt = (proc_wall - orig_wall).toNSec()/1E6;
        
      
        ROS_INFO_STREAM("Received result callback [" << baseline << "] ROS: " << ros_dt << "ms, WALL: " << wall_dt << "ms"); 
        
    }


public:
    void init()
    {
      int queue_ = 20;
      
      std::string orig_timing_topic = "/orig_timing";
      std::string proc_timing_topic = "/proc_timing";
      
      
      ROS_INFO("ImageProcessingTiming Node Initialized");
      
      message_filters::Subscriber<sensor_msgs::Image> origImageSub(nh_, "/orig_image_in", queue_);
      message_filters::Subscriber<sensor_msgs::CameraInfo> origInfoSub(nh_, "/orig_info_in", queue_);
      message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> origTimeSynchronizer(origImageSub, origInfoSub, queue_);
      origTimeSynchronizer.registerCallback(boost::bind(&ImageProcessingTiming::orig_msg_cb, this, _1, _2));
      orig_pub_ = nh_.advertise<nav_msgs::Path>(orig_timing_topic,queue_);
      
      message_filters::Subscriber<sensor_msgs::Image> procImageSub(nh_, "/proc_image_in", queue_);
      message_filters::Subscriber<sensor_msgs::CameraInfo> procInfoSub(nh_, "/proc_info_in", queue_);
      message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> procTimeSynchronizer(procImageSub, procInfoSub, queue_);
      procTimeSynchronizer.registerCallback(boost::bind(&ImageProcessingTiming::proc_msg_cb, this, _1, _2));
      proc_pub_ = nh_.advertise<nav_msgs::Path>(proc_timing_topic,queue_);
      
      message_filters::Subscriber<nav_msgs::Path> origTimeSub(nh_, orig_timing_topic, queue_);
      message_filters::Subscriber<nav_msgs::Path> procTimeSub(nh_, proc_timing_topic, queue_);
      message_filters::TimeSynchronizer<nav_msgs::Path, nav_msgs::Path> resultsTimeSynchronizer(origTimeSub, procTimeSub, queue_);
      resultsTimeSynchronizer.registerCallback(boost::bind(&ImageProcessingTiming::results_msg_cb, this, _1, _2)); 
      
      // This is needed as long as the synchronizers are local to this function
      ros::AsyncSpinner spinner(4);
      spinner.start();
      
      ros::waitForShutdown();
      
      //ros::spin(); 
      
    }


    ImageProcessingTiming(ros::NodeHandle& nh) :
            nh_(nh),
            tfListener_(buffer_), 
            it_(nh_)
    {
          
        
    }

};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "image_processing_timing");

    ros::NodeHandle nh;
    ImageProcessingTiming s(nh);
    s.init();

    ros::spin();


    return 0;
}
