#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace my_robot
{    
    class ROSVisualizer
    {
        public:
            ROSVisualizer();

            void showKeypoints(const cv::Mat& image, const std::map<int, cv::KeyPoint>& kpts);
            void showMatches(const cv::Mat& image, const std::map<int, cv::KeyPoint>& kpts1, const std::map<int, cv::KeyPoint>& kpts2);
            void publishTransform(Eigen::Vector3f pos, Eigen::Quaternionf rot, const std::string& frameId);
            void publishMapWords(pcl::PointCloud<pcl::PointXYZ>::Ptr points);

        private:
            ros::NodeHandle _nodeHandle;
            tf2_ros::TransformBroadcaster _transformBroadcaster; 
            ros::Publisher _publisherMapWords;
    };
}