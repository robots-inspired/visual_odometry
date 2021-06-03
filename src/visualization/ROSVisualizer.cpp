#include "src/visualization/include/ROSVisualizer.h"

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

#include <opencv2/core/hal/interface.h>
#include <opencv2/xfeatures2d.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> CloudType;

namespace my_robot
{
    ROSVisualizer::ROSVisualizer()
    {
        _nodeHandle = ros::NodeHandle();
        _transformBroadcaster = tf2_ros::TransformBroadcaster(); 
        _publisherMapWords = _nodeHandle.advertise<CloudType>("key_frames", 1);
    }

    void ROSVisualizer::showKeypoints(const cv::Mat& image, const std::map<int, cv::KeyPoint>& kpts)
    {
        std::vector<cv::KeyPoint> keypoints;
        for (auto kk = kpts.begin(); kk != kpts.end(); ++kk)
        {
            keypoints.push_back(kk->second);            
        }
        
        cv::Mat outImg;
        cv::drawKeypoints(image, keypoints, outImg, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imshow("outImg", outImg);
        cv::waitKey(1);
    }

    void ROSVisualizer::showMatches(const cv::Mat& image, const std::map<int, cv::KeyPoint>& kpts1, const std::map<int, cv::KeyPoint>& kpts2)
    {
        if (image.rows < 1)
            return;
            
        cv::Mat outImg;
        cv::cvtColor(image, outImg, cv::COLOR_GRAY2BGR);
        
        cv::RNG rng(12345);

        for (auto kptIter = kpts1.begin(); kptIter != kpts1.end(); ++kptIter)
        {
            int id = kptIter->first;

            auto math = kpts2.find(id);

            if (math != kpts2.end())
            {
                cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
                cv::circle(outImg, kptIter->second.pt, 3, color);
                cv::line(outImg, kptIter->second.pt, math->second.pt, color);
                cv::circle(outImg, math->second.pt, 6, color);
            }
        }

        cv::imshow("outImg", outImg);
        cv::waitKey(1);
    }

    void ROSVisualizer::publishTransform(Eigen::Vector3f pos, Eigen::Quaternionf rot, const std::string& frameId)
    {
        geometry_msgs::TransformStamped stamp;
        stamp.header.stamp = ros::Time::now(); 
        stamp.header.frame_id = "map" ;
        stamp.child_frame_id = frameId;
        stamp.transform.translation.x = pos[0]; 
        stamp.transform.translation.y = pos[1]; 
        stamp.transform.translation.z = pos[2]; 
        stamp.transform.rotation.x = rot.x();
        stamp.transform.rotation.y = rot.y();
        stamp.transform.rotation.z = rot.z();
        stamp.transform.rotation.w = rot.w();
        _transformBroadcaster.sendTransform(stamp);
    }

    void ROSVisualizer::publishMapWords(pcl::PointCloud<pcl::PointXYZ>::Ptr points)
    {
        if (_publisherMapWords.getNumSubscribers() == 0)
        {
            return;
        }

        pcl_conversions::toPCL(ros::Time::now(), points->header.stamp);
        points->header.frame_id = "map";
        points->width = points->size();
        points->height = 1;
        points->is_dense = true;

        _publisherMapWords.publish(points);
    }
}