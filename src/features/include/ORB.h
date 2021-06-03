#pragma once

#include "src/features/include/FeatureBase.h"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <vector>

namespace my_robot
{
	namespace features
	{
        class ORB : public FeatureBase
        {
            public:
                ORB();

            private:
                virtual std::vector<cv::KeyPoint> detectOverride(const cv::Mat & image, const cv::Mat & mask = cv::Mat()) const;
                virtual cv::Mat computeOverride(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

                cv::Ptr<cv::ORB> _detector;
        };
    }
}