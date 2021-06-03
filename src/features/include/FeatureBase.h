#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <list>

#include "src/camera/include/CameraData.h"

namespace my_robot
{
	namespace features
	{
		class FeatureBase
		{
			public:
				std::vector<cv::KeyPoint> detect(const cv::Mat & image, const cv::Mat & mask = cv::Mat()) const;
				cv::Mat compute(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

			private:
				virtual std::vector<cv::KeyPoint> detectOverride(const cv::Mat & image, const cv::Mat & mask = cv::Mat()) const = 0;
				virtual cv::Mat computeOverride(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const = 0;
		};
	}
}