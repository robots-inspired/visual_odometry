#pragma once

#include "src/features/include/FeatureBase.h"

namespace my_robot
{
	namespace features
	{
		class  SIFT : public FeatureBase
		{
			public:
				SIFT();

			private:
				virtual std::vector<cv::KeyPoint> detectOverride(const cv::Mat & image, const cv::Mat & mask = cv::Mat()) const;
				virtual cv::Mat computeOverride(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

				cv::Ptr<cv::SIFT> _detector;
		};
	}
}