#include "src/features/include/FeatureBase.h"

namespace my_robot 
{
	namespace features 
	{
		std::vector<cv::KeyPoint> FeatureBase::detect(const cv::Mat & image, const cv::Mat& mask) const
		{
			return detectOverride(image, mask);
		}

		cv::Mat FeatureBase::compute(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const
		{
			return computeOverride(image, keypoints);
		}
	}
}