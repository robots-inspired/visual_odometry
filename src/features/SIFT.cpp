#include <src/features/include/SIFT.h>

namespace my_robot 
{
	namespace features 
	{
		SIFT::SIFT()
		{
			_detector = cv::SIFT::create(2000, 3, 0.04, 10, 1.6);
		}

		std::vector<cv::KeyPoint> SIFT::detectOverride(const cv::Mat & image, const cv::Mat & mask) const
		{
			std::vector<cv::KeyPoint> keypoints;
			_detector->detect(image, keypoints, mask);

			return keypoints;
		}

		cv::Mat SIFT::computeOverride(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const
		{
			cv::Mat descriptors;
			_detector->compute(image, keypoints, descriptors);
			return descriptors;
		}
	}
}