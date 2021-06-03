#include <src/features/include/ORB.h>

namespace my_robot 
{
	namespace features 
	{
		ORB::ORB()
		{
			_detector = cv::ORB::create(2000, 1.2, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
		}

		std::vector<cv::KeyPoint> ORB::detectOverride(const cv::Mat & image, const cv::Mat & mask) const
		{
			std::vector<cv::KeyPoint> keypoints;
			_detector->detect(image, keypoints, mask);
			return keypoints;
		}

		cv::Mat ORB::computeOverride(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const
		{
			cv::Mat descriptors;
			_detector->compute(image, keypoints, descriptors);
			return descriptors;
		}

	}
}