#pragma once 

#include "src/base/include/FrameData.h"
#include <src/features/include/FeatureBase.h>

namespace my_robot
{
	class FeatureFinder
	{
		public:
			FeatureFinder();

			bool FindKeypoints(FrameData& firstFrame, FrameData& secondFrame) const;
			
		private:
			std::vector<cv::KeyPoint> GetKeypoints(const cv::Mat& image, const cv::Mat& depth, const std::map<int, cv::KeyPoint>& frameWords) const;
			std::vector<cv::Point3f> GetKeypoints3D(const std::vector<cv::KeyPoint>& kpts, const std::map<int, cv::Point3f>& signatureWords3, const CameraData& cameraData) const;
			cv::Mat GetDescriptors(const cv::Mat& image, std::vector<cv::KeyPoint>& kpts, const std::map<int, cv::Mat>& descriptors, bool& generated) const;

			int _featureType;
			int _featureMatching;
			float _featureMaxDist;

			features::FeatureBase* _detector;
	};

}