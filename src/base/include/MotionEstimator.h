#pragma once 

#include <src/base/include/Transform.h>
#include "src/base/include/FrameData.h"

namespace my_robot 
{
	class MotionEstimator
	{
		public:
			MotionEstimator();

			Transform computeMotion(FrameData& firstFrame, FrameData& secondFrame, int& inliers) const;
			Transform estimatePnP(const std::map<int, cv::Point3f>& prevWords3d, const std::map<int, cv::KeyPoint>& currentWords2d, const CameraModel& cameraModel, int& inliersCountOut) const;

		private:
			int _minInliers;
			int _iterations;
			int _flags;
			float _reprojError;
	};

}