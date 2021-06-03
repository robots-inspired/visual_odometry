#pragma once

#include <src/odometry/include/Odometry.h>
#include <src/base/include/FrameData.h>
#include <src/camera/include/CameraData.h>

namespace my_robot 
{
	class  OdometryF2F : public Odometry
	{
		public:
			OdometryF2F();

		private:
			virtual Transform calculateMotion(const CameraData& data);
			void ResetKeyframe(const CameraData& data, FrameData& newFrame);

			int _frameMinInliers;
			Transform _keyFramePos;
			FrameData _keyFrameData;
			FrameData _currentFrameData;
	};

}