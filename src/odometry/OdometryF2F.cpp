#include "src/odometry/include/OdometryF2F.h"

namespace my_robot 
{
	OdometryF2F::OdometryF2F() :  Odometry(),
		_frameMinInliers(200),
		_keyFramePos(Transform())
	{
	}

	Transform OdometryF2F::calculateMotion(const CameraData& data)
	{
		Transform output;
		FrameData newFrame(data, data.getCameraModel());
		int inliers = -1;

		if (_keyFrameData.cameraModel().isCalibrated())
		{
			FrameData tmpRefFrame = _keyFrameData;

			if (_featureFinder->FindKeypoints(tmpRefFrame, newFrame))
			{
				output = _motionEstimator->computeMotion(tmpRefFrame, newFrame, inliers);
			}
		}
		else
		{
			output = Transform::identity();
		}
		
		if (!output.isNull())
		{
			if (_keyFramePos.isNull())
			{
				_keyFramePos = getPose();
			}
			else
			{
				Transform motionSinceLastKeyFrame = _keyFramePos.inverse() * getPose();
				output = motionSinceLastKeyFrame.inverse() * output;
			}
			
			if (inliers <= _frameMinInliers)
			{
				ResetKeyframe(data, newFrame);
			}
		}
		else
		{
			ResetKeyframe(data, newFrame);
		}

		return output;
	}

	void OdometryF2F::ResetKeyframe(const CameraData& data, FrameData& newFrame)
	{
		int features = newFrame.get2dPoints().size();
		
		if (features == 0)
		{
			newFrame = FrameData(data, data.getCameraModel());

			FrameData tempFrame;
			_featureFinder->FindKeypoints(newFrame, tempFrame);
			features = (int)newFrame.get2dPoints().size();
		}

		if (features >= 100)
		{
			_keyFrameData = newFrame;
			_keyFramePos = Transform();
		}
	}
}
