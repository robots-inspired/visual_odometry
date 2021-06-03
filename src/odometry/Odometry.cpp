#include "src/odometry/include/Odometry.h"
#include "src/odometry/include/OdometryF2F.h"
#include "src/odometry/include/OdometryF2M.h"

namespace my_robot 
{
	Odometry::Odometry():
		_pose(Transform::identity()),
		_featureFinder(new FeatureFinder()),
		_motionEstimator(new MotionEstimator())
	{
	}

	Odometry::~Odometry()
	{
		delete _featureFinder;
		delete _motionEstimator;
	}

	Transform Odometry::process(const CameraData& data)
	{		
		Transform transform = this->calculateMotion(data);

		if (transform.isNull())
		{
			return _pose;
		}

		_pose = _pose * transform;

		return _pose;
	}
} 