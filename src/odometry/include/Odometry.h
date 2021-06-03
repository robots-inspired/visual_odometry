#pragma once

#include <src/base/include/Transform.h>
#include <src/camera/include/CameraData.h>
#include "src/base/include/FeatureFinder.h"
#include "src/base/include/MotionEstimator.h"

namespace my_robot 
{
	class Odometry
	{
		public:
			enum Type {
					F2F = 0,
					F2M = 1
				};

			Odometry();
			virtual ~Odometry();

			Transform process(const CameraData& data);

			const Transform& getPose() const { return _pose; }

		protected:
			FeatureFinder* _featureFinder;
			MotionEstimator* _motionEstimator;
			Transform _pose;
			
		private:
			virtual Transform calculateMotion(const CameraData& data) = 0;


	};

}