#pragma once

#include "src/odometry/include/Odometry.h"
#include "src/odometry/include/OdometryF2MData.h"

namespace my_robot 
{
	struct MapData;

	class OdometryF2M : public Odometry
	{
		public:
			OdometryF2M();
			~OdometryF2M();

		private:
			virtual Transform calculateMotion(const CameraData& data);

			int _frameMinInliers;
			int _maximumMapSize;

			MapData* _map;
	};
}