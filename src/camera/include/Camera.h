#pragma once

#include "src/camera/include/CameraData.h"

namespace my_robot
{
	class Camera
	{
		public:
			virtual bool init() = 0;
			virtual CameraData getFrame() = 0;
			virtual bool isReadyToUse() = 0;
	};

}
