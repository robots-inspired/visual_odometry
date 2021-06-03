
#pragma once

#include "src/camera/include/Camera.h"
#include "src/camera/include/CameraRealSense2.h"

namespace my_robot
{
	class CameraFactory
	{
		public:
			enum Type 
			{ 
				Realsense2 = 0 
			};

			static Camera* create(Type type = Type::Realsense2)
			{
				switch (type)
				{
					case Type::Realsense2:
						return new CameraRealSense2();
					default:
						return 0;
				}
			}
	};
}