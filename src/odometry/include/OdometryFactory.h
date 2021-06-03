#pragma once

#include <src/odometry/include/Odometry.h>
#include <src/odometry/include/OdometryF2F.h>
#include <src/odometry/include/OdometryF2M.h>

namespace my_robot 
{
	class OdometryFactory
	{
		public:
			enum Type 
			{
				F2F = 0,
				F2M = 1
			};

			static Odometry* create(Type type = Type::F2F)
			{
				switch (type)
				{
					case Type::F2F:
						return new OdometryF2F();
					case Type::F2M:
						return new OdometryF2M();
					default:
						return 0;
				} 
			}

	};

}