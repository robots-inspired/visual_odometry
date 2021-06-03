#pragma once

#include "src/features/include/FeatureBase.h"
#include "src/features/include/SIFT.h"
#include "src/features/include/ORB.h"

namespace my_robot
{
	namespace features
	{
		class FeatureFactory
		{
			public:
				enum Type 
				{
					Sift = 0,
					Orb = 1
				};

				static FeatureBase * create(Type type = Type::Orb)
				{
					switch (type)
					{
						case Type::Sift:
							return new SIFT();
						default:
							return new ORB();
					}
				}
		};
	}
}