#include "src/base/include/Transform.h"

namespace my_robot 
{    
	Transform Transform::operator* (const Transform& t) const
	{
		Eigen::Affine3f m = Eigen::Affine3f(getMatrix() * t.getMatrix());
		m.linear() = Eigen::Quaternionf(m.linear()).normalized().toRotationMatrix();

		return Transform(m.matrix());
	}
}