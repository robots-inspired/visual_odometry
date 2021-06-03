#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

// warp for Eigen::Matrix4f
// 3x3 rotation + 3x1 translation

namespace my_robot 
{
	class Transform
	{
        public:
            static Transform identity()
            {
                auto data = Eigen::Matrix4f();
                data << 1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1, 0,
                        0, 0, 0, 1;

                return Transform(data);
            }

        public:

            Transform() : _data(Eigen::Matrix4f::Zero()) {    }
            Transform(const Eigen::Matrix4f& data) : _data(data) {  }

            Eigen::Matrix4f getMatrix() const 
            { 
                return _data;
            }

            Eigen::Affine3f toAffine() const 
            { 
                return Eigen::Affine3f(_data);
            }

            Eigen::Vector3f getTranslation() const
            {
                return toAffine().translation();
            }

            Eigen::Quaternionf getQuaternion() const
            {
                return Eigen::Quaternionf(toAffine().linear()).normalized();
            }

            bool isNull() const
            {
                return (_data(0, 0) == 0.0f && _data(0, 1) == 0.0f && _data(0, 2) == 0.0f && _data(0, 3) == 0.0f &&
                        _data(1, 0) == 0.0f && _data(1, 1) == 0.0f && _data(1, 2) == 0.0f && _data(1, 3) == 0.0f &&
                        _data(2, 0) == 0.0f && _data(2, 1) == 0.0f && _data(2, 2) == 0.0f && _data(2, 3) == 0.0f) ||
                        std::isnan(_data(0, 0)) || std::isnan(_data(0, 1)) || std::isnan(_data(0, 2)) || std::isnan(_data(0, 3)) ||
                        std::isnan(_data(1, 0)) || std::isnan(_data(1, 1)) || std::isnan(_data(1, 2)) || std::isnan(_data(1, 3)) ||
                        std::isnan(_data(2, 0)) || std::isnan(_data(2, 1)) || std::isnan(_data(2, 2)) || std::isnan(_data(2, 3));
	        }

            Transform inverse() const
            {
        		return Transform(_data.inverse());
            }

            Transform operator*(const Transform & t) const;

        private:
            Eigen::Matrix4f _data;
	};
}