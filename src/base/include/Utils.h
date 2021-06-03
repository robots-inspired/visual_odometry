#pragma once

#include "src/camera/include/CameraData.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

namespace my_robot
{
	namespace utils
	{
		const std::vector<cv::Point3f> projectTo3d(const CameraData& cameraData, const std::vector<cv::KeyPoint>& keypoints);

		cv::Point3f transformPoint(const cv::Point3f& point, const Eigen::Matrix4f& transform);

		bool isFinite(const cv::Point3f& point);
	}
}