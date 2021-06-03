#include "src/base/include/Utils.h"

namespace my_robot
{
	namespace utils
	{
		const std::vector<cv::Point3f> projectTo3d(const CameraData& cameraData, const std::vector<cv::KeyPoint>& keypoints)
		{
			std::vector<cv::Point3f> keypoints3d(keypoints.size());
			cv::Mat depth = cameraData.getDepth();

			if (!depth.empty())
			{
				auto cameraModel = cameraData.getCameraModel();
				float nanVal = std::numeric_limits<float>::quiet_NaN();

				cv::Point3f nanPt(nanVal, nanVal, nanVal);

				for (int i = 0; i < keypoints.size(); ++i)
				{
					cv::Point3f point3d = cameraModel.projectPoint(depth, keypoints[i].pt.x, keypoints[i].pt.y);
					keypoints3d.at(i) = utils::isFinite(point3d) ? point3d : nanPt;
				}
			}

			return keypoints3d;
		}

		cv::Point3f transformPoint(const cv::Point3f& point, const Eigen::Matrix4f& transform)
		{
			cv::Point3f ret = point;
			ret.x = transform(0, 0) * point.x + transform(0, 1) * point.y + transform(0, 2) * point.z + transform(0, 3);
			ret.y = transform(1, 0) * point.x + transform(1, 1) * point.y + transform(1, 2) * point.z + transform(1, 3);
			ret.z = transform(2, 0) * point.x + transform(2, 1) * point.y + transform(2, 2) * point.z + transform(2, 3);

			return ret;
		}

		bool isFinite(const cv::Point3f& point)
		{
			return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
		}
	}
}