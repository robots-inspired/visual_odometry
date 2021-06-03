#include "src/base/include/MotionEstimator.h"
#include "src/base/include/Utils.h"
#include <opencv2/calib3d/calib3d_c.h>

namespace my_robot 
{
	MotionEstimator::MotionEstimator() :
		_flags(cv::SOLVEPNP_EPNP),
		_minInliers(20),
		_iterations(1000),
		_reprojError(1.99) 
	{	
	}

	Transform MotionEstimator::computeMotion(FrameData& firstFrame, FrameData & secondFrame, int& inliers) const
	{		
		if (firstFrame.get3dPoints().size() >= _minInliers && secondFrame.get2dPoints().size() >= _minInliers)
		{
			auto transform = estimatePnP(firstFrame.get3dPoints(), secondFrame.get2dPoints(), firstFrame.cameraModel(), inliers);

			if (!transform.isNull())
			{
				return transform;
			}
		}

		inliers = -1;
		return Transform();
	}

	Transform MotionEstimator::estimatePnP(const std::map<int, cv::Point3f>& prevWords3d, const std::map<int, cv::KeyPoint>& currentWords2d, const CameraModel& cameraModel, int& inliersCountOut) const
	{
		std::vector<cv::Point3f> points3d;
		std::vector<cv::Point2f> imagePoints;

		for (auto iter = currentWords2d.begin(); iter != currentWords2d.end(); ++iter)
		{
			std::map<int, cv::Point3f>::const_iterator point3d = prevWords3d.find(iter->first);

			if (point3d != prevWords3d.end() && utils::isFinite(point3d->second))
			{
				points3d.push_back(point3d->second);
				imagePoints.push_back(iter->second.pt);
			}
		}

		if (points3d.size() >= _minInliers)
		{
			cv::Mat K = cameraModel.getIntrinsics();
			cv::Mat D = cameraModel.getDistortions();
			cv::Mat rvec;
			cv::Mat tvec;

			std::vector<int> inliers;
			cv::solvePnPRansac(points3d, imagePoints, K, D, rvec, tvec, false, _iterations, _reprojError, 0.99, inliers, _flags);

			inliersCountOut = inliers.size();  

			if (inliersCountOut >= _minInliers)
			{
				cv::Mat R;
				cv::Rodrigues(rvec, R);

                auto data = Eigen::Matrix4f();
                data << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), tvec.at<double>(0),
						R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), tvec.at<double>(1),
						R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), tvec.at<double>(2),
                        0, 0, 0, 1;

				return Transform(data.inverse());
			}
		}

		return Transform();
	}
}