#pragma once

#include <opencv2/opencv.hpp>

namespace my_robot 
{
	class CameraModel
	{
		public:
			CameraModel() { }
			CameraModel(double fx, double fy, double cx, double cy, const cv::Size & size = cv::Size(0,0)) :
				_fx(fx),
				_fy(fy),
				_cx(cx),
				_cy(cy),
				_size(size)
			{
				_intrinsics = cv::Mat::zeros(cv::Size(3, 3), CV_64FC1);
				_intrinsics.at<double>(0,0) = fx;
				_intrinsics.at<double>(1,1) = fy;
				_intrinsics.at<double>(0,2) = cx;
				_intrinsics.at<double>(1,2) = cy;
				_distortions = cv::Mat::zeros(1,5,CV_64FC1);
			}

    		cv::Point3f projectPoint(const cv::Mat & depth, float x, float y) const;

			bool isCalibrated() const
			{
				return _fx > 0.0 && _fy > 0 && _cx > 0 && _cy > 0;				
			}

			const cv::Mat& getDistortions() const { return _distortions; }
			const cv::Mat& getIntrinsics() const { return _intrinsics; }

		private:
			cv::Size _size;
			double _fx;
			double _fy;
			double _cx;
			double _cy;

			cv::Mat _intrinsics;
			cv::Mat _distortions;
	};

}