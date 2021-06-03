#pragma once

#include "src/camera/include/CameraModel.h"
#include "src/camera/include/CameraData.h"

#include <opencv2/core/core.hpp>
#include <map>

namespace my_robot
{
	class FrameData
	{
		public:
			FrameData() { }

			FrameData(const CameraData& cameraData, const CameraModel& cameraModel) : 
				_cameraData(cameraData),
				_cameraModel(cameraModel) 
				{
				}

			void set2dPoints(const std::map<int, cv::KeyPoint> & points2d) { _points2d = points2d; };
			void set3dPoints(const std::map<int, cv::Point3f> & points3d) { _points3d = points3d; }
			void setDescriptors(const std::map<int, cv::Mat> & descriptors) { _descriptors = descriptors; }

			const std::map<int, cv::KeyPoint> & get2dPoints() const {return _points2d;}
			const std::map<int, cv::Point3f> & get3dPoints() const {return _points3d;}
			const std::map<int, cv::Mat> & getDescriptors() const {return _descriptors;}

			CameraData & cameraData() { return _cameraData; }
			CameraModel & cameraModel() { return _cameraModel; }

		private:
			std::map<int, cv::KeyPoint> _points2d;
			std::map<int, cv::Point3f> _points3d;
			std::map<int, cv::Mat> _descriptors;

			CameraData _cameraData;
			CameraModel _cameraModel;
	};
}