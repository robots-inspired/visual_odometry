#pragma once

#include <src/camera/include/CameraModel.h>

namespace my_robot
{
	class CameraData
	{
		public:

			CameraData() { };

			CameraData(const cv::Mat& image, const cv::Mat& depth, const CameraModel& cameraModel) :
                _image(image),
                _depth(depth),
                _cameraModel(cameraModel)
            {
            }

            const cv::Mat& getImage() const { return _image; }
            const cv::Mat& getDepth() const { return _depth; }
            const CameraModel& getCameraModel() const { return _cameraModel; }

			bool isValid() const { return _cameraModel.isCalibrated(); }

		private:
			cv::Mat _image;
			cv::Mat _depth;
			CameraModel _cameraModel;
	};

}