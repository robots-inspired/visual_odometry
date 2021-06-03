#pragma once

#include "src/camera/include/Camera.h"

#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/rs.hpp>

namespace my_robot
{
	class CameraRealSense2 : public Camera
	{
		public:
			CameraRealSense2();

			virtual ~CameraRealSense2();
			virtual bool init();
			virtual CameraData getFrame();

			virtual bool isReadyToUse()
			{
				return _model.isCalibrated();
			}

		private:
			void imageCallback(rs2::frame frame);

		private:
			rs2::context * _context;
			rs2::device* _device;
			rs2::syncer* _syncer;
			CameraModel _model;
			int _depthFormat;

			bool _enableEmitter;
			bool _useStereoCam;
			int _cameraWidth;
			int _cameraHeight;
			int _cameraFps;
	};
}
