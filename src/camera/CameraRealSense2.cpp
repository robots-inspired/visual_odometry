#include "src/camera/include/CameraRealSense2.h"

#include <opencv2/imgproc/types_c.h>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <thread>

namespace my_robot
{
	CameraRealSense2::CameraRealSense2() : Camera(),
		_context(new rs2::context),
		_device(),
		_syncer(new rs2::syncer),
		_useStereoCam(false)
	{
		_cameraWidth = 640;
		_cameraHeight = 480;
		_cameraFps = 30;
		_enableEmitter = false;
	}

	CameraRealSense2::~CameraRealSense2()
	{
		try
		{
			if (_device)
			{
				for (rs2::sensor sensor : _device->query_sensors())
				{
					try
					{
						std::string module_name = sensor.get_info(RS2_CAMERA_INFO_NAME);

						if (module_name == "Stereo Module")
						{
							sensor.stop();
							sensor.close();
						}
					}
					catch(const rs2::error & error)
					{
						std::cerr << "sensor stop error: " << error.what() << "\n";
					}
				}

				_device->hardware_reset();
				
				delete _device;
			}
		}
		catch (const rs2::error& error)
		{
			std::cerr << "device stop error: " << error.what() << "\n";
		}

		try 
		{
			delete _context;
		}
		catch (const rs2::error& error)
		{
			std::cerr << "context stop error: " << error.what() << "\n";
		}

		try 
		{
			delete _syncer;
		}
		catch (const rs2::error& error)
		{
			std::cerr << "syncer stop error: " << error.what() << "\n";
		}
	}

	void CameraRealSense2::imageCallback(rs2::frame frame)
	{
		auto stream = frame.get_profile().stream_type();
		switch (stream)
		{   
			case RS2_STREAM_DEPTH:
			case RS2_STREAM_INFRARED:
				(*_syncer)(frame);
				break;
			default:
				break;
		}
	}

	bool CameraRealSense2::init()
	{
		auto devices = _context->query_devices();
		if (devices.size() == 0)
		{
			std::cerr << "RealSense devices not found.";
			return false;
		}

		bool found=false;
		for (auto device : devices)
		{
			_device = new rs2::device();
			*_device = device;
			found=true;
			break;
		}

		if (!found)
		{
			std::cerr << "RealSense devices not found.";
			return false;
		}

		auto dev_sensors = _device->query_sensors();

		rs2::sensor sensor;

		for(auto&& dev_sensor : dev_sensors)
		{
			std::string module_name = dev_sensor.get_info(RS2_CAMERA_INFO_NAME);

			if (module_name == "Stereo Module")
			{
				sensor = dev_sensor;
				sensor.set_option(rs2_option::RS2_OPTION_EMITTER_ENABLED, _enableEmitter);
				sensor.set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);

				break;
			}
		}

		rs2::stream_profile depthStreamProfile;
		std::vector<rs2::stream_profile> sensorProfile;

		auto profiles = sensor.get_stream_profiles();

		for (auto& profile : profiles)
		{
			auto video_profile = profile.as<rs2::video_stream_profile>();

			if (video_profile.width()  == _cameraWidth && video_profile.height() == _cameraHeight && video_profile.fps() == _cameraFps)
			{
				auto intrinsic = video_profile.get_intrinsics();
				auto streamIndex  = video_profile.format();
				auto format = video_profile.format();

				if ((format == RS2_FORMAT_Y8 && video_profile.stream_index() == 1))
				{
					// first always image frame
					if (sensorProfile.empty())
					{
						sensorProfile.push_back(profile);
					}
					else
					{
						sensorProfile.push_back(sensorProfile.back());
						sensorProfile.front() = profile;
					}
					
					_model = CameraModel(intrinsic.fx, intrinsic.fy, intrinsic.ppx, intrinsic.ppy, cv::Size(intrinsic.width, intrinsic.height));
				}
				else if ((!_useStereoCam && format == RS2_FORMAT_Z16) || (_useStereoCam && format == RS2_FORMAT_Y8 && video_profile.stream_index() == 2))
				{
					_depthFormat = format == RS2_FORMAT_Y8 ? CV_8UC1 : CV_16UC1;

					sensorProfile.push_back(profile);
					depthStreamProfile = profile;
				}
				
				if (sensorProfile.size() == 2)
				{
					break;
				}
			}
		}

		if (!_model.isCalibrated())
		{
			return false;
		}

		std::function<void(rs2::frame)> callback_function = [this](rs2::frame frame){imageCallback(frame);};

		if (sensorProfile.size())
		{
			sensor.open(sensorProfile);
			sensor.start(callback_function);
		}

		std::this_thread::sleep_for(std::chrono::seconds(1));

		return true;
	}

	CameraData CameraRealSense2::getFrame()
	{
		CameraData data;
		
		try
		{
			auto frameset = _syncer->wait_for_frames(5000);
			int iters = 0;

			while (frameset.size() != 2 && iters < 2)
			{
				frameset = _syncer->wait_for_frames(1000);

				++iters;
			}

			if (frameset.size() == 2)
			{
				bool isImageGot = false;
				bool isDepthGot = false;
				rs2::frame image_frame;
				rs2::frame depth_frame;

				for (auto it = frameset.begin(); it != frameset.end(); ++it)
				{
					auto frame = (*it);
					auto stream_type = frame.get_profile().stream_type();

					if (stream_type == RS2_STREAM_INFRARED)
					{
						if (_useStereoCam)
						{
							if (!isDepthGot)
							{
								depth_frame = frame;
								isDepthGot = true;
							}
							else
							{
								image_frame = frame;
								isImageGot = true;
							}
						}
						else
						{
							image_frame = frame;
							isImageGot = true;
						}
					}
					else if (stream_type == RS2_STREAM_DEPTH)
					{
						depth_frame = frame;
						isDepthGot = true;
					}
				}

				if (isImageGot && isDepthGot)
				{
					cv::Mat image = cv::Mat(cv::Size(_cameraWidth, _cameraHeight), CV_8UC1, (void*)image_frame.get_data());
					cv::Mat depth = cv::Mat(cv::Size(_cameraWidth, _cameraHeight), _depthFormat, (void*)depth_frame.get_data());

					data = CameraData(image.clone(), depth.clone(), _model);
				}
			}
		}
		catch(const std::exception& ex)
		{
			std::cerr << "RealSense frame error: " << ex.what() << std::endl;
		}

		return data;
	}
}