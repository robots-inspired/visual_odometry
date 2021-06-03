#pragma once

#include <src/base/include/Transform.h>

#include <list>
#include <map>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace my_robot 
{
	struct MapWordData
	{
		int id;
		cv::Point3f pos3d;
		cv::KeyPoint keypoint;
		cv::Mat descriptor;
	};

	struct MapData
	{
		int id;
		CameraData cameraData;
		std::map<int, MapWordData> words; 

		void reset()
		{
			cameraData = CameraData();
			words = std::map<int, MapWordData>();
		}

		std::map<int, cv::KeyPoint> getWordsKeypoints()
		{
			std::map<int, cv::KeyPoint> keypoints;

			for (auto word = words.begin(); word != words.end(); ++word)
			{
				keypoints.insert(std::make_pair(word->first, word->second.keypoint));
			}

			return keypoints;
		}

		std::map<int, cv::Point3f> getWordsPos3d()
		{
			std::map<int, cv::Point3f> words3;

			for (auto word = words.begin(); word != words.end(); ++word)
			{
				words3.insert(std::make_pair(word->first, word->second.pos3d));
			}

			return words3;
		}

		std::map<int, cv::Mat> getWordsDescriptors() const
		{
			std::map<int, cv::Mat> descriptors;

			for (auto word = words.begin(); word != words.end(); ++word)
			{
				descriptors.insert(std::make_pair(word->first, word->second.descriptor));
			}

			return descriptors;
		}
	};
}