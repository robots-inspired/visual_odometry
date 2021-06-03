#include "src/odometry/include/OdometryF2M.h"
#include "src/base/include/Utils.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

namespace my_robot 
{
	OdometryF2M::OdometryF2M() : Odometry(),
		_frameMinInliers(200),
		_maximumMapSize(2000),
		_map(new MapData())
	{
	}

	OdometryF2M::~OdometryF2M()
	{
		std::cout << "Destroy OdometryF2M\n";

		delete _map;
	}

	Transform OdometryF2M::calculateMotion(const CameraData& data)
	{
		Transform output;
		FrameData currentFrame(data, data.getCameraModel());
		int inliers;

		if (!_map->words.size())
		{
			FrameData tempFrame;
			_featureFinder->FindKeypoints(currentFrame, tempFrame);

			auto wordsIter = currentFrame.get2dPoints().begin();
			auto descIter = currentFrame.getDescriptors().begin();
			auto pose = getPose().getMatrix();

			for (std::multimap<int, cv::Point3f>::const_iterator iter = currentFrame.get3dPoints().begin(); iter != currentFrame.get3dPoints().end(); ++iter, ++descIter, ++wordsIter)
			{
				if (utils::isFinite(iter->second))
				{
					MapWordData data;
					data.id = iter->first;
					data.pos3d = utils::transformPoint(iter->second, pose);
					data.descriptor = descIter->second;
					data.keypoint = currentFrame.get2dPoints().find(iter->first)->second;

					_map->words.insert(std::make_pair(data.id, data));
				}
			}

			output = Transform::identity();

			_map->cameraData = currentFrame.cameraData();
		}
		else if (_map->words.size())
		{
			FrameData tmpMap(_map->cameraData, _map->cameraData.getCameraModel());
			tmpMap.set2dPoints(_map->getWordsKeypoints());
			tmpMap.set3dPoints(_map->getWordsPos3d());
			tmpMap.setDescriptors(_map->getWordsDescriptors());
			
			if (!_featureFinder->FindKeypoints(tmpMap, currentFrame))
			{
				return output;
			}

			Transform transform = _motionEstimator->computeMotion(tmpMap, currentFrame, inliers);

			if (transform.isNull())
			{
				return output;
			}

			if (inliers > 0 && inliers < _frameMinInliers)
			{
				auto words = currentFrame.get2dPoints();
				auto descriptors = currentFrame.getDescriptors();

				for (auto word = currentFrame.get3dPoints().begin(); word != currentFrame.get3dPoints().end(); ++word)
				{
					int wordId = word->first;

					if (utils::isFinite(word->second))
					{
						MapWordData mapWordData;
						mapWordData.id = wordId;
						mapWordData.descriptor = descriptors.find(wordId)->second;
						mapWordData.pos3d = utils::transformPoint(word->second, transform.getMatrix());
						mapWordData.keypoint = words.find(wordId)->second;

						if (_map->words.find(wordId) == _map->words.end())
						{
							_map->words.insert(std::make_pair(wordId, mapWordData));
						}
					}
				}

				_map->cameraData = currentFrame.cameraData();

				auto mapWords = _map->words;

				if (mapWords.size() <= _maximumMapSize)
				{
					for (auto iter = mapWords.cbegin(); iter != mapWords.cend();)
					{
						if (mapWords.size() <= _maximumMapSize || iter == mapWords.cend())
						{
							break;
						}

						int wordId = iter->first;

						if (currentFrame.get2dPoints().find(wordId) == currentFrame.get2dPoints().end())
						{
							mapWords.erase(iter++);
						}
						else
						{
							++iter;
						}
					}
				}
				_map->words = mapWords;
			}

			if (!transform.isNull())
			{
				output = getPose().inverse() * transform;
			}
		}

		return output;
	}
}