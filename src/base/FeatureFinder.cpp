#include "src/base/include/FeatureFinder.h"
#include "src/base/include/Utils.h"
#include "src/features/include/FeatureFactory.h"

namespace my_robot
{
	FeatureFinder::FeatureFinder()
	{
		_featureType = 1;		//0 - Sift, 1 - Orb
		_featureMatching = 1; 	// 0 - BruteForce=0, 1 - Flann
		_featureMaxDist = 100;	

		_detector = features::FeatureFactory::create((features::FeatureFactory::Type)_featureType);
	}

	bool FeatureFinder::FindKeypoints(FrameData& firstFrame, FrameData& secondFrame) const
	{
		CameraData& cameraData1 = firstFrame.cameraData();
		CameraData& cameraData2 = secondFrame.cameraData();

		bool generated = false;
		cv::Mat image1 = cameraData1.getImage();
		std::vector<cv::KeyPoint> kpts1 = GetKeypoints(image1, cameraData1.getDepth(), firstFrame.get2dPoints());
		std::vector<cv::Point3f> kpts13D = GetKeypoints3D(kpts1, firstFrame.get3dPoints(), cameraData1);
		cv::Mat descriptorsFirst = GetDescriptors(image1, kpts1, firstFrame.getDescriptors(), generated);

		std::vector<int> wordIds;
		auto points2d1 = firstFrame.get2dPoints();

		if (!(generated || points2d1.empty()))
		{
			for (auto iter = points2d1.begin(); iter != points2d1.end(); ++iter)
			{
				wordIds.push_back(iter->first);
			}
		}
		
		cv::Mat image2 = cameraData2.getImage();
		std::vector<cv::KeyPoint> kpts2 = GetKeypoints(image2, cameraData2.getDepth(), secondFrame.get2dPoints());
		std::vector<cv::Point3f> kpts23D = GetKeypoints3D(kpts2, secondFrame.get3dPoints(), cameraData2);
		cv::Mat descriptorsSecond = GetDescriptors(image2, kpts2, secondFrame.getDescriptors(), generated);


		std::map<int, cv::KeyPoint> matched2dPoints1;
		std::map<int, cv::Point3f> matched3dPoints1;
		std::map<int, cv::Mat> matchedDescriptors1;
		std::map<int, cv::KeyPoint> matched2dPoints2;
		std::map<int, cv::Point3f> matched3dPoints2;
		std::map<int, cv::Mat> matchedDescriptors2;

		bool kpFound = false;

		if (descriptorsFirst.rows > 0 && descriptorsSecond.rows > 0)
		{
			std::vector<int> fromIds;

			for (int i = 0; i < descriptorsFirst.rows; ++i)
			{
				int id = wordIds.empty() ? (i + 1) : wordIds[i];
				fromIds.push_back(id);
			}

			std::vector<cv::DMatch> matches;

			if (_featureMatching == 1)
			{
				cv::Ptr<cv::DescriptorMatcher> matcher;

				if (descriptorsFirst.type() == CV_8U)
				{
					matcher = new cv::FlannBasedMatcher(new cv::flann::LshIndexParams(6, 12, 2));
				}
				else
				{
					matcher = new cv::FlannBasedMatcher(new cv::flann::KDTreeIndexParams(4));
				}

				matcher->add(descriptorsFirst);
				matcher->train();

				std::vector< std::vector<cv::DMatch>> matchesKnn;
				matcher->knnMatch(descriptorsSecond, matchesKnn, 2);

				const float ratio_thresh = 0.6f;
				for (size_t i = 0; i < matchesKnn.size(); i++)
				{
					if (matchesKnn[i].size() == 2)
					{
						if (matchesKnn[i][0].distance < ratio_thresh * matchesKnn[i][1].distance)
						{
							matches.push_back(matchesKnn[i][0]);
						}
					}
				}
			}
			else 
			{
				cv::BFMatcher matcher(descriptorsFirst.type() == CV_8U ? cv::NORM_HAMMING : cv::NORM_L2SQR, true);
				matcher.match(descriptorsSecond, descriptorsFirst, matches);
			}

			// разделить совпадения на пары
			std::vector<int> toIdsVec(descriptorsSecond.rows, 0);
			kpFound = matches.size() > 0;

			std::cout << "matches.size(): " << matches.size() << std::endl;

			for (int i = 0; i < matches.size(); ++i)
			{
				toIdsVec[matches[i].queryIdx] = fromIds[matches[i].trainIdx];
			}

			std::list<int> toIds;
			for (int i = 0; i < toIdsVec.size(); ++i)
			{
				int toId = toIdsVec[i] != 0 ? toIdsVec[i] : (fromIds.back() + (i + 1));
				toIds.push_back(toId);
			}

			int i = 0;
			for (auto iter = fromIds.begin(); iter != fromIds.end(); ++iter)
			{
				matched2dPoints1.insert(std::make_pair(*iter, kpts1[i]));
				matched3dPoints1.insert(std::make_pair(*iter, kpts13D[i]));
				matchedDescriptors1.insert(std::make_pair(*iter, descriptorsFirst.row(i)));
				i++;
			}

			// добавить только точки с единственным совпадением
			std::multiset<int> toIdsSet(toIds.begin(), toIds.end());
			i = 0;
			for(auto iter = toIds.begin(); iter != toIds.end(); ++iter)
			{
				if (toIdsSet.count(*iter) == 1)
				{
					matched2dPoints2.insert(std::make_pair(*iter, kpts2[i]));
					matched3dPoints2.insert(std::make_pair(*iter, kpts23D[i]));
					matchedDescriptors2.insert(std::make_pair(*iter, descriptorsSecond.row(i)));
				}
				++i;
			}
		}
		else if (descriptorsFirst.rows)
		{
			for (int i = 0; i < descriptorsFirst.rows; ++i)
			{
				matched2dPoints1.insert(std::make_pair(i, kpts1[i]));
				matchedDescriptors1.insert(std::make_pair(i, descriptorsFirst.row(i)));
				matched3dPoints1.insert(std::make_pair(i, kpts13D[i]));
			}
		}

		firstFrame.set2dPoints(matched2dPoints1);
		firstFrame.set3dPoints(matched3dPoints1);
		firstFrame.setDescriptors(matchedDescriptors1);
		
		secondFrame.set2dPoints(matched2dPoints2);
		secondFrame.set3dPoints(matched3dPoints2);
		secondFrame.setDescriptors(matchedDescriptors2);

		return kpFound;
	}

	template<class T>
	inline std::vector<T> getMapValues(const std::map<int, T>& kpts)
	{
		std::vector<T> result;

		for (auto iter = kpts.begin(); iter != kpts.end(); ++iter)
		{
			result.push_back(iter->second);
		}

		return result;
	}

	std::vector<cv::KeyPoint> FeatureFinder::GetKeypoints(const cv::Mat& image, const cv::Mat& depth, const std::map<int, cv::KeyPoint>& frameWords) const
	{
		if (frameWords.empty())
		{
			cv::Mat mask;
			return _detector->detect(image, mask);
		}

		return getMapValues<cv::KeyPoint>(frameWords);
	}

	cv::Mat FeatureFinder::GetDescriptors(const cv::Mat& image, std::vector<cv::KeyPoint>& kpts, const std::map<int, cv::Mat>& descriptors, bool& generated) const
	{
		int descriptorsSize = descriptors.size();

		if (descriptorsSize && (descriptorsSize == kpts.size() || kpts.empty()))
		{
			cv::Mat result = cv::Mat(descriptorsSize, descriptors.begin()->second.cols, descriptors.begin()->second.type());

			int i = 0;
			for (std::map<int, cv::Mat>::const_iterator iter = descriptors.begin(); iter != descriptors.end(); ++iter, ++i)
			{
				iter->second.copyTo(result.row(i));
			}

			return result;
		}

		if (!image.empty())
		{
			generated = true;

			return _detector->compute(image, kpts);

		}

		return cv::Mat();
	}

	std::vector<cv::Point3f> FeatureFinder::GetKeypoints3D(const std::vector<cv::KeyPoint>& kpts, const std::map<int, cv::Point3f>& signatureWords3, const CameraData& cameraData) const
	{
		if (kpts.size() == signatureWords3.size())
		{
			return getMapValues(signatureWords3);
		}

		return utils::projectTo3d(cameraData, kpts);
	}
}