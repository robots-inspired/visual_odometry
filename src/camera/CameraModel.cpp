#include "src/camera/include/CameraModel.h"

namespace my_robot 
{
    cv::Point3f CameraModel::projectPoint(const cv::Mat& depth, float x, float y) const
    {
        int u = int(x + 0.5f);
        int v = int(y + 0.5f);
        float d =  float(depth.at<unsigned short>(v, u)) * 0.001f; // realsense depth format

        cv::Point3f pt;

        if (d > 0.0f)
        {
            pt.x = (x - _cx) * d / _fx;
            pt.y = (y - _cy) * d / _fy;
            pt.z = d;
        }
        else
        {
            pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
        }

        return pt;
    }
}