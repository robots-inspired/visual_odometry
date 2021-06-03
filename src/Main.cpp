#include <iostream>
#include <vector>
#include <string>

#include <src/camera/include/CameraFactory.h>
#include <src/odometry/include/OdometryFactory.h>
#include <src/visualization/include/ROSVisualizer.h>

#include <opencv2/highgui.hpp>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "VisualOdometry");

    int odomType = 0;

    for(int i = 1; i < argc; ++i)
    {
        if (i < argc - 1)
        {
            if (strcmp(argv[i], "ODOM") == 0)
            {
                odomType = std::stoi(argv[i + 1]);
            }
        }
    }

    std::cout << "odomType: " << odomType << std::endl; 

    if (!ros::ok() || !ros::master::check())
    {
        std::cerr << "ROS core not found\n";

        return 0;
    }

    auto cam = my_robot::CameraFactory::create(my_robot::CameraFactory::Realsense2);

    if (!cam)
    {
        std::cerr << "Cam not found\n";

        return 0;
    }

    bool inited = cam->init();

    auto frameRotMatrix = Eigen::Matrix4f();
    frameRotMatrix << 0, 0, 1, 0, 
                      -1, 0, 0, 0, 
                      0, -1, 0, 0, 
                      0, 0, 0, 1;

    my_robot::Transform frameRot(frameRotMatrix);

    if (inited && cam->isReadyToUse())
    {
        auto odom = my_robot::OdometryFactory::create((my_robot::OdometryFactory::Type)odomType);

        if (odom)
        {
            auto vis = new my_robot::ROSVisualizer();

            ros::Rate rate(50.0);

            while (ros::ok() && ros::master::check())
            {
                std::cout << "===============================================" << std::endl; 
                auto frame = cam->getFrame();

                if (frame.isValid())
                {
                    odom->process(frame);

                    if (!odom->getPose().isNull())
                    {
                        auto pose = frameRot * odom->getPose();
                        Eigen::Vector3f pos = pose.getTranslation();
                        Eigen::Quaternionf rot = pose.getQuaternion();
                        vis->publishTransform(pos, rot, "robot");
                    }
                    else
                    {
                        cv::imshow("frame", frame.getImage());
                    }

                    cv::waitKey(1);
                }
                else
                {
                    std::cerr << "Frame is not valid\n";
                }

                ros::spinOnce();
                rate.sleep();
            }

            delete odom;
            delete vis;
        }
        else
        {
            std::cerr << "Odom is null\n";
        }
    }

    delete cam;
 
    return 0;
}