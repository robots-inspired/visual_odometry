cmake_minimum_required(VERSION 3.0.2)

project(robot_odometry_cpp)

SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

find_package(OpenCV 4.0 REQUIRED PATHS /home/rustam/programs/opencv/release)
find_package(RealSense2 2.41.0 REQUIRED)
find_package(Eigen3 REQUIRED)

MESSAGE(STATUS "RealSense2_INCLUDE_DIR: ${RealSense2_INCLUDE_DIR}")


SET(INCLUDE_DIRS
	${PROJECT_SOURCE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${RealSense2_INCLUDE_DIR}
	${EIGEN3_INCLUDE_DIR}
)

set(SOURCE_LIB
	src/camera/CameraModel.cpp
	src/camera/CameraRealSense2.cpp

    src/base/FeatureFinder.cpp
    src/base/MotionEstimator.cpp
    src/base/Transform.cpp
    src/base/Utils.cpp

    src/features/FeatureBase.cpp
    src/features/ORB.cpp
    src/features/SIFT.cpp

    src/odometry/Odometry.cpp
    src/odometry/OdometryF2F.cpp
    src/odometry/OdometryF2M.cpp
)

SET(LIBRARIES
	${OpenCV_LIBRARIES} 
	${realsense2_LIBRARIES}
	sqlite3
)

add_library(MyRoboLib STATIC ${SOURCE_LIB})
target_link_libraries(MyRoboLib "${LIBRARIES}")
target_include_directories(MyRoboLib PUBLIC "${INCLUDE_DIRS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  pcl_ros
  image_transport
  cv_bridge
)

SET(APP_INCLUDE_DIRS
	${INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}	
)

SET(CLIENT_FILES
	src/Main.cpp

    src/visualization/ROSVisualizer.cpp
)

add_executable(OdometryTest ${CLIENT_FILES})
target_link_libraries(OdometryTest MyRoboLib ${catkin_LIBRARIES})
target_include_directories(OdometryTest PUBLIC "${APP_INCLUDE_DIRS}")

set_target_properties(OdometryTest PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON)