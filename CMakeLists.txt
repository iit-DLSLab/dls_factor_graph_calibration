cmake_minimum_required(VERSION 2.8)

project(self_camera_calibration)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3)
message(STATUS "Eigen3 found: ${EIGEN3_INCLUDE_DIR}")
include_directories(${EIGEN3_INCLUDE_DIRS})


find_package(PkgConfig)
pkg_check_modules(ipopt ipopt)
include_directories(${ipopt_INCLUDE_DIRS})

find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)


include_directories(include)

find_package(OpenCV 3.3)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})


find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge tf tf_conversions)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(INCLUDE_DIRS include
               LIBRARIES cam_marker_calib
               CATKIN_DEPENDS roscpp sensor_msgs cv_bridge tf
               DEPENDS ipopt OpenCV)


add_library(cam_marker_calib src/CameraMarkerTransform.cpp)
target_link_libraries(cam_marker_calib ${OpenCV_LIBRARIES})

#GTSAM ONE CALIBRATION
add_executable(gtsam_calib_one_camera src/OneCameraNode.cpp)
target_link_libraries(gtsam_calib_one_camera gtsam ${catkin_LIBRARIES} cam_marker_calib ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

#GTSAM MULTICAMERA CALIBRATION
add_executable(gtsam_calib_multi src/MultiCameraNode.cpp)
target_link_libraries(gtsam_calib_multi gtsam ${catkin_LIBRARIES} cam_marker_calib ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})


