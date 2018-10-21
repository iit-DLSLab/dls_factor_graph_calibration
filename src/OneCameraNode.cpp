#include <iostream>

#include <CameraMarkerTransform.hpp>

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Geometry>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>

#include <vector>

using namespace dls::perception;
using namespace cv;
using namespace cv::aruco;
using namespace gtsam;


typedef typename message_filters::Subscriber<sensor_msgs::Image> ImgSubscriber;
typedef typename std::shared_ptr<ImgSubscriber> ImgSubscriberPtr;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MyPolicy;
typedef typename message_filters::Synchronizer<MyPolicy> Synchronizer;
typedef typename std::shared_ptr<Synchronizer> SynchronizerPtr;


typedef Expression<Pose3> Pose3_;

class MultiCameraCalibNode {
public:
    typedef Eigen::Affine3d Transform;
public:
    MultiCameraCalibNode(ros::NodeHandle& nh) :
        nh_(nh),
        x_sensor(Symbol('x', 0))
    {

    tf::Matrix3x3 temp_tf_matrix;
    temp_tf_matrix.setRPY(0,30*M_PI/180,0);
    tf::Vector3 temp_tf_vector(0.65,0,0.1);
    tf::Transform temp_tf_transform(temp_tf_matrix,temp_tf_vector);
    tf::transformTFToEigen(temp_tf_transform,init_sensor);


    tf::Matrix3x3 temp_tf_matrix2;
    temp_tf_matrix2.setRPY(0,0,0);
    tf::Vector3 temp_tf_vector2(0.9,0,0);
    tf::Transform temp_tf_transform2(temp_tf_matrix2,temp_tf_vector2);
    tf::transformTFToEigen(temp_tf_transform2,init_sensor2);


    priorModel = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(1e-6), Vector3::Constant(1e-4)).finished());
    measurementModel = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(1e-6), Vector3::Constant(1e-4)).finished());
    }

    void run(){
        bool tf_ok_sensor = false;
        bool tf_ok_extender=false;

        if(verbose){
            ROS_INFO("Preparing to run the calibration with parameters:");
            ROS_INFO_STREAM("sensor_image_topic: " << image_topic);
            ROS_INFO_STREAM("sensor_camera_info_topic: " << camera_info_topic);
            ROS_INFO_STREAM("sensor_camera_frame_name: " << camera_frame_name);
            ROS_INFO_STREAM("sensor_sensor_frame_name: " << sensor_frame_name);
            ROS_INFO_STREAM("base_frame_name: " << base_frame_name);
            ROS_INFO_STREAM("marker_frame_name" << marker_frame_name);
            ROS_INFO_STREAM("knee_frame_name" << knee_frame_name);
        }

        sensor_camera_info_sub_ = nh_.subscribe(camera_info_topic,10,&MultiCameraCalibNode::sensorInfoCallback,this);
        camera_sub_ = nh_.subscribe(image_topic,10, &MultiCameraCalibNode::cameraCallback, this);
        // Don't leave until you get the camera to sensor mount transform!
        while(nh_.ok() && !tf_ok_sensor && !tf_ok_extender){
                   try{
                       ros::Time now = ros::Time::now();
                       tf_listener_.waitForTransform(sensor_frame_name, camera_frame_name,
                                                     now, ros::Duration(3.0));
                       tf_listener_.lookupTransform(sensor_frame_name, camera_frame_name,
                                                    now, tf_transform_);

                       tf::transformTFToEigen(tf_transform_, sTc_sensor);

                       if(verbose){
                           ROS_INFO("sensor Camera to sensor captured:");
                           ROS_INFO_STREAM(sTc_sensor.matrix().format(fmt_));
                       }
                       tf_ok_sensor = true;

                   }
                   catch (tf::TransformException ex){
                       ROS_ERROR("%s",ex.what());
                       ros::Duration(1.0).sleep();
                   }


                   try{
                       ros::Time now = ros::Time::now();
                       tf_listener_.waitForTransform(marker_frame_name, knee_frame_name,
                                                     now, ros::Duration(3.0));
                       tf_listener_.lookupTransform(marker_frame_name, knee_frame_name,
                                                    now, tf_transform_);

                       tf::transformTFToEigen(tf_transform_, mTk_);

                       if(verbose){
                           ROS_INFO("Knee to Marker captured:");
                           ROS_INFO_STREAM(mTk_.matrix().format(fmt_));
                       }
                       tf_ok_extender = true;

                   }
                   catch (tf::TransformException ex){
                       ROS_ERROR("%s",ex.what());
                       ros::Duration(1.0).sleep();
                   }
               }


        // don't leave until I say so!
        while(nh_.ok() && !stop){
            ros::spinOnce();
        }



                std::cout << " Time to process data and factor graph " << std::endl;

                graph.print("\nFactor Graph:\n");

                Values initials;
                initials.insert(Symbol('x', 0), Pose3(Rot3(init_sensor.rotation()),init_sensor.translation()));

                for (int i=1;i<=points_number;i++)
                {
                    std::cout << "init: " << i << std::endl;
                    initials.insert(Symbol('x',i),Pose3(Rot3(init_sensor2.rotation()),init_sensor2.translation()));
                }


                GaussNewtonParams parameters;
                parameters.setVerbosity("ERROR");

                // optimize!
                 GaussNewtonOptimizer optimizer(graph, initials, parameters);
                 Values results = optimizer.optimize();

               // print final values
                results.print("Final Result:\n");



    }

    void cameraCallback(const sensor_msgs::ImageConstPtr& image_sensor)
    {
        std::cout << "msg received" << std::endl;

        cv_bridge::CvImageConstPtr sensor_temp_image = cv_bridge::toCvShare(image_sensor, "mono8");
        Mat sensor_frame = sensor_temp_image->image;

            if(debug){
                std::cout << "sensor image: " << sensor_frame.rows
                          << " rows and " << sensor_frame.cols << " cols" << std::endl;
            }
            std::cout << "\nsensor looks for the image ... " <<std::endl;
            if(!sensor_camera_marker_transf_.getCameraMarkerTransform(sensor_frame,cTm_sensor))
            {
                return;
            }

            ros::Time now_sensor = image_sensor->header.stamp;
            ros::Time now  = ros::Time(0);
            if(1)
            {
                std::cout << " sensor_stamp " << now_sensor << " multisese_stamp "  << " now " << now << std::endl;
            }

            try{
                       tf_listener_.waitForTransform(base_frame_name, knee_frame_name,
                                                    now_sensor, ros::Duration(3.0));
                       tf_listener_.lookupTransform(base_frame_name, knee_frame_name,
                                                   now_sensor, tf_transform_);

                       tf::transformTFToEigen(tf_transform_, bTk_);

                   }
                   catch (tf::TransformException ex)
                   {
                       ROS_ERROR("%s",ex.what());
                       ros::Duration(1.0).sleep();
                       std::cout << " DATA SKIPPED " << std::endl;
                       return;
                   }

    factor_dot_sensor = bTk_ * mTk_.inverse();
    sTm = sTc_sensor*cTm_sensor;

    chain = bTk_ * mTk_.inverse() * cTm_sensor.inverse() * sTc_sensor.inverse();
    if (verbose)
           {
           std::cout << " \n<------------------DOT FACTORS --------------->" << std::endl;
           std::cout << " sTc_sensor " << std::endl;
           std::cout << sTc_sensor.matrix().format(fmt_) << std::endl << std::endl;
           std::cout << " cTm_sensor " << std::endl;
           std::cout << cTm_sensor.matrix().format(fmt_) << std::endl << std::endl;
           std::cout << " mTk_ " << std::endl;
           std::cout << mTk_.matrix().format(fmt_) << std::endl << std::endl;
           std::cout << " bTk_ " << std::endl;
           std::cout << bTk_.matrix().format(fmt_) << std::endl << std::endl;
           std::cout << " kTb_ " << std::endl;
           std::cout << bTk_.inverse().matrix().format(fmt_) << std::endl << std::endl;
           std::cout << " factor_dot_sensor " << std::endl;
           std::cout << factor_dot_sensor.matrix().format(fmt_) << std::endl << std::endl;
           std::cout << " sTc_sensor*cTm_sensor " << std::endl;
           std::cout << (sTc_sensor*cTm_sensor).matrix().format(fmt_) << std::endl << std::endl;
           std::cout << " sTc_sensor*cTm_sensor " << std::endl;
           std::cout << (sTc_sensor*cTm_sensor).inverse().matrix().format(fmt_) << std::endl << std::endl;

           }

    if (count <= points_number)
    {
Pose3_ foot_pose(Symbol('x', count));


    graph.addExpressionFactor(foot_pose,
                              Pose3(Rot3(factor_dot_sensor.rotation()),
                                    factor_dot_sensor.translation()),
                              priorModel);

    graph.addExpressionFactor(between(foot_pose,x_sensor),
                          Pose3(Rot3( sTm.inverse().rotation()),
                                sTm.inverse().translation()),
                                measurementModel);

}
         if (count == points_number)
         {
             std::cout << " Accumulated point exceeded, stop = true " << std::endl;
             stop=true;
         }



         if (1)
         {
             std::cout << "Number of factors accumulated " << count << std::endl;
         }
         count++;

    }

    void sensorInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info_msg){
        if(verbose){
            std::cout << "Received camera info: " << std::endl;
            std::cout << "-- distortion model: " << camera_info_msg->distortion_model << std::endl;
            std::cout << "-- coefficients: (k1, k2, t1, t2, k3): ";
        }
        for(int i = 0; i < 5; i++){
            std::cout << camera_info_msg->D[i]  << " ";
        }
        std::cout << std::endl;

        std::cout << camera_info_msg->D.size() << std::endl;

        if(!camera_info_msg->distortion_model.compare("plumb_bob") == 0){
            std::cout << "Sorry, only plumb_bob distortion model is supported" << std::endl;
            return;
        }

        Matx33d cameraMatrix;
        for(int i = 0; i < 9; i++) {
            cameraMatrix(i / 3, i % 3) = camera_info_msg->K[i];
        }

        cv::Vec<double,5> distCoeff;
        for(int i = 0; i < 5; i++) {
            distCoeff(i) = (double)camera_info_msg->D[i];
        }

        if(verbose){
            std::cout << "-- Receiving sensor parameters" << std::endl;
            std::cout << "   Camera Matrix: " << std::endl << cameraMatrix << std::endl;
            std::cout << std::endl;
            std::cout << "   Distortion coefficients: " << std::endl << distCoeff;
            std::cout << std::endl;
        }
        sensor_camera_marker_transf_.setCameraParameters(cameraMatrix, distCoeff);

        // we need the parameters only once.
        sensor_camera_info_sub_.shutdown();
    }



private:
    ros::NodeHandle nh_;
    CameraMarkerTransform sensor_camera_marker_transf_;


    ros::Subscriber camera_sub_;
    ros::Subscriber sensor_camera_info_sub_;


    tf::TransformListener tf_listener_;
    tf::StampedTransform tf_transform_;

    Transform cTm_sensor = Transform::Identity();
    Transform bTk_ = Transform::Identity(); // from base frame to knee frame
    Transform mTk_ = Transform::Identity(); // from knee frame to marker frame
    Transform sTc_sensor = Transform::Identity();

    Transform factor_dot_sensor=Transform::Identity();
    Transform sTm = Transform::Identity();
    Transform chain = Transform::Identity();

    Transform pose_sensor=Transform::Identity();

    Transform init_sensor=Transform::Identity();
    Transform init_sensor2=Transform::Identity();

    Eigen::IOFormat fmt_ = Eigen::IOFormat(Eigen::FullPrecision, 0, "\t ", "\n", "[", "]");

    bool stop = false;

    int count=1;

    ExpressionFactorGraph graph;
    noiseModel::Diagonal::shared_ptr measurementModel;
    noiseModel::Diagonal::shared_ptr priorModel;


    Pose3_ x_sensor;
    std::vector<gtsam::Pose3> poses;

public:
    std::string image_topic = "/multisense/left/image_mono";
    std::string camera_info_topic = "/multisense/left/image_mono/camera_info";

    std::string camera_frame_name = "/multisense/left_camera_optical_frame";
    std::string sensor_frame_name = "/head";
    std::string knee_frame_name = "/lf_lowerleg";

    std::string base_frame_name = "/base_link";
    std::string marker_frame_name = "/ext_calibration_plate_head";

    int points_number = 1;

    bool debug = false;
    bool verbose = false;
};




int main(int argc, char** argv)
{
    ros::init(argc,argv,"multicamera_calibrator_node");
    ros::NodeHandle nh("~");
    MultiCameraCalibNode calib_node(nh);

    std::string image_topic = "/multisense/left/image_mono";
    std::string camera_info_topic = "/multisense/left/image_mono/camera_info";

    std::string camera_frame_name = "/multisense/left_camera_optical_frame";
    std::string sensor_frame_name = "/head";
    std::string knee_frame_name = "/lf_lowerleg";

    bool verbose = false;
    int points_number=1;


    // getting parameters from outside (e.g., launch file)
    nh.getParam("image_topic", image_topic);
    nh.getParam("camera_info_topic", camera_info_topic);

    nh.getParam("camera_frame_name", camera_frame_name);
    nh.getParam("sensor_frame_name",sensor_frame_name);

    nh.getParam("verbose", verbose);
    nh.getParam("points_number",points_number);

    calib_node.image_topic = image_topic;
    calib_node.camera_info_topic = camera_info_topic;
    calib_node.camera_frame_name = camera_frame_name;
    calib_node.sensor_frame_name = sensor_frame_name;

    calib_node.points_number = points_number;

    calib_node.verbose = verbose;

    calib_node.run();
    return 0;
}
