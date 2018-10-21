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
        x1_asus(Symbol('x', 0)),
        x2_multisense(Symbol('x', 1))
    {

    tf::Matrix3x3 temp_tf_matrix;
    temp_tf_matrix.setRPY(0,60*M_PI/180,0);
    tf::Vector3 temp_tf_vector(0.65,0,0.1);
    tf::Transform temp_tf_transform(temp_tf_matrix,temp_tf_vector);
    tf::transformTFToEigen(temp_tf_transform,init_asus);

    tf::Matrix3x3 temp_tf_matrix2;
    temp_tf_matrix2.setRPY(0,30*M_PI/180,0);
    tf::Vector3 temp_tf_vector2(0.75,0,0.1);
    tf::Transform temp_tf_transform2(temp_tf_matrix2,temp_tf_vector2);
    tf::transformTFToEigen(temp_tf_transform2,init_multisense);

    tf::Matrix3x3 temp_tf_matrix3;
    temp_tf_matrix3.setRPY(0,30*M_PI/180,0);
    tf::Vector3 temp_tf_vector3(0.85,0,-0.1);
    tf::Transform temp_tf_transform3(temp_tf_matrix3,temp_tf_vector3);
    tf::transformTFToEigen(temp_tf_transform3,init_pose);



    asus_image_sub.reset(new ImgSubscriber(nh, asus_image_topic, 1));
    multisense_image_sub.reset(new ImgSubscriber(nh, multisense_image_topic, 1));
    sync.reset(new Synchronizer(MyPolicy(10),*asus_image_sub,*multisense_image_sub));
    sync->registerCallback(boost::bind(&MultiCameraCalibNode::camerasCallback,this,_1,_2));
    priorModel = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(1e-6), Vector3::Constant(1e-4)).finished());
    measurementModel = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(1e-6), Vector3::Constant(1e-4)).finished());
    }

    void run(){
        bool tf_ok_asus = false;
        bool tf_ok_multisense = false;
        bool tf_ok_extender=false;

        if(verbose){
            ROS_INFO("Preparing to run the calibration with parameters:");
            ROS_INFO_STREAM("multisense_image_topic: " << multisense_image_topic);
            ROS_INFO_STREAM("multisense_camera_info_topic: " << multisense_camera_info_topic);
            ROS_INFO_STREAM("multisense_camera_frame_name: " << multisense_camera_frame_name);
            ROS_INFO_STREAM("multisense_sensor_frame_name: " << multisense_sensor_frame_name);
            ROS_INFO_STREAM("asus_image_topic: " << asus_image_topic);
            ROS_INFO_STREAM("asus_camera_info_topic: " << asus_camera_info_topic);
            ROS_INFO_STREAM("asus_camera_frame_name: " << asus_camera_frame_name);
            ROS_INFO_STREAM("asus_sensor_frame_name: " << asus_sensor_frame_name);
            ROS_INFO_STREAM("base_frame_name: " << base_frame_name);
            ROS_INFO_STREAM("marker_frame_name" << marker_frame_name);
            ROS_INFO_STREAM("knee_frame_name" << knee_frame_name);
        }

       // cameras_sub_ = nh_.subscribe(multisense_image_topic,10, &MultiCameraCalibNode::multisenseInfoCallback, this);
        multisense_camera_info_sub_ = nh_.subscribe(multisense_camera_info_topic, 10, &MultiCameraCalibNode::multisenseInfoCallback, this);
        asus_camera_info_sub_ = nh_.subscribe(asus_camera_info_topic,10,&MultiCameraCalibNode::asusInfoCallback,this);

        // Don't leave until you get the camera to sensor mount transform!
        while(nh_.ok() && !tf_ok_multisense && !tf_ok_asus && !tf_ok_extender){
                   try{
                       ros::Time now = ros::Time::now();
                       tf_listener_.waitForTransform(asus_sensor_frame_name, asus_camera_frame_name,
                                                     now, ros::Duration(3.0));
                       tf_listener_.lookupTransform(asus_sensor_frame_name, asus_camera_frame_name,
                                                    now, tf_transform_);

                       tf::transformTFToEigen(tf_transform_, sTc_asus);

                       if(verbose){
                           ROS_INFO("Asus Camera to sensor captured:");
                           ROS_INFO_STREAM(sTc_asus.matrix().format(fmt_));
                       }
                       tf_ok_asus = true;

                   }
                   catch (tf::TransformException ex){
                       ROS_ERROR("%s",ex.what());
                       ros::Duration(1.0).sleep();
                   }

            try{
                ros::Time now = ros::Time::now();
                tf_listener_.waitForTransform(multisense_sensor_frame_name, multisense_camera_frame_name,
                                              now, ros::Duration(3.0));
                tf_listener_.lookupTransform(multisense_sensor_frame_name, multisense_camera_frame_name,
                                             now, tf_transform_);

                tf::transformTFToEigen(tf_transform_, sTc_multisense);

                if(verbose){
                    ROS_INFO("Camera to sensor captured:");
                    ROS_INFO_STREAM(sTc_multisense.matrix().format(fmt_));
                }
                tf_ok_multisense = true;

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
                initials.insert(Symbol('x', 0), Pose3(Rot3(init_asus.rotation()),init_asus.translation()));
                initials.insert(Symbol('x', 1), Pose3(Rot3(init_multisense.rotation()),init_multisense.translation()));

                for (int i=2;i<=points_number;i++)
                             {
                                 std::cout << "init: " << i << std::endl;
                                 initials.insert(Symbol('x',i),Pose3(Rot3(init_pose.rotation()),init_pose.translation()));
                             }



                GaussNewtonParams parameters;
                parameters.setVerbosity("ERROR");

                // optimize!
                 GaussNewtonOptimizer optimizer(graph, initials, parameters);
                 Values results = optimizer.optimize();

               // print final values
                results.print("Final Result:\n");

//                // Calculate marginal covariances for all poses
//                Marginals marginals(graph, results);

//                // print marginal covariances
//                std::cout << "x1 covariance:\n" << marginals.marginalCovariance(Symbol('x', 1)) << std::endl;
//                std::cout << "x2 covariance:\n" << marginals.marginalCovariance(Symbol('x', 2)) << std::endl;


    }

    void camerasCallback(const sensor_msgs::ImageConstPtr& image_asus,
                         const sensor_msgs::ImageConstPtr& image_multisense)
    {
        std::cout << "msg received" << std::endl;

        cv_bridge::CvImageConstPtr asus_temp_image = cv_bridge::toCvShare(image_asus, "mono8");
        Mat asus_frame = asus_temp_image->image;

        cv_bridge::CvImageConstPtr multisense_temp_image = cv_bridge::toCvShare(image_multisense, "mono8");
        Mat multisense_frame = multisense_temp_image->image;
            if(debug){
                std::cout << "ASUS image: " << asus_frame.rows
                          << " rows and " << asus_frame.cols << " cols" << std::endl;
                std::cout << "Multisense image: " << multisense_frame.rows
                          << " rows and " << multisense_frame.cols << " cols" << std::endl;
            }
            std::cout << "\nAsus looks for the image ... " <<std::endl;
            if(!asus_camera_marker_transf_.getCameraMarkerTransform(asus_frame,cTm_asus))
            {
                return;
            }
            std::cout << "\nMultisense looks for the image ... " << std::endl;
            if(!multisense_camera_marker_transf_.getCameraMarkerTransform(multisense_frame,cTm_multisense))
            {
                return;
            }

            ros::Time now_asus = image_asus->header.stamp;
            ros::Time now_multisese = image_multisense->header.stamp;
            ros::Time now  = ros::Time(0);
            if(1)
            {
                std::cout << "\nasus_stamp - multisense_stamp = " << now_asus-now_multisese << std::endl;
                std::cout << " asus_stamp " << now_asus << " multisese_stamp " << now_multisese << " now " << now << std::endl;
            }

            try{
                       tf_listener_.waitForTransform(base_frame_name, knee_frame_name,
                                                    now_asus, ros::Duration(3.0));
                       tf_listener_.lookupTransform(base_frame_name, knee_frame_name,
                                                   now_asus, tf_transform_);

                       tf::transformTFToEigen(tf_transform_, bTk_);

                   }
                   catch (tf::TransformException ex)
                   {
                       ROS_ERROR("%s",ex.what());
                       ros::Duration(1.0).sleep();
                       std::cout << " DATA SKIPPED " << std::endl;
                       return;
                   }


    factor_dot_asus = bTk_ * mTk_.inverse() * cTm_asus.inverse()*sTc_asus.inverse();
    factor_dot_multisense = bTk_ * mTk_.inverse() * cTm_multisense.inverse()*sTc_multisense.inverse();
    Transform asusTmulti=sTc_asus*cTm_asus*cTm_multisense.inverse()*sTc_multisense.inverse();


    Transform pose_knee = bTk_ * mTk_.inverse();
    Transform sTm_asus = sTc_asus * cTm_asus;
    Transform sTm_multisense = sTc_multisense * cTm_multisense;


    if (verbose)
           {
           std::cout << " \n<------------------DOT FACTORS --------------->" << std::endl;
           std::cout << " sTc_asus " << std::endl;
           std::cout << sTc_asus.matrix().format(fmt_) << std::endl << std::endl;
           std::cout << " sTc_multisense " << std::endl;
           std::cout << sTc_multisense.matrix().format(fmt_) << std::endl << std::endl;
           std::cout << " cTm_asus " << std::endl;
           std::cout << cTm_asus.matrix().format(fmt_) << std::endl << std::endl;
           std::cout << " cTm_multisense " << std::endl;
           std::cout << cTm_multisense.matrix().format(fmt_) << std::endl << std::endl;
           std::cout << " mTk_ " << std::endl;
           std::cout << mTk_.matrix().format(fmt_) << std::endl << std::endl;
           std::cout << " bTk_ " << std::endl;
           std::cout << bTk_.matrix().format(fmt_) << std::endl << std::endl;
           std::cout << " factor_dot_asus " << std::endl;
           std::cout << factor_dot_asus.matrix().format(fmt_) << std::endl << std::endl;
           std::cout << " factor_dot_multisense " << std::endl;
           std::cout << factor_dot_multisense.matrix().format(fmt_) << std::endl << std::endl;
           std::cout << " relative transform asus - multisense " << std::endl;
           std::cout << asusTmulti.matrix().format(fmt_) << std::endl << std::endl;
           }


    if (count <= points_number)
       {
   Pose3_ foot_pose(Symbol('x', count));


   graph.addExpressionFactor(foot_pose,
                                Pose3(Rot3(pose_knee.rotation()),
                                      pose_knee.translation()),
                                priorModel);


   graph.addExpressionFactor(between(foot_pose,x1_asus),
                         Pose3(Rot3( sTm_asus.inverse().rotation()),
                               sTm_asus.inverse().translation()),
                               measurementModel);


   graph.addExpressionFactor(between(foot_pose,x2_multisense),
                         Pose3(Rot3( sTm_multisense.inverse().rotation()),
                               sTm_multisense.inverse().translation()),
                               measurementModel);


     graph.addExpressionFactor(between(x1_asus,x2_multisense),
                            Pose3(Rot3(asusTmulti.rotation()),asusTmulti.translation()),
                            measurementModel);


    }







//    graph.addExpressionFactor(x1_asus,
//                              Pose3(Rot3(factor_dot_asus.rotation()),factor_dot_asus.translation()),
//                              priorModel);
//    graph.addExpressionFactor(x2_multisense,
//                              Pose3(Rot3(factor_dot_multisense.rotation()),factor_dot_multisense.translation()),
//                              priorModel);

//    graph.addExpressionFactor(between(x1_asus,x2_multisense),
//                          Pose3(Rot3(asusTmulti.rotation()),asusTmulti.translation()),
//                          measurementModel);

         if (count >= points_number)
         {
             std::cout << " Accumulated point exceeded, stop = true " << std::endl;
             stop=true;
         }

         count++;

         if (1)
         {
             std::cout << "Number of factors accumulated " << count << std::endl;
         }

    }




    void multisenseInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info_msg){
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
            std::cout << "-- Receiving Multisense parameters" << std::endl;
            std::cout << "   Camera Matrix: " << std::endl << cameraMatrix << std::endl;
            std::cout << std::endl;
            std::cout << "   Distortion coefficients: " << std::endl << distCoeff;
            std::cout << std::endl;
        }
        multisense_camera_marker_transf_.setCameraParameters(cameraMatrix, distCoeff);

        // we need the parameters only once.
        multisense_camera_info_sub_.shutdown();
    }

    void asusInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info_msg){
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
            std::cout << "-- Receiving Asus parameters" << std::endl;
            std::cout << "   Camera Matrix: " << std::endl << cameraMatrix << std::endl;
            std::cout << std::endl;
            std::cout << "   Distortion coefficients: " << std::endl << distCoeff;
            std::cout << std::endl;
        }
        asus_camera_marker_transf_.setCameraParameters(cameraMatrix, distCoeff);

        // we need the parameters only once.
        asus_camera_info_sub_.shutdown();
    }



private:
    ros::NodeHandle nh_;
    CameraMarkerTransform multisense_camera_marker_transf_;
    CameraMarkerTransform asus_camera_marker_transf_;


    ImgSubscriberPtr asus_image_sub;
    ImgSubscriberPtr multisense_image_sub;

    ros::Subscriber cameras_sub_;
    ros::Subscriber multisense_camera_info_sub_;
    ros::Subscriber asus_camera_info_sub_;

    SynchronizerPtr sync;

    tf::TransformListener tf_listener_;
    tf::StampedTransform tf_transform_;

    Transform cTm_asus = Transform::Identity();
    Transform cTm_multisense = Transform::Identity();
    Transform bTk_ = Transform::Identity(); // from base frame to knee frame
    Transform mTk_ = Transform::Identity(); // from knee frame to marker frame
    Transform sTc_asus = Transform::Identity();
    Transform sTc_multisense = Transform::Identity();

    Transform factor_dot_asus=Transform::Identity();
    Transform factor_dot_multisense=Transform::Identity();

    Transform pose_asus=Transform::Identity();
    Transform pose_multisense=Transform::Identity();

    Transform init_asus=Transform::Identity();
    Transform init_multisense=Transform::Identity();
    Transform init_pose = Transform::Identity();

    Eigen::IOFormat fmt_ = Eigen::IOFormat(Eigen::FullPrecision, 0, "\t ", "\n", "[", "]");

    bool stop = false;

    int count=2;

    ExpressionFactorGraph graph;
    noiseModel::Diagonal::shared_ptr measurementModel;
    noiseModel::Diagonal::shared_ptr priorModel;


    Pose3_ x2_multisense,x1_asus;


public:
    std::string multisense_image_topic = "/multisense/left/image_mono";
    std::string multisense_camera_info_topic = "/multisense/left/image_mono/camera_info";
    std::string asus_image_topic="/asus/rgb/image_raw";
    std::string asus_camera_info_topic = "/asus/rgb/camera_info";

    std::string base_frame_name = "/base_link";
    std::string multisense_camera_frame_name = "/multisense/left_camera_optical_frame";
    std::string multisense_sensor_frame_name = "/head";
    std::string asus_camera_frame_name = "/asus_rgb_optical_frame";
    std::string asus_sensor_frame_name = "/asus_mount_link";
    std::string marker_frame_name = "/ext_calibration_plate_head";
    std::string knee_frame_name = "/lf_lowerleg";

    int points_number = 100;

    bool debug = false;
    bool verbose = false;
};




int main(int argc, char** argv)
{
    ros::init(argc,argv,"multicamera_calibrator_node");
    ros::NodeHandle nh("~");
    MultiCameraCalibNode calib_node(nh);

    std::string multisense_image_topic = "/multisense/left/image_mono";
    std::string multisense_camera_info_topic = "/multisense/left/image_mono/camera_info";
    std::string asus_image_topic="/asus/rgb/image_raw";
    std::string asus_camera_info_topic = "/asus/rgb/camera_info";

    std::string multisense_camera_frame_name = "/multisense/left_camera_optical_frame";
    std::string multisense_sensor_frame_name = "/head";
    std::string asus_camera_frame_name = "/asus_rgb_optical_frame";
    std::string asus_sensor_frame_name = "/asus_mount_link";
    std::string knee_frame_name = "/lf_lowerleg";

    bool verbose = true;
    int points_number=100;

    // getting parameters from outside (e.g., launch file)
    nh.getParam("multisense_image_topic", multisense_image_topic);
    nh.getParam("multisense_camera_info_topic", multisense_camera_info_topic);
    nh.getParam("asus_image_topic", asus_image_topic);
    nh.getParam("asus_camera_info_topic", asus_camera_info_topic);

    nh.getParam("multisense_camera_frame_name", multisense_camera_frame_name);
    nh.getParam("multisense_sensor_frame_name",multisense_sensor_frame_name);
    nh.getParam("asus_camera_frame_name", asus_camera_frame_name);
    nh.getParam("asus_sensor_frame_name",asus_sensor_frame_name);

    nh.getParam("verbose", verbose);
    nh.getParam("points_number",points_number);

    calib_node.multisense_image_topic = multisense_image_topic;
    calib_node.multisense_camera_info_topic = multisense_camera_info_topic;
    calib_node.multisense_camera_frame_name = multisense_camera_frame_name;
    calib_node.multisense_sensor_frame_name = multisense_sensor_frame_name;

    calib_node.asus_image_topic = asus_image_topic;
    calib_node.asus_camera_info_topic = asus_camera_info_topic;
    calib_node.asus_camera_frame_name = asus_camera_frame_name;
    calib_node.asus_sensor_frame_name = asus_sensor_frame_name;

    calib_node.points_number = points_number;

    calib_node.verbose = verbose;

    calib_node.run();
    return 0;
}
