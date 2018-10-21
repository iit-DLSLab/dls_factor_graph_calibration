#include <CameraMarkerTransform.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <iostream>
#include <pwd.h>
#include <unistd.h>
using namespace cv;

namespace dls {
namespace perception {

using Transform = CameraMarkerTransform::Transform;

CameraMarkerTransform::CameraMarkerTransform()   {

}

CameraMarkerTransform::CameraMarkerTransform(const CameraMatrix &camera_matrix,
                                             const DistortionCoeff &distortion_coeff)
{
    setCameraParameters(camera_matrix, distortion_coeff);
}

void CameraMarkerTransform::setDebug(const bool &debug){
    debug_ = debug;
}

void CameraMarkerTransform::setVerbose(const bool &verbose){
    verbose_ = verbose;
}

void CameraMarkerTransform::setCameraParameters(const CameraMatrix &camera_matrix,
                                                const DistortionCoeff &distortion_coeff){
    camera_matrix_ = camera_matrix;
    distortion_coeff_ = distortion_coeff;
}

bool CameraMarkerTransform::getCameraMarkerTransform(const Image &image,
                                                     Transform& cTm){
    std::vector<int> ids;
    CornerList corner_list;
    std::vector<cv::Point2f> charucoCorners;
    std::vector<int> charucoIds;
    cv::Vec3d rvec, tvec;
    bool print_marker = false;
    bool valid = true;

    auto dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);

    // creating a board with 5 x 5 tiles, 4 cm wide, with
    // 2 cm aruco marker inside. The dictionary is small
    // because the marker are small, so we want them to be
    // visibly different from each other
    auto board = aruco::CharucoBoard::create(5,
                                             5,
                                             0.04,
                                             0.025,
                                             dictionary);

    aruco::detectMarkers(image,dictionary,corner_list,ids);

    if(print_marker){
        cv::Mat board_print;
        // 2362 px are 20 cm at 300 dpi
        board->draw(cv::Size(2362,2362),board_print);
        passwd* pw = getpwuid(getuid());
        std::string home_path(pw->pw_dir);
        std::string full_path = home_path + "/calibration_marker.png";
        std::cout << "Writing marker for print: " << full_path << std::endl;
        cv::imwrite(full_path,board_print);
    }

    if (ids.size() > 0)  {
        if(debug_) {
            std::cout << "Ids found: " << ids.size() << std::endl;
        }
        try{
            cv::aruco::interpolateCornersCharuco(corner_list,
                                                 ids,
                                                 image,
                                                 board,
                                                 charucoCorners,
                                                 charucoIds,
                                                 camera_matrix_,
                                                 distortion_coeff_);

            // we want all the markers to be detected
            if(charucoIds.size() >= 16) {
                if(debug_) {
                    std::cout << "Charuco ids found: " << charucoIds.size() << std::endl;
                }



                for(int i = 0; i < charucoCorners.size(); i++) {
                    // if there are two corners in the same place, there is
                    // something wrong
                    if(i != 0 && charucoCorners[i].x == charucoCorners[i - 1].x
                            && charucoCorners[i].y == charucoCorners[i - 1].y) {
                        valid = false;
                        break;
                    }
                }

                if(valid) {
                    valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners,
                                                                charucoIds,
                                                                board,
                                                                camera_matrix_,
                                                                distortion_coeff_,
                                                                rvec,
                                                                tvec);
                } else {
                    return false;
                }
            } else {
                return false;
            }


        } catch(cv::Exception e){
            std::cout << "Error says: " << e.what() << std::endl;
        }



        if(valid) {
            if(debug_) {
                Image image_color(image.rows, image.cols, CV_8UC3);;
                cvtColor(image, image_color, CV_GRAY2BGR);

                cv::aruco::drawDetectedCornersCharuco(image_color,
                                                      charucoCorners,
                                                      charucoIds,
                                                      cv::Scalar(255, 0, 0));

                cv::aruco::drawAxis(image_color,
                                    camera_matrix_,
                                    distortion_coeff_, rvec, tvec, 0.1);

                cv::imshow("out", image_color);
                waitKey(1);
            }

            Mat rotmatrix;
            Rodrigues(rvec, rotmatrix);
            Eigen::Matrix3d rotation;
            Eigen::Vector3d translation;

            cv2eigen(rotmatrix, rotation);
            cv2eigen(tvec, translation);

            camera_marker_transform_.setIdentity();
            camera_marker_transform_.matrix().block<3, 3>(0, 0) = rotation;
            camera_marker_transform_.matrix().block<3, 1>(0, 3) = translation;

            cTm = camera_marker_transform_;



            if(verbose_) {
                //std::cout << "Ids found: " << ids.size() << std::endl;
                //std::cout << "Translation: " << camera_marker_transform_.translation().transpose() << std::endl;
                //std::cout << "Rot matrix: " << camera_marker_transform_.rotation() << std::endl;
                    std::cout << "Marker to camera captured: ";
                    std::cout << cTm.matrix();

            }
        } else {
            return false;
        }

        return true;
    }
    return false;
}

}
}
