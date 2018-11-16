#ifndef DLS_PERCEPTION_CAMERA_MARKER_TRANSFORM_HPP__
#define DLS_PERCEPTION_CAMERA_MARKER_TRANSFORM_HPP__

#include <opencv2/core.hpp>
#include <Eigen/Dense>

namespace dls {
namespace perception {

/**
 * @brief The CameraMarkerTransform class computes the full 3D Transform between
 * the camera frame and an aruco marker, given a single camera image.
 */
class CameraMarkerTransform {

public:
    typedef typename cv::Matx33d CameraMatrix;
    typedef typename cv::Vec<double,5> DistortionCoeff;
    typedef typename Eigen::Affine3d Transform;
    typedef typename cv::Mat Image; 
    typedef std::vector<cv::Point2f> Corners;
    typedef std::vector<Corners> CornerList;

public:
    CameraMarkerTransform();

    void setCameraParameters(const CameraMatrix& camera_matrix,
                             const DistortionCoeff& distortion_coeff);

    bool getCameraMarkerTransform(const Image& image,
                                  Transform& cTm);

    CameraMarkerTransform(const CameraMatrix& camera_matrix,
                          const DistortionCoeff& distortion_coeff);

    void setDebug(const bool& debug);
    void setVerbose(const bool& verbose);



private:
    CameraMatrix camera_matrix_;
    DistortionCoeff distortion_coeff_;
    bool verbose_ = false;
    bool debug_ = false;

    Transform camera_marker_transform_ = Transform::Identity();
};

}
}

#endif // DLS_PERCEPTION_CAMERA_MARKER_TRANSFORM_HPP__
