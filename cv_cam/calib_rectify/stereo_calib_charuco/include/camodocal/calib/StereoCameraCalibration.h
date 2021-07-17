#ifndef STEREOCAMERACALIBRATION_H
#define STEREOCAMERACALIBRATION_H

#include "CameraCalibration.h"

namespace camodocal {

class StereoCameraCalibration {
 public:
    StereoCameraCalibration() {}
  StereoCameraCalibration(
      Camera::ModelType modelType, const std::string &cameraLeftName,
      const std::string &cameraRightName, const cv::Size &imageSize,
      const cv::Size &boardSize);

  void clear(void);

  void addCalibData(
            const std::vector<cv::Point2f> &img_corners_l,
            const std::vector<cv::Point3f> &obj_corners_l,
            const std::vector<cv::Point2f> &img_corners_r,
            const std::vector<cv::Point3f> &obj_corners_r);

  void addCalibDataAll(
            std::vector<std::vector<int> > ids_l,
            std::vector<std::vector<cv::Point2f> > img_points_l,
            std::vector<std::vector<cv::Point3f> > obj_points_l,
            std::vector<std::vector<int> > ids_r,
            std::vector<std::vector<cv::Point2f> > img_points_r,
            std::vector<std::vector<cv::Point3f> > obj_points_r);

  bool calibrate(void);

  int sampleCount(void) const;
  const std::vector<std::vector<cv::Point2f> > &imagePointsLeft(void) const;
  const std::vector<std::vector<cv::Point2f> > &imagePointsRight(void) const;
  const std::vector<std::vector<cv::Point3f> > &scenePoints(void) const;

  const Eigen::Quaterniond get_r() const { return m_q; }
  const Eigen::Vector3d get_t() const { return m_t; }

  CameraPtr &cameraLeft(void);
  const CameraConstPtr cameraLeft(void) const;

  CameraPtr &cameraRight(void);
  const CameraConstPtr cameraRight(void) const;

  void drawResults(
      std::vector<cv::Mat> &imagesLeft,
      std::vector<cv::Mat> &imagesRight) const;

  void writeParams(const std::string &directory) const;
  void setVerbose(bool verbose);

    std::vector<double> intrinsic_params_l;
    std::vector<double> intrinsic_params_r;

 private:
  CameraCalibration m_calibLeft;
  CameraCalibration m_calibRight;

  Eigen::Quaterniond m_q;
  Eigen::Vector3d m_t;

  bool m_verbose;
  std::vector<double> stereo_error;
};
}

#endif
