#include "camodocal/calib/CameraCalibration.h"

#include <algorithm>
#include <cstdio>
#include "eigen3/Eigen/Dense"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/gpl/EigenQuaternionParameterization.h"
#include "camodocal/gpl/EigenUtils.h"
#include "camodocal/sparse_graph/Transform.h"

#include "camodocal/camera_models/EquidistantCamera.h"

#include "ceres/ceres.h"
namespace camodocal {

    template <class CameraT>
    class ReprojectionError1 {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ReprojectionError1(
                const Eigen::Vector3d &observed_P, const Eigen::Vector2d &observed_p)
                : m_observed_P(observed_P),
                  m_observed_p(observed_p),
                  m_sqrtPrecisionMat(Eigen::Matrix2d::Identity()) {}

        ReprojectionError1(
                const Eigen::Vector3d &observed_P, const Eigen::Vector2d &observed_p,
                const Eigen::Matrix2d &sqrtPrecisionMat)
                : m_observed_P(observed_P),
                  m_observed_p(observed_p),
                  m_sqrtPrecisionMat(sqrtPrecisionMat) {}

        ReprojectionError1(
                const std::vector<double> &intrinsic_params,
                const Eigen::Vector3d &observed_P, const Eigen::Vector2d &observed_p)
                : m_intrinsic_params(intrinsic_params),
                  m_observed_P(observed_P),
                  m_observed_p(observed_p) {}

        // variables: camera intrinsics and camera extrinsics
        template <typename T>
        bool operator()(
                const T *const intrinsic_params, const T *const q, const T *const t,
                T *residuals) const {
          Eigen::Matrix<T, 3, 1> P = m_observed_P.cast<T>();

          Eigen::Matrix<T, 2, 1> predicted_p;
          CameraT::spaceToPlane(intrinsic_params, q, t, P, predicted_p);

          Eigen::Matrix<T, 2, 1> e = predicted_p - m_observed_p.cast<T>();

          Eigen::Matrix<T, 2, 1> e_weighted = m_sqrtPrecisionMat.cast<T>() * e;

          residuals[0] = e_weighted(0);
          residuals[1] = e_weighted(1);

          return true;
        }

        // variables: camera-odometry transforms and odometry poses
        template <typename T>
        bool operator()(
                const T *const q_cam_odo, const T *const t_cam_odo, const T *const p_odo,
                const T *const att_odo, T *residuals) const {
          T q[4], t[3];
          worldToCameraTransform(q_cam_odo, t_cam_odo, p_odo, att_odo, q, t);

          Eigen::Matrix<T, 3, 1> P = m_observed_P.cast<T>();

          std::vector<T> intrinsic_params(
                  m_intrinsic_params.begin(), m_intrinsic_params.end());

          // project 3D object point to the image plane
          Eigen::Matrix<T, 2, 1> predicted_p;
          CameraT::spaceToPlane(intrinsic_params.data(), q, t, P, predicted_p);

          residuals[0] = predicted_p(0) - T(m_observed_p(0));
          residuals[1] = predicted_p(1) - T(m_observed_p(1));

          return true;
        }

        // private:
        // camera intrinsics
        std::vector<double> m_intrinsic_params;

        // observed 3D point
        Eigen::Vector3d m_observed_P;

        // observed 2D point
        Eigen::Vector2d m_observed_p;

        // square root of precision matrix
        Eigen::Matrix2d m_sqrtPrecisionMat;
    };



CameraCalibration::CameraCalibration()
    : m_boardSize(cv::Size(0, 0)), m_verbose(false) {}

CameraCalibration::CameraCalibration(
    const Camera::ModelType modelType,
    const std::string &cameraName,
    const cv::Size &imageSize,
    const cv::Size &boardSize)
    : m_boardSize(boardSize), m_verbose(false) {

  EquidistantCameraPtr camera(new EquidistantCamera);
  EquidistantCamera::Parameters params = camera->getParameters();
  params.cameraName()  = cameraName;
  params.imageWidth()  = imageSize.width;
  params.imageHeight() = imageSize.height;
  camera->setParameters(params);

  m_camera = camera;
}

void CameraCalibration::clear(void) {
  m_imagePoints.clear();
  m_scenePoints.clear();
}

void CameraCalibration::addCalibData(
    const std::vector<cv::Point2f> &image_corners,
    const std::vector<cv::Point3f> &obj_points) {
  m_imagePoints.push_back(image_corners);
  m_scenePoints.push_back(obj_points);
}

void CameraCalibration::addCalibDataAll(
        std::vector<std::vector<int> > ids,
        std::vector<std::vector<cv::Point2f> > img_points,
        std::vector<std::vector<cv::Point3f> > obj_points) {
  m_arucoIds = ids;
  m_imagePoints = img_points;
  m_scenePoints = obj_points;
};

bool CameraCalibration::calibrate(void) {
  int imageCount = m_imagePoints.size();

  // compute intrinsic camera parameters and extrinsic parameters for each of
  // the views
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;


  bool ret = calibrateHelper(m_camera, rvecs, tvecs);

  m_cameraPoses = cv::Mat(imageCount, 6, CV_64F);
  for (int i = 0; i < imageCount; ++i) {
    m_cameraPoses.at<double>(i, 0) = rvecs.at(i).at<double>(0);
    m_cameraPoses.at<double>(i, 1) = rvecs.at(i).at<double>(1);
    m_cameraPoses.at<double>(i, 2) = rvecs.at(i).at<double>(2);
    m_cameraPoses.at<double>(i, 3) = tvecs.at(i).at<double>(0);
    m_cameraPoses.at<double>(i, 4) = tvecs.at(i).at<double>(1);
    m_cameraPoses.at<double>(i, 5) = tvecs.at(i).at<double>(2);
  }

  // Compute measurement covariance.
  std::vector<std::vector<cv::Point2f> > errVec(m_imagePoints.size());
  Eigen::Vector2d errSum = Eigen::Vector2d::Zero();
  size_t errCount = 0;
  for (size_t i = 0; i < m_imagePoints.size(); ++i) {
    std::vector<cv::Point2f> estImagePoints;
    m_camera->projectPoints(m_scenePoints.at(i), rvecs.at(i), tvecs.at(i), estImagePoints);

    for (size_t j = 0; j < m_imagePoints.at(i).size(); ++j) {
      cv::Point2f pObs = m_imagePoints.at(i).at(j);
      cv::Point2f pEst = estImagePoints.at(j);

      cv::Point2f err = pObs - pEst;

      errVec.at(i).push_back(err);

      errSum += Eigen::Vector2d(err.x, err.y);
    }

    errCount += m_imagePoints.at(i).size();
  }

  Eigen::Vector2d errMean = errSum / static_cast<double>(errCount);

  Eigen::Matrix2d measurementCovariance = Eigen::Matrix2d::Zero();
  for (size_t i = 0; i < errVec.size(); ++i) {
    for (size_t j = 0; j < errVec.at(i).size(); ++j) {
      cv::Point2f err = errVec.at(i).at(j);
      double d0 = err.x - errMean(0);
      double d1 = err.y - errMean(1);

      measurementCovariance(0, 0) += d0 * d0;
      measurementCovariance(0, 1) += d0 * d1;
      measurementCovariance(1, 1) += d1 * d1;
    }
  }
  measurementCovariance /= static_cast<double>(errCount);
  measurementCovariance(1, 0) = measurementCovariance(0, 1);

  m_measurementCovariance = measurementCovariance;

  return ret;
}

int CameraCalibration::sampleCount(void) const {
  return m_imagePoints.size();
}

std::vector<std::vector<cv::Point2f> > &CameraCalibration::imagePoints(void) {
  return m_imagePoints;
}

const std::vector<std::vector<cv::Point2f> > &CameraCalibration::imagePoints(
    void) const {
  return m_imagePoints;
}

    const std::vector<std::vector<int> > &CameraCalibration::arucoIds(void) const {
      return m_arucoIds;
    }

std::vector<std::vector<cv::Point3f> > &CameraCalibration::scenePoints(void) {
  return m_scenePoints;
}

const std::vector<std::vector<cv::Point3f> > &CameraCalibration::scenePoints(
    void) const {
  return m_scenePoints;
}

CameraPtr &CameraCalibration::camera(void) {
  return m_camera;
}

const CameraConstPtr CameraCalibration::camera(void) const {
  return m_camera;
}

Eigen::Matrix2d &CameraCalibration::measurementCovariance(void) {
  return m_measurementCovariance;
}

const Eigen::Matrix2d &CameraCalibration::measurementCovariance(void) const {
  return m_measurementCovariance;
}

cv::Mat &CameraCalibration::cameraPoses(void) {
  return m_cameraPoses;
}

const cv::Mat &CameraCalibration::cameraPoses(void) const {
  return m_cameraPoses;
}

void CameraCalibration::drawResults(std::vector<cv::Mat> &images) const {
  std::vector<cv::Mat> rvecs, tvecs;

  for (size_t i = 0; i < images.size(); ++i) {
    cv::Mat rvec(3, 1, CV_64F);
    rvec.at<double>(0) = m_cameraPoses.at<double>(i, 0);
    rvec.at<double>(1) = m_cameraPoses.at<double>(i, 1);
    rvec.at<double>(2) = m_cameraPoses.at<double>(i, 2);

    cv::Mat tvec(3, 1, CV_64F);
    tvec.at<double>(0) = m_cameraPoses.at<double>(i, 3);
    tvec.at<double>(1) = m_cameraPoses.at<double>(i, 4);
    tvec.at<double>(2) = m_cameraPoses.at<double>(i, 5);

    rvecs.push_back(rvec);
    tvecs.push_back(tvec);
  }

  int drawShiftBits = 4;
  int drawMultiplier = 1 << drawShiftBits;

  cv::Scalar green(0, 255, 0);
  cv::Scalar red(0, 0, 255);

  for (size_t i = 0; i < images.size(); ++i) {
    cv::Mat &image = images.at(i);
    if (image.channels() == 1) {
      cv::cvtColor(image, image, CV_GRAY2RGB);
    }

    std::vector<cv::Point2f> estImagePoints;
    m_camera->projectPoints(
        m_scenePoints.at(i), rvecs.at(i), tvecs.at(i), estImagePoints);

    float errorSum = 0.0f;
    float errorMax = std::numeric_limits<float>::min();

    for (size_t j = 0; j < m_imagePoints.at(i).size(); ++j) {
      cv::Point2f pObs = m_imagePoints.at(i).at(j);
      cv::Point2f pEst = estImagePoints.at(j);

      cv::circle(
          image, cv::Point(
                     cvRound(pObs.x * drawMultiplier),
                     cvRound(pObs.y * drawMultiplier)),
          5, green, 2, CV_AA, drawShiftBits);

      cv::circle(
          image, cv::Point(
                     cvRound(pEst.x * drawMultiplier),
                     cvRound(pEst.y * drawMultiplier)),
          5, red, 2, CV_AA, drawShiftBits);

      float error = cv::norm(pObs - pEst);

      errorSum += error;
      if (error > errorMax) {
        errorMax = error;
      }
    }

    std::ostringstream oss;
    oss << "Reprojection error: avg = " << errorSum / m_imagePoints.at(i).size()
        << "   max = " << errorMax;

    cv::putText(
        image, oss.str(), cv::Point(10, image.rows - 10),
        cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, CV_AA);
  }
}

void CameraCalibration::writeParams(const std::string &filename) const {
  m_camera->writeParametersToYamlFile(filename);
}


void CameraCalibration::setVerbose(bool verbose) {
  m_verbose = verbose;
}

bool CameraCalibration::calibrateHelper(
    CameraPtr &camera, std::vector<cv::Mat> &rvecs, std::vector<cv::Mat> &tvecs) const {

  rvecs.assign(m_scenePoints.size(), cv::Mat());
  tvecs.assign(m_scenePoints.size(), cv::Mat());

  // STEP 1: Estimate intrinsics
  camera->estimateIntrinsics(m_boardSize, m_arucoIds, m_scenePoints, m_imagePoints);

  // STEP 2: Estimate extrinsics
  for (size_t i = 0; i < m_scenePoints.size(); ++i) {
    camera->estimateExtrinsics(m_scenePoints.at(i), m_imagePoints.at(i), rvecs.at(i), tvecs.at(i));
  }

  if (m_verbose) {
    std::cout << "[" << camera->cameraName() << "] "
              << "# INFO: "
              << "Initial reprojection error: " << std::fixed
              << std::setprecision(3)
              << camera->reprojectionError(m_scenePoints, m_imagePoints, rvecs, tvecs)
              << " pixels" << std::endl;
  }

  // STEP 3: optimization using ceres
  optimize(camera, rvecs, tvecs);

  if (m_verbose) {
    double err = camera->reprojectionError(m_scenePoints, m_imagePoints, rvecs, tvecs);
    std::cout << "[" << camera->cameraName() << "] "
              << "# INFO: Final reprojection error: " << err << " pixels" << std::endl;
    std::cout << "[" << camera->cameraName() << "] "
              << "# INFO: " << camera->parametersToString() << std::endl;
  }

  return true;
}

void CameraCalibration::optimize(
    CameraPtr &camera, std::vector<cv::Mat> &rvecs,
    std::vector<cv::Mat> &tvecs) const {
  // Use ceres to do optimization
  ceres::Problem problem;

  std::vector<Transform, Eigen::aligned_allocator<Transform> > transformVec(
      rvecs.size());
  for (size_t i = 0; i < rvecs.size(); ++i) {
    Eigen::Vector3d rvec;
    cv::cv2eigen(rvecs.at(i), rvec);

    transformVec.at(i).rotation() =
        Eigen::AngleAxisd(rvec.norm(), rvec.normalized());
    transformVec.at(i).translation() << tvecs[i].at<double>(0),
        tvecs[i].at<double>(1), tvecs[i].at<double>(2);
  }

  std::vector<double> intrinsicCameraParams;
  m_camera->writeParameters(intrinsicCameraParams);

  // create residuals for each observation
  for (size_t i = 0; i < m_imagePoints.size(); ++i) {
    for (size_t j = 0; j < m_imagePoints.at(i).size(); ++j) {
      const cv::Point3f &spt = m_scenePoints.at(i).at(j);
      const cv::Point2f &ipt = m_imagePoints.at(i).at(j);

      ceres::CostFunction *costFunction = new ceres::AutoDiffCostFunction<
              ReprojectionError1<EquidistantCamera>, 2, 8, 4, 3>(
              new ReprojectionError1<EquidistantCamera>(
                      Eigen::Vector3d(spt.x, spt.y, spt.z), Eigen::Vector2d(ipt.x, ipt.y)));

      ceres::LossFunction *lossFunction = new ceres::CauchyLoss(1.0);
      problem.AddResidualBlock(
          costFunction, lossFunction, intrinsicCameraParams.data(),
          transformVec.at(i).rotationData(),
          transformVec.at(i).translationData());
    }

    ceres::LocalParameterization *quaternionParameterization =
        new EigenQuaternionParameterization;

    problem.SetParameterization(
        transformVec.at(i).rotationData(), quaternionParameterization);
  }

  ceres::Solver::Options options;
  options.max_num_iterations = 1000;
  options.num_threads = 1;

  if (m_verbose) {
    options.minimizer_progress_to_stdout = true;
  }

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  if (m_verbose) {
    std::cout << summary.FullReport() << std::endl;
  }

  camera->readParameters(intrinsicCameraParams);

  for (size_t i = 0; i < rvecs.size(); ++i) {
    Eigen::AngleAxisd aa(transformVec.at(i).rotation());

    Eigen::Vector3d rvec = aa.angle() * aa.axis();
    cv::eigen2cv(rvec, rvecs.at(i));

    cv::Mat &tvec = tvecs.at(i);
    tvec.at<double>(0) = transformVec.at(i).translation()(0);
    tvec.at<double>(1) = transformVec.at(i).translation()(1);
    tvec.at<double>(2) = transformVec.at(i).translation()(2);
  }
}

template <typename T>
void CameraCalibration::readData(std::ifstream &ifs, T &data) const {
  char *buffer = new char[sizeof(T)];

  ifs.read(buffer, sizeof(T));

  data = *(reinterpret_cast<T *>(buffer));

  delete[] buffer;
}

template <typename T>
void CameraCalibration::writeData(std::ofstream &ofs, T data) const {
  char *pData = reinterpret_cast<char *>(&data);

  ofs.write(pData, sizeof(T));
}
}
