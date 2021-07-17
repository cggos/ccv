#include "camodocal/calib/StereoCameraCalibration.h"

#include <boost/filesystem.hpp>
#include <opencv2/core/eigen.hpp>

#include "camodocal/gpl/EigenUtils.h"
#include "camodocal/gpl/EigenQuaternionParameterization.h"
#include "camodocal/camera_models/EquidistantCamera.h"

#include "ceres/ceres.h"

namespace camodocal {

    // variables: camera intrinsics and camera extrinsics
    template<class CameraT>
    class StereoReprojectionError
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        StereoReprojectionError(const Eigen::Vector3d& observed_P,
                                const Eigen::Vector2d& observed_p_l,
                                const Eigen::Vector2d& observed_p_r)
                : m_observed_P(observed_P)
                , m_observed_p_l(observed_p_l)
                , m_observed_p_r(observed_p_r)
        {

        }

        template <typename T>
        bool operator()(const T* const intrinsic_params_l,
                        const T* const intrinsic_params_r,
                        const T* const q_l,
                        const T* const t_l,
                        const T* const q_l_r,
                        const T* const t_l_r,
                        T* residuals) const
        {
          Eigen::Matrix<T,3,1> P;
          P(0) = T(m_observed_P(0));
          P(1) = T(m_observed_P(1));
          P(2) = T(m_observed_P(2));

          Eigen::Matrix<T,2,1> predicted_p_l;
          CameraT::spaceToPlane(intrinsic_params_l, q_l, t_l, P, predicted_p_l);

          Eigen::Quaternion<T> q_r = Eigen::Quaternion<T>(q_l_r) * Eigen::Quaternion<T>(q_l);

          Eigen::Matrix<T,3,1> t_r;
          t_r(0) = t_l[0];
          t_r(1) = t_l[1];
          t_r(2) = t_l[2];

          t_r = Eigen::Quaternion<T>(q_l_r) * t_r;
          t_r(0) += t_l_r[0];
          t_r(1) += t_l_r[1];
          t_r(2) += t_l_r[2];

          Eigen::Matrix<T,2,1> predicted_p_r;
          CameraT::spaceToPlane(intrinsic_params_r, q_r.coeffs().data(), t_r.data(), P, predicted_p_r);

          residuals[0] = predicted_p_l(0) - T(m_observed_p_l(0));
          residuals[1] = predicted_p_l(1) - T(m_observed_p_l(1));
          residuals[2] = predicted_p_r(0) - T(m_observed_p_r(0));
          residuals[3] = predicted_p_r(1) - T(m_observed_p_r(1));

          return true;
        }

    private:
        // observed 3D point
        Eigen::Vector3d m_observed_P;

        // observed 2D point
        Eigen::Vector2d m_observed_p_l;
        Eigen::Vector2d m_observed_p_r;
    };

StereoCameraCalibration::StereoCameraCalibration(
    Camera::ModelType modelType,
    const std::string &cameraLeftName,
    const std::string &cameraRightName,
    const cv::Size &imageSize,
    const cv::Size &boardSize)
    : m_calibLeft(modelType, cameraLeftName, imageSize, boardSize),
      m_calibRight(modelType, cameraRightName, imageSize, boardSize),
      m_verbose(false) {
       stereo_error.resize(2, 0.0);
    }

void StereoCameraCalibration::clear(void) {
  m_calibLeft.clear();
  m_calibRight.clear();
}

void StereoCameraCalibration::addCalibData(
    const std::vector<cv::Point2f> &img_corners_l,
    const std::vector<cv::Point3f> &obj_corners_l,
    const std::vector<cv::Point2f> &img_corners_r,
    const std::vector<cv::Point3f> &obj_corners_r) {
  m_calibLeft.addCalibData(img_corners_l, obj_corners_l);
  m_calibRight.addCalibData(img_corners_r, obj_corners_r);
}

void StereoCameraCalibration::addCalibDataAll(
        std::vector<std::vector<int> > ids_l,
        std::vector<std::vector<cv::Point2f> > img_points_l,
        std::vector<std::vector<cv::Point3f> > obj_points_l,
        std::vector<std::vector<int> > ids_r,
        std::vector<std::vector<cv::Point2f> > img_points_r,
        std::vector<std::vector<cv::Point3f> > obj_points_r) {
  m_calibLeft.addCalibDataAll(ids_l, img_points_l, obj_points_l);
  m_calibRight.addCalibDataAll(ids_r, img_points_r, obj_points_r);
};

bool StereoCameraCalibration::calibrate(void) {
  // calibrate cameras individually
  if (!m_calibLeft.calibrate()) {
    return false;
  }
  if (!m_calibRight.calibrate()) {
    return false;
  }

  // perform stereo calibration
  int imageCount = imagePointsLeft().size();

  // find best estimate for initial transform from left camera frame to right
  // camera frame
  double minReprojErr = std::numeric_limits<double>::max();
  for (int i = 0; i < imageCount; ++i) {
    Eigen::Vector3d rvec;
    rvec << m_calibLeft.cameraPoses().at<double>(i, 0),
        m_calibLeft.cameraPoses().at<double>(i, 1),
        m_calibLeft.cameraPoses().at<double>(i, 2);

    Eigen::Quaterniond q_l = AngleAxisToQuaternion(rvec);

    Eigen::Vector3d t_l;
    t_l << m_calibLeft.cameraPoses().at<double>(i, 3),
        m_calibLeft.cameraPoses().at<double>(i, 4),
        m_calibLeft.cameraPoses().at<double>(i, 5);

    rvec << m_calibRight.cameraPoses().at<double>(i, 0),
        m_calibRight.cameraPoses().at<double>(i, 1),
        m_calibRight.cameraPoses().at<double>(i, 2);

    Eigen::Quaterniond q_r = AngleAxisToQuaternion(rvec);

    Eigen::Vector3d t_r;
    t_r << m_calibRight.cameraPoses().at<double>(i, 3),
        m_calibRight.cameraPoses().at<double>(i, 4),
        m_calibRight.cameraPoses().at<double>(i, 5);

    Eigen::Quaterniond q_l_r = q_r * q_l.conjugate();
    Eigen::Vector3d t_l_r = -q_l_r.toRotationMatrix() * t_l + t_r;

    std::vector<cv::Mat> rvecs(imageCount);
    std::vector<cv::Mat> tvecs(imageCount);

    for (int j = 0; j < imageCount; ++j) {
      rvec << m_calibLeft.cameraPoses().at<double>(j, 0),
          m_calibLeft.cameraPoses().at<double>(j, 1),
          m_calibLeft.cameraPoses().at<double>(j, 2);

      q_l = AngleAxisToQuaternion(rvec);

      t_l << m_calibLeft.cameraPoses().at<double>(j, 3),
          m_calibLeft.cameraPoses().at<double>(j, 4),
          m_calibLeft.cameraPoses().at<double>(j, 5);

      Eigen::Quaterniond q_r = q_l_r * q_l;
      Eigen::Vector3d t_r = q_l_r.toRotationMatrix() * t_l + t_l_r;

      QuaternionToAngleAxis(q_r.coeffs().data(), rvec);
      cv::eigen2cv(rvec, rvecs.at(j));

      cv::eigen2cv(t_r, tvecs.at(j));
    }

//    double reprojErr = cameraRight()->reprojectionError(scenePoints(), imagePointsRight(), rvecs, tvecs);
    double reprojErr = cameraRight()->reprojectionError(
            m_calibRight.scenePoints(), imagePointsRight(), rvecs, tvecs);

    if (reprojErr < minReprojErr) {
      minReprojErr = reprojErr;
      m_q = q_l_r;
      m_t = t_l_r;
    }
  }

  std::vector<cv::Mat> rvecsL(imageCount);
  std::vector<cv::Mat> tvecsL(imageCount);
  std::vector<cv::Mat> rvecsR(imageCount);
  std::vector<cv::Mat> tvecsR(imageCount);

  double *extrinsicCameraLParams[scenePoints().size()];
  for (int i = 0; i < imageCount; ++i) {
    extrinsicCameraLParams[i] = new double[7];

    Eigen::Vector3d rvecL(
        m_calibLeft.cameraPoses().at<double>(i, 0),
        m_calibLeft.cameraPoses().at<double>(i, 1),
        m_calibLeft.cameraPoses().at<double>(i, 2));

    AngleAxisToQuaternion(rvecL, extrinsicCameraLParams[i]);

    extrinsicCameraLParams[i][4] = m_calibLeft.cameraPoses().at<double>(i, 3);
    extrinsicCameraLParams[i][5] = m_calibLeft.cameraPoses().at<double>(i, 4);
    extrinsicCameraLParams[i][6] = m_calibLeft.cameraPoses().at<double>(i, 5);

    cv::eigen2cv(rvecL, rvecsL.at(i));

    Eigen::Vector3d tvecL;
    tvecL << m_calibLeft.cameraPoses().at<double>(i, 3),
        m_calibLeft.cameraPoses().at<double>(i, 4),
        m_calibLeft.cameraPoses().at<double>(i, 5);

    cv::eigen2cv(tvecL, tvecsL.at(i));

    Eigen::Quaterniond q_r = m_q * AngleAxisToQuaternion(rvecL);
    Eigen::Vector3d t_r = m_q.toRotationMatrix() * tvecL + m_t;

    Eigen::Vector3d rvecR;
    QuaternionToAngleAxis(q_r.coeffs().data(), rvecR);

    cv::eigen2cv(rvecR, rvecsR.at(i));
    cv::eigen2cv(t_r, tvecsR.at(i));
  }

  if (m_verbose) {
    double roll, pitch, yaw;
    mat2RPY(m_q.toRotationMatrix(), roll, pitch, yaw);

    std::cout << "[stereo]"
              << "# INFO: Initial extrinsics: " << std::endl
              << "r: " << roll << "  p: " << pitch << "  yaw: " << yaw
              << std::endl
              << "x: " << m_t(0) << "  y: " << m_t(1) << "  z: " << m_t(2)
              << std::endl;

    double error = cameraLeft()->reprojectionError(scenePoints(), imagePointsLeft(), rvecsL, tvecsL);
    std::cout << "[" << cameraLeft()->cameraName() << "] "
              << "# INFO: Initial reprojection error: " << error << " pixels"
              << std::endl;

    error = cameraRight()->reprojectionError(m_calibRight.scenePoints(), imagePointsRight(), rvecsR, tvecsR);
    std::cout << "[" << cameraRight()->cameraName() << "] "
              << "# INFO: Initial reprojection error: " << error << " pixels"
              << std::endl;
  }

  std::vector<double> intrinsicCameraLParams;
  cameraLeft()->writeParameters(intrinsicCameraLParams);

  std::vector<double> intrinsicCameraRParams;
  cameraRight()->writeParameters(intrinsicCameraRParams);

  ceres::Problem problem;

  const std::vector<std::vector<int> > &aruco_ids_l = m_calibLeft.arucoIds();
  const std::vector<std::vector<int> > &aruco_ids_r = m_calibRight.arucoIds();

  for (int i = 0; i < imageCount; ++i) {

    std::vector<int> ids_l = aruco_ids_l[i];
    std::vector<int> ids_r = aruco_ids_r[i];

    for (size_t j = 0; j < scenePoints().at(i).size(); ++j) {

      int n_r = -1;
      for(int n=0; n<ids_r.size(); ++n) {
        if(ids_l[j] == ids_r[n]) {
          n_r = n;
          break;
        }
      }
      if(n_r == -1)
        continue;

      const cv::Point3f &spt  = scenePoints().at(i).at(j);
      const cv::Point2f &iptL = imagePointsLeft().at(i).at(j);
      const cv::Point2f &iptR = imagePointsRight().at(i).at(n_r);

      ceres::CostFunction *costFunction =
              new ceres::AutoDiffCostFunction<StereoReprojectionError<EquidistantCamera>, 4, 8, 8, 4, 3, 4, 3>(
              new StereoReprojectionError<EquidistantCamera>(
                      Eigen::Vector3d(spt.x, spt.y, spt.z),
                      Eigen::Vector2d(iptL.x, iptL.y),
                      Eigen::Vector2d(iptR.x, iptR.y)));

      ceres::LossFunction *lossFunction = new ceres::CauchyLoss(1.0);
      problem.AddResidualBlock(
          costFunction,
          lossFunction,
          intrinsicCameraLParams.data(),
          intrinsicCameraRParams.data(),
          extrinsicCameraLParams[i],
          extrinsicCameraLParams[i] + 4,
          m_q.coeffs().data(),
          m_t.data());
    }
  }

  for (int i = 0; i < imageCount; ++i) {
    ceres::LocalParameterization *quaternionParameterization = new EigenQuaternionParameterization;
    problem.SetParameterization(extrinsicCameraLParams[i], quaternionParameterization);
  }

  ceres::LocalParameterization *quaternionParameterization = new EigenQuaternionParameterization;

  problem.SetParameterization(m_q.coeffs().data(), quaternionParameterization);

  ceres::Solver::Options options;
  options.max_num_iterations = 1000;
  options.num_threads = 8;

  if (m_verbose) {
    options.minimizer_progress_to_stdout = true;
  }

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  if (m_verbose) {
    std::cout << summary.FullReport() << "\n";
  }

  cameraLeft()->readParameters(intrinsicCameraLParams);
  cameraRight()->readParameters(intrinsicCameraRParams);

  for (int i = 0; i < imageCount; ++i) {
    Eigen::Vector3d rvecL;
    QuaternionToAngleAxis(extrinsicCameraLParams[i], rvecL);

    m_calibLeft.cameraPoses().at<double>(i, 0) = rvecL(0);
    m_calibLeft.cameraPoses().at<double>(i, 1) = rvecL(1);
    m_calibLeft.cameraPoses().at<double>(i, 2) = rvecL(2);
    m_calibLeft.cameraPoses().at<double>(i, 3) = extrinsicCameraLParams[i][4];
    m_calibLeft.cameraPoses().at<double>(i, 4) = extrinsicCameraLParams[i][5];
    m_calibLeft.cameraPoses().at<double>(i, 5) = extrinsicCameraLParams[i][6];

    cv::eigen2cv(rvecL, rvecsL.at(i));

    Eigen::Vector3d tvecL;
    tvecL << extrinsicCameraLParams[i][4], extrinsicCameraLParams[i][5], extrinsicCameraLParams[i][6];

    cv::eigen2cv(tvecL, tvecsL.at(i));

    Eigen::Quaterniond q_r = m_q * AngleAxisToQuaternion(rvecL);
    Eigen::Vector3d t_r = m_q.toRotationMatrix() * tvecL + m_t;

    Eigen::Vector3d rvecR;
    QuaternionToAngleAxis(q_r.coeffs().data(), rvecR);

    m_calibRight.cameraPoses().at<double>(i, 0) = rvecR(0);
    m_calibRight.cameraPoses().at<double>(i, 1) = rvecR(1);
    m_calibRight.cameraPoses().at<double>(i, 2) = rvecR(2);
    m_calibRight.cameraPoses().at<double>(i, 3) = t_r(0);
    m_calibRight.cameraPoses().at<double>(i, 4) = t_r(1);
    m_calibRight.cameraPoses().at<double>(i, 5) = t_r(2);

    cv::eigen2cv(rvecR, rvecsR.at(i));
    cv::eigen2cv(t_r, tvecsR.at(i));
  }

  if (m_verbose) {
    double roll, pitch, yaw;
    mat2RPY(m_q.toRotationMatrix(), roll, pitch, yaw);

    std::cout << "[stereo]"
              << "# INFO: Final extrinsics: " << std::endl
              << "r: " << roll << "  p: " << pitch << "  yaw: " << yaw
              << std::endl
              << "x: " << m_t(0) << "  y: " << m_t(1) << "  z: " << m_t(2)
              << std::endl;

    stereo_error[0] = cameraLeft()->reprojectionError(scenePoints(), imagePointsLeft(), rvecsL, tvecsL);
    std::cout << "[" << cameraLeft()->cameraName() << "] "
              << "# INFO: Final reprojection error: " << stereo_error[0] << " pixels"
              << std::endl;
    std::cout << "[" << cameraLeft()->cameraName() << "] "
              << "# INFO: " << cameraLeft()->parametersToString() << std::endl;

    stereo_error[1] = cameraRight()->reprojectionError(m_calibRight.scenePoints(), imagePointsRight(), rvecsR, tvecsR);
    std::cout << "[" << cameraRight()->cameraName() << "] "
              << "# INFO: Final reprojection error: " << stereo_error[1] << " pixels"
              << std::endl;
    std::cout << "[" << cameraRight()->cameraName() << "] "
              << "# INFO: " << cameraRight()->parametersToString() << std::endl;
  }

  cameraLeft()->writeParameters(intrinsic_params_l);
  cameraRight()->writeParameters(intrinsic_params_r);

  return true;
}

int StereoCameraCalibration::sampleCount(void) const {
  return m_calibLeft.sampleCount();
}

const std::vector<std::vector<cv::Point2f> >
    &StereoCameraCalibration::imagePointsLeft(void) const {
  return m_calibLeft.imagePoints();
}

const std::vector<std::vector<cv::Point2f> >
    &StereoCameraCalibration::imagePointsRight(void) const {
  return m_calibRight.imagePoints();
}

const std::vector<std::vector<cv::Point3f> >
    &StereoCameraCalibration::scenePoints(void) const {
  return m_calibLeft.scenePoints();
}

CameraPtr &StereoCameraCalibration::cameraLeft(void) {
  return m_calibLeft.camera();
}

const CameraConstPtr StereoCameraCalibration::cameraLeft(void) const {
  return m_calibLeft.camera();
}

CameraPtr &StereoCameraCalibration::cameraRight(void) {
  return m_calibRight.camera();
}

const CameraConstPtr StereoCameraCalibration::cameraRight(void) const {
  return m_calibRight.camera();
}

void StereoCameraCalibration::drawResults(
    std::vector<cv::Mat> &imagesLeft, std::vector<cv::Mat> &imagesRight) const {
  m_calibLeft.drawResults(imagesLeft);
  m_calibRight.drawResults(imagesRight);
}

void StereoCameraCalibration::writeParams(const std::string &directory) const {
  if (!boost::filesystem::exists(directory)) {
    boost::filesystem::create_directory(directory);
  }

  cameraLeft()->writeParametersToYamlFile(directory + "/camera_left.yaml");
  cameraRight()->writeParametersToYamlFile(directory + "/camera_right.yaml");

  cv::FileStorage fs(directory + "/extrinsics.yaml", cv::FileStorage::WRITE);

  auto r = m_q.toRotationMatrix();
  std::vector<double> rotations;
  for (Eigen::Index i = 0; i < 3; i++) {
    for (Eigen::Index j = 0; j < 3; j++) {
      rotations.push_back(r.coeffRef(i, j));
    }
  }

  fs << "transform";
  fs << "{"
     << "q_x" << m_q.x()
     << "q_y" << m_q.y()
     << "q_z" << m_q.z()
     << "q_w" << m_q.w()
     << "t_x" << m_t(0)
     << "t_y" << m_t(1)
     << "t_z" << m_t(2)
     << "r" << rotations
     << "}";

  fs.release();
  cv::FileStorage error_file(directory + "/stereo_reprojection_error.yaml", cv::FileStorage::WRITE);
  error_file << "left_reprojection_error" << stereo_error[0];
  error_file << "right_reprojection_error" << stereo_error[1];
  error_file.release();
}

void StereoCameraCalibration::setVerbose(bool verbose) {
  m_verbose = verbose;
  m_calibLeft.setVerbose(verbose);
  m_calibRight.setVerbose(verbose);
}
}
