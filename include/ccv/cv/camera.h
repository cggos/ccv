#ifndef CCV_CV_CAMERA_H
#define CCV_CV_CAMERA_H

namespace cg {

struct CameraModel {
  float cx;
  float cy;
  float fx;
  float fy;
};

struct DistortModel {
  float k1;
  float k2;
  float p1;
  float p2;
};

struct StereoCameraModel {
  float baseline;
  CameraModel left;
  CameraModel right;
};

};  // namespace cg

#endif  // CCV_CV_CAMERA_H
