#ifndef CGOCV_CAMERA_H
#define CGOCV_CAMERA_H

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

};

#endif //CGOCV_CAMERA_H
