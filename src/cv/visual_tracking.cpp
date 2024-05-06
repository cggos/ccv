//
// Created by cg on 10/30/19.
//

#include "ccv/cv/visual_tracking.h"

#include <Eigen/Core>
#include <Eigen/Dense>

#include "ccv/maths/vector.h"

namespace cg {

void optical_flow_single_level(const cg::YImg8 &img1,
                               const cg::YImg8 &img2,
                               const std::vector<cg::Point2f> &kpt1,
                               std::vector<cg::Point2f> &kpt2,
                               std::vector<unsigned char> &success,
                               int path_size,
                               int max_iters,
                               bool inverse) {
  int half_patch_size = path_size / 2 + 1;
  bool have_initial = !kpt2.empty();

  if (!success.empty()) success.clear();

  for (size_t i = 0; i < kpt1.size(); i++) {
    auto kpt = kpt1[i];

    double dx = 0, dy = 0;  // dx,dy need to be estimated
    if (have_initial) {
      dx = kpt2[i].x - kpt.x;
      dy = kpt2[i].y - kpt.y;
    }

    double cost = 0, lastCost = 0;
    bool succ = 1;  // indicate if this point succeeded

    Eigen::Vector2d Jf;               // forward J
    std::vector<Eigen::Vector2d> vJ;  // inverse
    Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
    Eigen::Vector2d b = Eigen::Vector2d::Zero();
    // pre-compute J and H for Inverse Composition
    if (inverse) {
      Eigen::Vector2d Ji;
      for (int x = -half_patch_size; x < half_patch_size; x++) {
        for (int y = -half_patch_size; y < half_patch_size; y++) {
          float xf = kpt.x + x;
          float yf = kpt.y + y;
          // Inverse Jacobian
          // NOTE this J does not change when dx, dy is updated, so we can store it and only compute error
          Ji[0] = (get_pixel_value(img1, xf + 1, yf) - get_pixel_value(img1, xf - 1, yf)) / 2;
          Ji[1] = (get_pixel_value(img1, xf, yf + 1) - get_pixel_value(img1, xf, yf - 1)) / 2;
          H += Ji * Ji.transpose();
          vJ.push_back(Ji);
        }
      }
    }
    // Gauss-Newton iterations
    for (int iter = 0; iter < max_iters; iter++) {
      cost = 0;
      if (kpt.x + dx <= half_patch_size || kpt.x + dx >= img1.cols() - half_patch_size ||
          kpt.y + dy <= half_patch_size || kpt.y + dy >= img1.rows() - half_patch_size) {
        succ = 0;
        break;
      }
      if (inverse)
        b = Eigen::Vector2d::Zero();
      else {
        H = Eigen::Matrix2d::Zero();
        b = Eigen::Vector2d::Zero();
      }
      int n = 0;
      for (int x = -half_patch_size; x < half_patch_size; x++) {
        for (int y = -half_patch_size; y < half_patch_size; y++) {
          float xf = kpt.x + x;
          float yf = kpt.y + y;
          double error = get_pixel_value(img2, xf + dx, yf + dy) - get_pixel_value(img1, xf, yf);
          cost += error * error;
          if (inverse) {
            b += -vJ[n].transpose() * error;
            n++;
          } else {
            Jf[0] = (get_pixel_value(img2, xf + dx + 1, yf + dy) - get_pixel_value(img2, xf + dx - 1, yf + dy)) / 2;
            Jf[1] = (get_pixel_value(img2, xf + dx, yf + dy + 1) - get_pixel_value(img2, xf + dx, yf + dy - 1)) / 2;
            H += Jf * Jf.transpose();
            b += -Jf.transpose() * error;
          }
        }
      }
      Eigen::Vector2d update = H.ldlt().solve(b);

      if (std::isnan(update[0])) {
        // sometimes occurred when we have a black or white patch and H is irreversible
        succ = 0;
        break;
      }

      if (iter > 0 && cost > lastCost) {
        break;
      }

      dx += update[0];
      dy += update[1];
      lastCost = cost;
      succ = 1;
    }

    success.push_back(succ);

    if (have_initial) {
      kpt2[i] = kpt + cg::Point2f(dx, dy);
    } else {
      kpt2.push_back(kpt + cg::Point2f(dx, dy));
    }
  }
}

void optical_flow_multi_level(const std::vector<cg::YImg8> &pyr1,
                              const std::vector<cg::YImg8> &pyr2,
                              const std::vector<cg::Point2f> &kpt1,
                              std::vector<cg::Point2f> &kpt2,
                              std::vector<unsigned char> &success,
                              int path_size,
                              int max_iters,
                              bool inverse) {
  // parameters
  int pyramids = pyr1.size();

  float pyramid_scale = pyr1[1].cols() / (double)pyr1[0].cols();  // <=1

  bool have_initial = !kpt2.empty();

  // coarse-to-fine LK tracking in pyramids
  size_t size_kp1 = kpt1.size();
  std::vector<cg::Point2f> kpt1_top;
  kpt1_top.reserve(size_kp1);
  for (int i = 0; i < size_kp1; i++) {
    cg::Point2f kpt = kpt1[i];
    kpt *= (float)std::pow(pyramid_scale, pyramids - 1);
    kpt1_top.push_back(kpt);
  }
  if (have_initial) {
    for (int i = 0; i < kpt2.size(); i++) {
      cg::Point2f &kpt = kpt2[i];
      kpt *= (float)std::pow(pyramid_scale, pyramids - 1);
    }
  }
  for (int l = pyramids - 1; l >= 0; l--) {
    if (l < pyramids - 1) {
      for (int i = 0; i < kpt2.size(); i++) {
        kpt1_top[i] /= pyramid_scale;
        kpt2[i] /= pyramid_scale;
      }
    }
    optical_flow_single_level(pyr1[l], pyr2[l], kpt1_top, kpt2, success, path_size, max_iters, inverse);
  }
}

}  // namespace cg
