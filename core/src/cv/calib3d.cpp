//
// Created by cg on 10/17/19.
//

#include "cvkit/cv/calib3d.h"

#include "cvkit/kinematics/convertor.h"
#include "cvkit/kinematics/transform.h"

namespace cg {

    void project_points(
            const std::vector<cg::Point2f> &pts_in, std::vector<cg::Point2f> &pts_out,
            const cg::Vector3 &rvec, const cg::Vector3 &tvec,
            const cg::Mat3 &camera_matrix,
            const cg::Vector4 &distortion_coeffs) {

        if(pts_in.empty()) {
            std::cerr << "ERROR: pts_in is EMPTY!!!" << std::endl;
            return;
        }

        FLOAT fx = camera_matrix(0, 0);
        FLOAT fy = camera_matrix(1, 1);
        FLOAT cx = camera_matrix(0, 2);
        FLOAT cy = camera_matrix(1, 2);

        FLOAT k1 = distortion_coeffs[0];
        FLOAT k2 = distortion_coeffs[1];
        FLOAT p1 = distortion_coeffs[2];
        FLOAT p2 = distortion_coeffs[3];

        cg::RotationMatrix R = cg::rodrigues(rvec);

        pts_out.resize(pts_in.size());

        for(int i=0; i<pts_in.size(); ++i) {
            const cg::Point2f &pt_in = pts_in[i];
            cg::Point2f &pt_out = pts_out[i];

            cg::Vector3 pt_homogeneous({pt_in.x, pt_in.y, 1.0});

            cg::Vector3 pt_3d = R * pt_homogeneous + tvec;
            FLOAT x = pt_3d[0] / pt_3d[2];
            FLOAT y = pt_3d[1] / pt_3d[2];

            double x2 = x*x, y2 = y*y, xy = x*y, r2 = x2 + y2;
            double x_radial = x * (1 + k1*r2 + k2*r2*r2);
            double y_radial = y * (1 + k1*r2 + k2*r2*r2);
            double x_tangential = 2*p1*xy + p2*(r2 + 2*x2);
            double y_tangential = 2*p2*xy + p1*(r2 + 2*y2);
            double xd = x_radial + x_tangential;
            double yd = y_radial + y_tangential;

            double ud = xd*fx + cx;
            double vd = yd*fy + cy;

            pt_out.x = ud;
            pt_out.y = vd;
        }
    }
}

