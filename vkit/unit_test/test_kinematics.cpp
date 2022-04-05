//
// Created by cg on 9/17/19.
//
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/calib3d/calib3d.hpp>

#include "kinematics/rotation_matrix.h"
#include "kinematics/transform.h"
#include "kinematics/angle_axis.h"
#include "kinematics/convertor.h"
#include "math_utils.hpp"

TEST(kinematics, Quarternion)
{
    // cg::Quarternion q(); // TODO: why
    cg::Quarternion qa;
    cg::Quarternion qb(0.93, 0.2, 0.5, 0.1);
    cg::Quarternion qc = cg::Quarternion::unit_random();

    std::cout << "qa: " << qa << std::endl;
    std::cout << "qb: " << qb << std::endl;
    std::cout << "qc: " << qc << endll;

    // rotation_matrix
    cg::Matrix m_qb = qb.rotation_matrix();
    std::cout << "m_qb_01:\n" << m_qb << std::endl;

    Eigen::Vector4d v4b(qb.x(), qb.y(), qb.z(), qb.w());
    Eigen::Matrix3d mb = msckf::quaternionToRotation(v4b);
    std::cout << "m_qb_02:\n" << mb << endll;

    // normalized
    std::cout << "qb  normalized: " << qb.normalized() << std::endl;
    msckf::quaternionNormalize(v4b);
    std::cout << "v4b normalized: " << v4b.transpose() << endll;

    // q1 * q2
    Eigen::Vector4d v4c(qc.x(), qc.y(), qc.z(), qc.w());

    cg::Quarternion q = qb * qc;
    std::cout << "qb * qc 01: " << q << std::endl;

    Eigen::Vector4d v4 = msckf::quaternionMultiplication(v4b, v4c);
    std::cout << "qb * qc 02: " << v4.transpose() << endll;

    // small_angle_quaternion
    cg::Vector<3> v3_s = qc.vec();
    q = cg::Quarternion::small_angle_quaternion(v3_s);
    std::cout << "small q 01: " << q << std::endl;

    Eigen::Vector3d v3_e;
    v3_e[0] = v3_s[0];
    v3_e[1] = v3_s[1];
    v3_e[2] = v3_s[2];
    v4 = msckf::smallAngleQuaternion(v3_e);
    std::cout << "small q 02: " << v4.transpose() << endll;
}

TEST(kinematics, RotationMatrix)
{
    // rotationToQuaternion
    cg::Quarternion qa = cg::Quarternion::unit_random();
    cg::RotationMatrix m_qa = qa.rotation_matrix();

    Eigen::Matrix3d m3;
    for(int i=0; i<3; ++i)
        for(int j=0; j<3; ++j)
            m3(i,j) = m_qa(i,j);

    std::cout << "m_qa:\n" << m_qa << std::endl;

    std::cout << "q01: " << m_qa.quarternion() << endll;

    std::cout << "q01 hamilton: " << m_qa.quarternion_hamilton() << std::endl;
    Eigen::Quaterniond q4_r(m3);
    std::cout << "q01 hamilton (Eigen): " << q4_r.coeffs().transpose() << endll;

    std::cout << "q02: " << msckf::rotationToQuaternion(m3).transpose() << endll;

    std::cout << "angle_axis(by RotationMatrix): \t" << m_qa.angle_axis() << std::endl;
    std::cout << "angle_axis(by Quarternion): \t"    << m_qa.quarternion().angle_axis() << std::endl;
    std::cout << "angle_axis(by Eigen): \t\t"
              << Eigen::AngleAxisd(m3).angle() << ", " << Eigen::AngleAxisd(m3).axis().transpose() << std::endl;
}

TEST(kinematics, Convertor)
{
    // rodrigues
    cg::FLOAT val[3] = {0.04345, -0.05236, -0.01810};
    cg::Vector3 v3(val);
    cg::RotationMatrix R01 = cg::rodrigues(v3);
    std::cout << "R 01:\n" << R01 << std::endl;
    std::cout << "R 01 I:\n" << cg::rodrigues(cg::Vector3()) << std::endl;

    //    cv::Mat_<float> invec = (cv::Mat_<float>(3, 1) << 0.04345, -0.05236, -0.01810);
    cv::Vec3f invec(0.04345, -0.05236, -0.01810);
    cv::Mat R02;
    cv::Rodrigues(invec, R02);
    std::cout << "R 02:\n" << R02 << std::endl;

    // from_two_vector
    cg::Vector3 v31({1,2,3});
    cg::Vector3 v32({3,2,1});
    cg::RotationMatrix R12 = cg::from_two_vector(v31, v32);
    std::cout << "R12:\n" << R12 << std::endl;

    Eigen::Vector3d ev31(1,2,3);
    Eigen::Vector3d ev32(3,2,1);
    Eigen::Matrix3d eR12 = Eigen::Quaterniond::FromTwoVectors(ev31, ev32).toRotationMatrix();
    std::cout << "eR12:\n" << eR12 << std::endl;
}

TEST(kinematics, EuclideanTransform)
{
    cg::EuclideanTransform et01;
    std::cout << "et01: \n" << et01 << std::endl;

    cg::Quarternion q = cg::Quarternion::unit_random();
    cg::EuclideanTransform et02(q.rotation_matrix(), cg::Vector3({1,2,3}));
    std::cout << "et02: \n" << et02 << std::endl;
}
