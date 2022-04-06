#ifndef CAPTURE_IMGS_COC_STEREO_RECTIFY_H
#define CAPTURE_IMGS_COC_STEREO_RECTIFY_H

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "camodocal/camera_models/Camera.h"
#include "camodocal/camera_models/EquidistantCamera.h"

class CocStereoRectify {

public:
    CocStereoRectify() {}
    CocStereoRectify(std::string str_param_dir) : str_param_dir_(str_param_dir) {
        cam_l_ = camodocal::EquidistantCameraPtr(new camodocal::EquidistantCamera);
        cam_r_ = camodocal::EquidistantCameraPtr(new camodocal::EquidistantCamera);
    }
    ~CocStereoRectify() {}

    inline void get_calib_data_from_file(
            cv::Mat &K1, cv::Mat &D1,
            cv::Mat &K2, cv::Mat &D2,
            cv::Mat &R,  cv::Mat &t,
            cv::Size &img_size) {

        camodocal::EquidistantCamera::Parameters params_l, params_r;
        params_l.readFromYamlFile(str_param_dir_+"camera_left.yaml");
        params_r.readFromYamlFile(str_param_dir_+"camera_right.yaml");

        cam_l_->setParameters(params_l);
        cam_r_->setParameters(params_r);

        img_size = cv::Size(params_l.imageWidth(), params_l.imageHeight());

        K1 = cv::Mat::eye(cv::Size(3, 3), CV_64F);
        K1.at<double>(0,0) = static_cast<double>(params_l.mu());
        K1.at<double>(1,1) = static_cast<double>(params_l.mv());
        K1.at<double>(0,2) = static_cast<double>(params_l.u0());
        K1.at<double>(1,2) = static_cast<double>(params_l.v0());

        D1 = cv::Mat::eye(cv::Size(1, 4), CV_64F);
        D1.at<double>(0,0) = static_cast<double>(params_l.k2());
        D1.at<double>(0,1) = static_cast<double>(params_l.k3());
        D1.at<double>(0,2) = static_cast<double>(params_l.k4());
        D1.at<double>(0,3) = static_cast<double>(params_l.k5());

        K2 = cv::Mat::eye(cv::Size(3, 3), CV_64F);
        K2.at<double>(0,0) = static_cast<double>(params_r.mu());
        K2.at<double>(1,1) = static_cast<double>(params_r.mv());
        K2.at<double>(0,2) = static_cast<double>(params_r.u0());
        K2.at<double>(1,2) = static_cast<double>(params_r.v0());

        D2 = cv::Mat::eye(cv::Size(1, 4), CV_64F);
        D2.at<double>(0,0) = static_cast<double>(params_r.k2());
        D2.at<double>(0,1) = static_cast<double>(params_r.k3());
        D2.at<double>(0,2) = static_cast<double>(params_r.k4());
        D2.at<double>(0,3) = static_cast<double>(params_r.k5());

        cv::FileStorage fs(str_param_dir_+"extrinsics.yaml", cv::FileStorage::READ);
        if(!fs.isOpened())
            std::cerr << "open FAILED!" << std::endl;
        cv::FileNode n_T = fs["transform"];
        double q_x = static_cast<double>(n_T["q_x"]);
        double q_y = static_cast<double>(n_T["q_y"]);
        double q_z = static_cast<double>(n_T["q_z"]);
        double q_w = static_cast<double>(n_T["q_w"]);
        double t_x = static_cast<double>(n_T["t_x"]);
        double t_y = static_cast<double>(n_T["t_y"]);
        double t_z = static_cast<double>(n_T["t_z"]);
        fs.release();

        Eigen::Matrix3d  m3_R = Eigen::Quaterniond(q_w, q_x, q_y, q_z).toRotationMatrix();
        Eigen::Vector3d  v3_t(t_x, t_y, t_z);

        cv::eigen2cv(m3_R, R);
        cv::eigen2cv(v3_t, t);

        static bool is_write = false;
        if(!is_write) {
            std::cout << "K1: \n" << K1 << std::endl;
            std::cout << "D1: \n" << D1 << std::endl;
            std::cout << "K2: \n" << K2 << std::endl;
            std::cout << "D2: \n" << D2 << std::endl;

            std::cout << "m3_R: \n" << R << std::endl;
            std::cout << "v3_t: \n" << t << std::endl;

            is_write = true;
        }
    }

    inline void compute_rectify_map(
            const cv::Mat &K1, const cv::Mat &D1,
            const cv::Mat &K2, const cv::Mat &D2,
            const cv::Mat &R,  const cv::Mat &t,
            cv::Mat &Q,
            const cv::Size &img_size) {

        {
            R1_ = cv::Mat(cv::Size(3, 3), CV_64F);
            R2_ = cv::Mat(cv::Size(3, 3), CV_64F);
            P1_ = cv::Mat(3, 4, CV_64F);
            P2_ = cv::Mat(3, 4, CV_64F);
            Q   = cv::Mat(4, 4, CV_64F);

//            cv::stereoRectify(
//                    K1, D1, K2, D2, img_size1, mat_R, mat_t,
//                    R1, R2, P1, P2, Q);

//            cv::fisheye::stereoRectify(
//                    K1, D1, K2, D2, img_size1, mat_R, mat_t,
//                    R1, R2, P1, P2, Q,
//                    CV_CALIB_ZERO_DISPARITY, img_size1, 0.0, 1.1);

            CvMat c_R = R, c_t = t;
            CvMat c_K1 = K1, c_K2 = K2, c_D1 = D1, c_D2 = D2;
            CvMat c_R1 = R1_, c_R2 = R2_, c_P1 = P1_, c_P2 = P2_;
            CvMat c_Q  = Q;
            stereo_rectify(&c_K1, &c_K2, &c_D1, &c_D2, img_size, &c_R, &c_t, &c_R1, &c_R2, &c_P1, &c_P2, &c_Q,
                           cv::CALIB_ZERO_DISPARITY, 0);
        }

        static bool is_write = false;
        if(!is_write) {
            std::cout << "R1: \n" << R1_ << std::endl;
            std::cout << "R2: \n" << R2_ << std::endl;
            std::cout << "P1: \n" << P1_ << std::endl;
            std::cout << "P2: \n" << P2_ << std::endl;
            is_write = true;
        }

        double fx_l = P1_.at<double>(0,0);
        double fy_l = P1_.at<double>(1,1);
        double cx_l = P1_.at<double>(0,2);
        double cy_l = P1_.at<double>(1,2);

        double fx_r = P2_.at<double>(0,0);
        double fy_r = P2_.at<double>(1,1);
        double cx_r = P2_.at<double>(0,2);
        double cy_r = P2_.at<double>(1,2);

        std::cout << "start initUndistortRectifyMap" << std::endl;

        cam_l_->initUndistortRectifyMap(rectify_map_[0][0], rectify_map_[0][1], fx_l, fy_l, img_size, cx_l, cy_l, R1_);
        cam_r_->initUndistortRectifyMap(rectify_map_[1][0], rectify_map_[1][1], fx_r, fy_r, img_size, cx_r, cy_r, R2_);

        std::cout << "end initUndistortRectifyMap" << std::endl;
    }

    inline void rectify(cv::Mat &img_l, cv::Mat &img_r) {
        static bool is_get_map = false;
        if(!is_get_map) {
            cv::Mat R, t, Q;
            cv::Size img_size;
            get_calib_data_from_file(K1_, D1_, K2_, D2_, R, t, img_size);
            compute_rectify_map(K1_, D1_, K2_, D2_, R, t, Q, img_size);
            {
                cv::FileStorage fs(str_param_dir_+"camera_Q.yaml", cv::FileStorage::WRITE);
                fs << "Q" << Q;
                fs.release();

                cv::Mat Q1;
                FileStorage fStorage(str_param_dir_+"camera_Q.yaml", FileStorage::READ);
                fStorage["Q"] >> Q1;
                std::cout << "\n=================\n" << Q1 << std::endl;
                fStorage.release();
            }
            is_get_map = true;
        }
        cv::remap(img_l, img_l, rectify_map_[0][0], rectify_map_[0][1], cv::INTER_LINEAR);
        cv::remap(img_r, img_r, rectify_map_[1][0], rectify_map_[1][1], cv::INTER_LINEAR);
    }

#if 0
    cv::Mat rectify_rad(const cv::Mat& R)
    {
        cv::Mat r_vec;
        cv::Rodrigues(R, r_vec);
        //pi/180 = x/179 ==> x = 3.1241
        double rad = cv::norm(r_vec);
        if (rad >= 3.1241)
        {
            cv::Mat r_dir;
            cv::normalize(r_vec, r_dir);
            cv::Mat r = r_dir*(3.1415926 - rad);
            cv::Mat r_r;
            cv::Rodrigues(r, r_r);
            return r_r.clone();
        }

        return R.clone();
    }
#endif

    void
    icvGetRectangles(int type, CvSize imgSize, cv::Rect_<float>& inner, cv::Rect_<float>& outer) {
        const int N = 9;
        int x, y, k;
        cv::Ptr<CvMat> _pts(cvCreateMat(1, N * N, CV_32FC2));
        CvPoint2D32f *pts = (CvPoint2D32f *) (_pts->data.ptr);

        for (y = k = 0; y < N; y++)
            for (x = 0; x < N; x++)
                pts[k++] = cvPoint2D32f((float) x * imgSize.width / (N - 1),
                                        (float) y * imgSize.height / (N - 1));

//        cvUndistortPoints(_pts, _pts, cameraMatrix, distCoeffs, R, newCameraMatrix);

        Eigen::Vector2d a;
        Eigen::Vector3d b;
        for (int n = 0; n < N; ++n) {
            a.x() = pts[n].x;
            a.y() = pts[n].y;
            if (0 == type) {
                cam_l_->liftProjective(a, b);
            } else {

                cam_r_->liftProjective(a, b);
            }
            pts[n].x = b.x() / b.z();
            pts[n].y = b.y() / b.z();
        }

        float iX0 = -FLT_MAX, iX1 = FLT_MAX, iY0 = -FLT_MAX, iY1 = FLT_MAX;
        float oX0 = FLT_MAX, oX1 = -FLT_MAX, oY0 = FLT_MAX, oY1 = -FLT_MAX;
        // find the inscribed rectangle.
        // the code will likely not work with extreme rotation matrices (R) (>45%)
        for (y = k = 0; y < N; y++)
            for (x = 0; x < N; x++) {
                CvPoint2D32f p = pts[k++];
                oX0 = MIN(oX0, p.x);
                oX1 = MAX(oX1, p.x);
                oY0 = MIN(oY0, p.y);
                oY1 = MAX(oY1, p.y);

                if (x == 0)
                    iX0 = MAX(iX0, p.x);
                if (x == N - 1)
                    iX1 = MIN(iX1, p.x);
                if (y == 0)
                    iY0 = MAX(iY0, p.y);
                if (y == N - 1)
                    iY1 = MIN(iY1, p.y);
            }
        inner = cv::Rect_<float>(iX0, iY0, iX1 - iX0, iY1 - iY0);
        outer = cv::Rect_<float>(oX0, oY0, oX1 - oX0, oY1 - oY0);
    }

    void stereo_rectify(
            const CvMat* K1, const CvMat* K2, const CvMat* D1, const CvMat* D2,
            CvSize imageSize, const CvMat* matR, const CvMat* matT,
            CvMat* _R1, CvMat* _R2, CvMat* _P1, CvMat* _P2, CvMat* matQ,
            int flags = cv::CALIB_ZERO_DISPARITY, double alpha = -1, CvSize newImgSize = cv::Size()) {

        double _om[3], _t[3] = {0}, _uu[3]={0,0,0}, _r_r[3][3], _pp[3][4];
        double _ww[3], _wr[3][3], _z[3] = {0,0,0}, _ri[3][3], _w3[3];
        cv::Rect_<float> inner1, inner2, outer1, outer2;

        CvMat om  = cvMat(3, 1, CV_64F, _om);
        CvMat t   = cvMat(3, 1, CV_64F, _t);
        CvMat uu  = cvMat(3, 1, CV_64F, _uu);
        CvMat r_r = cvMat(3, 3, CV_64F, _r_r);
        CvMat pp  = cvMat(3, 4, CV_64F, _pp);
        CvMat ww  = cvMat(3, 1, CV_64F, _ww); // temps
        CvMat w3  = cvMat(3, 1, CV_64F, _w3); // temps
        CvMat wR  = cvMat(3, 3, CV_64F, _wr);
        CvMat Z   = cvMat(3, 1, CV_64F, _z);
        CvMat Ri  = cvMat(3, 3, CV_64F, _ri);
        double nx = imageSize.width, ny = imageSize.height;
        int i, k;
        double nt, nw;
        if( matR->rows == 3 && matR->cols == 3 )
            cvRodrigues2(matR, &om);          // get vector rotation
        else
            cvConvert(matR, &om); // it's already a rotation vector

        cvConvertScale(&om, &om, -0.5); // get average rotation

        cvRodrigues2(&om, &r_r);        // rotate cameras to same orientation by averaging
        cvMatMul(&r_r, matT, &t);

        int idx = fabs(_t[0]) > fabs(_t[1]) ? 0 : 1;

        // if idx == 0
        //   e1 = T / ||T||
        //   e2 = e1 x [0,0,1]

        // if idx == 1
        //   e2 = T / ||T||
        //   e1 = e2 x [0,0,1]

        // e3 = e1 x e2

        _uu[2] = 1;
        cvCrossProduct(&uu, &t, &ww);
        nt = cvNorm(&t, 0, CV_L2);
        nw = cvNorm(&ww, 0, CV_L2);

        cvConvertScale(&ww, &ww, 1 / nw);

        cvCrossProduct(&t, &ww, &w3);
        nw = cvNorm(&w3, 0, CV_L2);

        cvConvertScale(&w3, &w3, 1 / nw);

        _uu[2] = 0;

        for (i = 0; i < 3; ++i)
        {
            _wr[idx][i] = -_t[i] / nt;
            _wr[idx ^ 1][i] = -_ww[i];
            _wr[2][i] = _w3[i] * (1 - 2 * idx); // if idx == 1 -> opposite direction
        }

        // apply to both views
        cvGEMM(&wR, &r_r, 1, 0, 0, &Ri, CV_GEMM_B_T);
        cvConvert( &Ri, _R1 );
        cvGEMM(&wR, &r_r, 1, 0, 0, &Ri, 0);
        cvConvert( &Ri, _R2 );
        cvMatMul(&Ri, matT, &t);

        // calculate projection/camera matrices
        // these contain the relevant rectified image internal params (fx, fy=fx, cx, cy)
        double fc_new = DBL_MAX;
        CvPoint2D64f cc_new[2] = {{0,0}, {0,0}};
        newImgSize = newImgSize.width * newImgSize.height != 0 ? newImgSize : imageSize;
        const double ratio_x = (double)newImgSize.width / imageSize.width / 2;
        const double ratio_y = (double)newImgSize.height / imageSize.height / 2;
        const double ratio = idx == 1 ? ratio_x : ratio_y;
        fc_new = (cvmGet(K1, idx ^ 1, idx ^ 1) + cvmGet(K2, idx ^ 1, idx ^ 1)) * ratio;

        for( k = 0; k < 2; k++ )
        {
            CvPoint2D32f _pts[4];
            CvPoint3D32f _pts_3[4];
            CvMat pts = cvMat(1, 4, CV_32FC2, _pts);
            CvMat pts_3 = cvMat(1, 4, CV_32FC3, _pts_3);

            for( i = 0; i < 4; i++ )
            {
                int j = (i<2) ? 0 : 1;
                _pts[i].x = (float)((i % 2)*(nx));
                _pts[i].y = (float)(j*(ny));
            }
            Eigen::Vector2d a;
            Eigen::Vector3d b;
            for( int n=0; n<4; ++n) {
                a.x() = _pts[n].x;
                a.y() = _pts[n].y;
                if (0 == k){
                    cam_l_->liftProjective(a, b);
                }
                else{

                    cam_r_->liftProjective(a, b);
                }
                _pts[n].x = b.x()/b.z();
                _pts[n].y = b.y()/b.z();
            }
            cvConvertPointsHomogeneous( &pts, &pts_3 );

            //Change camera matrix to have cc=[0,0] and fc = fc_new
            double _a_tmp[3][3];
            CvMat A_tmp  = cvMat(3, 3, CV_64F, _a_tmp);
            _a_tmp[0][0]=fc_new;
            _a_tmp[1][1]=fc_new;
            _a_tmp[0][2]=0.0;
            _a_tmp[1][2]=0.0;
            cvProjectPoints2( &pts_3, k == 0 ? _R1 : _R2, &Z, &A_tmp, 0, &pts );
            CvScalar avg = cvAvg(&pts);
            cc_new[k].x = (nx)/2 - avg.val[0];
            cc_new[k].y = (ny)/2 - avg.val[1];

            //cc_new[k].x = (nx)/2;
            //cc_new[k].y = (ny)/2;
        }

        if( flags & cv::CALIB_ZERO_DISPARITY )
        {
            cc_new[0].x = cc_new[1].x = (cc_new[0].x + cc_new[1].x)*0.5;
            cc_new[0].y = cc_new[1].y = (cc_new[0].y + cc_new[1].y)*0.5;
        }
        else if( idx == 0 ) // horizontal stereo
            cc_new[0].y = cc_new[1].y = (cc_new[0].y + cc_new[1].y)*0.5;
        else // vertical stereo
            cc_new[0].x = cc_new[1].x = (cc_new[0].x + cc_new[1].x)*0.5;

        cvZero( &pp );
        _pp[0][0] = _pp[1][1] = fc_new;
        _pp[0][2] = cc_new[0].x;
        _pp[1][2] = cc_new[0].y;
        _pp[2][2] = 1;
        cvConvert(&pp, _P1);

        _pp[0][2] = cc_new[1].x;
        _pp[1][2] = cc_new[1].y;
        _pp[idx][3] = _t[idx]*fc_new; // baseline * focal length
        cvConvert(&pp, _P2);


        alpha = MIN(alpha, 1.);

        icvGetRectangles(0, imageSize, inner1, outer1);
        icvGetRectangles(1, imageSize, inner2, outer2);

        {
            newImgSize = newImgSize.width*newImgSize.height != 0 ? newImgSize : imageSize;
            double cx1_0 = cc_new[0].x;
            double cy1_0 = cc_new[0].y;
            double cx2_0 = cc_new[1].x;
            double cy2_0 = cc_new[1].y;
            double cx1 = newImgSize.width*cx1_0/imageSize.width;
            double cy1 = newImgSize.height*cy1_0/imageSize.height;
            double cx2 = newImgSize.width*cx2_0/imageSize.width;
            double cy2 = newImgSize.height*cy2_0/imageSize.height;
            double s = 1.;

            if( alpha >= 0 )
            {
                double s0 = std::max(std::max(std::max((double)cx1/(cx1_0 - inner1.x), (double)cy1/(cy1_0 - inner1.y)),
                                              (double)(newImgSize.width - cx1)/(inner1.x + inner1.width - cx1_0)),
                                     (double)(newImgSize.height - cy1)/(inner1.y + inner1.height - cy1_0));
                s0 = std::max(std::max(std::max(std::max((double)cx2/(cx2_0 - inner2.x), (double)cy2/(cy2_0 - inner2.y)),
                                                (double)(newImgSize.width - cx2)/(inner2.x + inner2.width - cx2_0)),
                                       (double)(newImgSize.height - cy2)/(inner2.y + inner2.height - cy2_0)),
                              s0);

                double s1 = std::min(std::min(std::min((double)cx1/(cx1_0 - outer1.x), (double)cy1/(cy1_0 - outer1.y)),
                                              (double)(newImgSize.width - cx1)/(outer1.x + outer1.width - cx1_0)),
                                     (double)(newImgSize.height - cy1)/(outer1.y + outer1.height - cy1_0));
                s1 = std::min(std::min(std::min(std::min((double)cx2/(cx2_0 - outer2.x), (double)cy2/(cy2_0 - outer2.y)),
                                                (double)(newImgSize.width - cx2)/(outer2.x + outer2.width - cx2_0)),
                                       (double)(newImgSize.height - cy2)/(outer2.y + outer2.height - cy2_0)),
                              s1);

                s = s0*(1 - alpha) + s1*alpha;
            }

            fc_new *= s;
            cc_new[0] = cvPoint2D64f(cx1, cy1);
            cc_new[1] = cvPoint2D64f(cx2, cy2);

            cvmSet(_P1, 0, 0, fc_new);
            cvmSet(_P1, 1, 1, fc_new);
            cvmSet(_P1, 0, 2, cx1);
            cvmSet(_P1, 1, 2, cy1);

            cvmSet(_P2, 0, 0, fc_new);
            cvmSet(_P2, 1, 1, fc_new);
            cvmSet(_P2, 0, 2, cx2);
            cvmSet(_P2, 1, 2, cy2);
            cvmSet(_P2, idx, 3, s*cvmGet(_P2, idx, 3));
        }

        if( matQ ) {
            double q[] =
                    {
                            1, 0, 0, -cc_new[0].x,
                            0, 1, 0, -cc_new[0].y,
                            0, 0, 0, fc_new,
                            0, 0, -1. / _t[idx],
                            (idx == 0 ? cc_new[0].x - cc_new[1].x : cc_new[0].y - cc_new[1].y) / _t[idx]
                    };
            CvMat Q = cvMat(4, 4, CV_64F, q);
            cvConvert(&Q, matQ);
        }
    }

    inline void math_flann_surf(const cv::Mat &img_1, const cv::Mat &img_2) {

        if( !img_1.data || !img_2.data ) {
            std::cout << " --(!) Error reading images " << std::endl;
            return;
        }

        //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
        int minHessian = 400;
        Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create();
        detector->setHessianThreshold(minHessian);

        std::vector<KeyPoint> keypoints_1, keypoints_2;
        cv::Mat descriptors_1, descriptors_2;
        detector->detectAndCompute( img_1, Mat(), keypoints_1, descriptors_1 );
        detector->detectAndCompute( img_2, Mat(), keypoints_2, descriptors_2 );

        //-- Step 2: Matching descriptor vectors using FLANN matcher
        FlannBasedMatcher matcher;
        std::vector< DMatch > matches;
        matcher.match( descriptors_1, descriptors_2, matches );

        //-- Quick calculation of max and min distances between keypoints
        double max_dist = 0; double min_dist = 100;
        for( int i = 0; i < descriptors_1.rows; i++ ) {
            double dist = matches[i].distance;
            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;
        }
        printf("-- Max dist : %f \n", max_dist );
        printf("-- Min dist : %f \n", min_dist );

        //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
        //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
        //-- small)
        //-- PS.- radiusMatch can also be used here.
        std::vector< DMatch > good_matches;
        for( int i = 0; i < descriptors_1.rows; i++ ) {
            if (matches[i].distance <= max(2 * min_dist, 0.02))
                good_matches.push_back(matches[i]);
        }

        //-- Draw only "good" matches
        Mat img_matches;
        drawMatches( img_1, keypoints_1, img_2, keypoints_2,
                     good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                     vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        cv::imshow( "FLANN Good Matches (SURF)", img_matches );
        double mean_error = 0.0;
        for( int i = 0; i < (int)good_matches.size(); i++ ) {
            int idx1 = good_matches[i].queryIdx;
            int idx2 = good_matches[i].trainIdx;
            cv::Point2f pt1 = keypoints_1[idx1].pt;
            cv::Point2f pt2 = keypoints_2[idx2].pt;
            std::cout << "\"-- Good Match [" << std::setw(2) << i << "] Keypoint 1: "
                      << std::setw(4) << idx1 << "  -- Keypoint 2: " << std::setw(4) << idx2 << " --> "
                      << pt1 << " <--> " << pt2 << std::endl;
            mean_error += std::abs(pt1.y-pt2.y);
        }
        mean_error /= good_matches.size();

        std::cout << "-- Mean Error (y): " << mean_error << std::endl;
    }

private:
    std::string str_param_dir_;
    cv::Mat rectify_map_[2][2];
    camodocal::EquidistantCameraPtr cam_l_;
    camodocal::EquidistantCameraPtr cam_r_;

public:
    cv::Mat K1_, D1_, K2_, D2_, R1_, P1_, R2_, P2_;
};

#endif //CAPTURE_IMGS_COC_STEREO_RECTIFY_H
