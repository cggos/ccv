#include "capture_imgs/capture_stereo_charuco_nodelet.h"

#include <boost/filesystem.hpp>
#include <sensor_msgs/distortion_models.h>

namespace fs=boost::filesystem;

namespace capture_imgs {

    void CaptureStereoCharuco::process(const cv::Mat &img_l, const cv::Mat &img_r) {

        if(is_rectify_)
            rectify(img_l, img_r);
        else {
            static bool is_init_calib = false;
            if (!is_init_calib) {
                coc_stereo_calib_.init(cv::Size(img_l.cols, img_l.rows));
                is_init_calib = true;
            }
            calib(img_l, img_r);
        }
    }

    void CaptureStereoCharuco::calib(const cv::Mat &img_l, const cv::Mat &img_r) {

        char key = (char) waitKey(10);

        cv::Mat img_show_l;
        cv::Mat img_show_r;
        charuco_detector_l_.detect(img_l, img_show_l);
        charuco_detector_r_.detect(img_r, img_show_r);

        {
            putText(img_show_l, "left",  Point(10, 40), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
            putText(img_show_r, "right", Point(10, 40), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);

            cv::Mat img_concat;

            cv::hconcat(img_show_l, img_show_r, img_concat);

            putText(img_concat, "Press 's' to add current frame. 'c' to finish and calibrate",
                    Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);

            cv::imshow("CharucoDetector", img_concat);
        }

        static int n_count = 0;
        if (key == 's') {
            cout << "Frame captured " << n_count << endl;
            charuco_detector_l_.add_corners();
            charuco_detector_r_.add_corners();
            n_count++;
        }

        if (key == 'c') {

            std::cout << "capture imgs end, start calib..." << std::endl;

            vector<vector<int> > all_charuco_ids_l;
            vector<vector<Point2f> > vec_charuco_corners_l;
            vector<vector<Point3f> > vec_obj_points_l;

            vector<vector<int> > all_charuco_ids_r;
            vector<vector<Point2f> > vec_charuco_corners_r;
            vector<vector<Point3f> > vec_obj_points_r;

            charuco_detector_l_.get_calib_data(all_charuco_ids_l, vec_charuco_corners_l, vec_obj_points_l);
            charuco_detector_r_.get_calib_data(all_charuco_ids_r, vec_charuco_corners_r, vec_obj_points_r);

            coc_stereo_calib_.add_calib_data(
                    all_charuco_ids_l, vec_charuco_corners_l, vec_obj_points_l,
                    all_charuco_ids_r, vec_charuco_corners_r, vec_obj_points_r);

            coc_stereo_calib_.calib();

            n_count = 0;

            std::cout << "\n====================== calib complete ======================\n" << std::endl;
        }
    }

    void CaptureStereoCharuco::rectify(const cv::Mat &img_l, const cv::Mat &img_r) {

        img_rect_l_ = img_l.clone();
        img_rect_r_ = img_r.clone();

        coc_stereo_rectify_.rectify(img_rect_l_, img_rect_r_);

#if __USER_DEBUG__
        char key = (char) waitKey(10);
        {
            cv::Mat img_concat;
            cv::hconcat(img_rect_l_, img_rect_r_, img_concat);
            cv::cvtColor(img_concat, img_concat, COLOR_GRAY2BGR);
            for (int i = 0; i < img_concat.rows; i += 32)
                cv::line(img_concat, Point(0, i), Point(img_concat.cols, i), Scalar(0, 255, 0), 1, 8);

            putText(img_concat, "Press 'm' to show match with surf, and 's' to save imgs",
                    Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);

            cv::imshow("rect", img_concat);

            if (key == 'm') {
#if WITH_MATCH
                coc_stereo_rectify_.math_flann_surf(img_rect_l_, img_rect_r_);
#endif                
            }
            if (key == 's') {
                static int n = 0;
                fs::path cur_path(fs::current_path());
                std::string str_dir = cur_path.string() + "/";
                char name_l[255];
                char name_r[255];
                sprintf(name_l, "image_%d_l.png", n);
                sprintf(name_r, "image_%d_r.png", n);
                std::string str_l = name_l;
                std::string str_r = name_r;
                std::string str_img_l = str_dir + str_l;
                std::string str_img_r = str_dir + str_r;
                cv::imwrite(str_img_l, img_rect_l_);
                cv::imwrite(str_img_r, img_rect_r_);
                std::cout << "capture imgs: " << n << ", and save left and right rect images in: " << str_dir << std::endl;
                n++;
            }
        }
#endif
    }

    void CaptureStereoCharuco::pub_img_rect(
            const cv::Mat &img_l, const cv::Mat &img_r,
            const std_msgs::Header &header_l,
            const std_msgs::Header &header_r) {

        sensor_msgs::Image img_msg_l;
        cv_bridge::CvImage(header_l, sensor_msgs::image_encodings::MONO8, img_l).toImageMsg(img_msg_l);

        sensor_msgs::Image img_msg_r;
        cv_bridge::CvImage(header_r, sensor_msgs::image_encodings::MONO8, img_r).toImageMsg(img_msg_r);

        sensor_msgs::CameraInfo cam_info_l;
        cam_info_l.header = img_msg_l.header;
        cam_info_l.width  = img_l.cols;
        cam_info_l.height = img_l.rows;
        cam_info_l.distortion_model = sensor_msgs::distortion_models::EQUIDISTANT;

        cam_info_l.D.resize(4);
        for(int i=0; i<4; ++i)
            cam_info_l.D[i] = ((double*)coc_stereo_rectify_.D1_.data)[i];

        for(int i=0; i<9; ++i)
            cam_info_l.K[i] = ((double*)coc_stereo_rectify_.K1_.data)[i];

        for(int i=0; i<9; ++i)
            cam_info_l.R[i] = ((double*)coc_stereo_rectify_.R1_.data)[i];

        for(int i=0; i<12; ++i)
            cam_info_l.P[i] = ((double*)coc_stereo_rectify_.P1_.data)[i];


        sensor_msgs::CameraInfo cam_info_r;
        cam_info_r.header = img_msg_l.header;
        cam_info_r.width  = img_r.cols;
        cam_info_r.height = img_r.rows;
        cam_info_r.distortion_model = sensor_msgs::distortion_models::EQUIDISTANT;

        cam_info_r.D.resize(4);
        for(int i=0; i<4; ++i)
            cam_info_r.D[i] = ((double*)coc_stereo_rectify_.D2_.data)[i];

        for(int i=0; i<9; ++i)
            cam_info_r.K[i] = ((double*)coc_stereo_rectify_.K2_.data)[i];

        for(int i=0; i<9; ++i)
            cam_info_r.R[i] = ((double*)coc_stereo_rectify_.R2_.data)[i];

        for(int i=0; i<12; ++i)
            cam_info_r.P[i] = ((double*)coc_stereo_rectify_.P2_.data)[i];


//        rect_pub_l_.publish(img_msg_l, cam_info_l, header_l.stamp);
//        rect_pub_r_.publish(img_msg_r, cam_info_r, header_r.stamp);

        rect_it_pub_l_.publish(img_msg_l);
        rect_it_pub_r_.publish(img_msg_r);
        rect_caminfo_pub_l_.publish(cam_info_l);
        rect_caminfo_pub_r_.publish(cam_info_r);
    }

}


