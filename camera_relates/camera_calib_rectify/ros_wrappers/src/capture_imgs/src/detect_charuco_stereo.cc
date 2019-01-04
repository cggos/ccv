#include "detect_charuco/detect_charuco_stereo.h"

void StereoCharucoDetector::process(const cv::Mat &img_l, const cv::Mat &img_r) {
    cv::Mat img_show_l;
    cv::Mat img_show_r;
    charuco_detector_l_.detect(img_l, img_show_l);
    charuco_detector_r_.detect(img_r, img_show_r);

    {
        cv::Mat img_concat;
        cv::hconcat(img_show_l, img_show_r, img_concat);

        putText(img_concat, "Press 's' to add current frame. 'c' to finish and calibrate",
                Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);

        cv::imshow("StereoCharucoDetector", img_concat);
    }

    char key = (char) waitKey(10);

    static int n_count = 0;
    if (key == 's') {
        cout << "Frame captured " << n_count << endl;
        charuco_detector_l_.add_corners();
        charuco_detector_r_.add_corners();
        n_count++;
    }

    if (key == 'c') {
        vector<vector<Point2f> > vec_charuco_corners_l;
        vector<vector<Point3f> > vec_obj_points_l;
        vector<vector<Point2f> > vec_charuco_corners_r;
        vector<vector<Point3f> > vec_obj_points_r;
        charuco_detector_l_.get_calib_data(vec_charuco_corners_l, vec_obj_points_l);
        charuco_detector_r_.get_calib_data(vec_charuco_corners_r, vec_obj_points_r);

        std::cout << "size: " << vec_charuco_corners_l.size() << std::endl;
    }
}
