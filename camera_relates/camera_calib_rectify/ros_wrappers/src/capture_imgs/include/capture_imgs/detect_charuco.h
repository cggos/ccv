#ifndef CAPTURE_IMGS_DETECT_CHARUCO_H
#define CAPTURE_IMGS_DETECT_CHARUCO_H

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <vector>
#include <iostream>

using namespace std;
using namespace cv;

class CharucoDetector {
public:
    CharucoDetector() {}
    CharucoDetector(int id_dictionary,
            int squares_x, int squares_y,
            float square_length, float marker_length)
            : is_calib_started_(false) {
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(id_dictionary));

        detector_params_ = cv::aruco::DetectorParameters::create();

        charuco_board_ = cv::aruco::CharucoBoard::create(squares_x, squares_y, square_length, marker_length, dictionary_);
        board_ = charuco_board_.staticCast<cv::aruco::Board>();
    }

    ~CharucoDetector() {}

    inline void detect(
            const cv::Mat &image, cv::Mat &img_show,
            bool is_refine=true, bool is_show_markers=false, bool is_show_rejected=false) {

        p_image_ = &image;

        aruco::detectMarkers(image, dictionary_, marker_corners_, marker_ids_, detector_params_, rejected_markers_);

        if(is_refine)
            aruco::refineDetectedMarkers(image, board_, marker_corners_, marker_ids_, rejected_markers_);

        cv::cvtColor(*p_image_, img_show, COLOR_GRAY2BGR);
        {
            vector<int> charuco_ids;
            vector<Point2f> charuco_corners;
            int interpolated_corners = 0;
            if (marker_ids_.size() > 0)
                interpolated_corners =
                        aruco::interpolateCornersCharuco(marker_corners_, marker_ids_, *p_image_, charuco_board_,
                                                         charuco_corners, charuco_ids);
            if (interpolated_corners > 0) {
                Scalar color;
                color = Scalar(255, 0, 0);
                aruco::drawDetectedCornersCharuco(img_show, charuco_corners, charuco_ids, color);
            }
            if(is_show_markers) {
                if (marker_ids_.size() > 0)
                    aruco::drawDetectedMarkers(img_show, marker_corners_);
                if (is_show_rejected && rejected_markers_.size() > 0)
                    aruco::drawDetectedMarkers(img_show, rejected_markers_, noArray(), Scalar(100, 0, 255));
            }
        }
    }

    inline void add_corners() {
        if(marker_ids_.size() > 0) {
            all_corners_.push_back(marker_corners_);
            all_ids_.push_back(marker_ids_);
            all_imgs_.push_back(*p_image_);
        }
    }

    inline void get_calib_data(
            vector<vector<Point2f> > &all_charuco_corners,
            vector<vector<Point3f> > &all_obj_points) {

        int n_frames = (int) all_corners_.size();
        all_charuco_corners.reserve(n_frames);

        vector<vector<int> > all_charuco_ids;
        all_charuco_ids.reserve(n_frames);

        for (int i = 0; i < n_frames; i++) {
            // interpolate using camera parameters
            vector<Point2f> current_charuco_corners;
            vector<int> current_charuco_ids;
            aruco::interpolateCornersCharuco(all_corners_[i], all_ids_[i], all_imgs_[i], charuco_board_,
                                             current_charuco_corners, current_charuco_ids);
            all_charuco_corners.push_back(current_charuco_corners);
            all_charuco_ids.push_back(current_charuco_ids);
        }

        if (all_charuco_corners.size() < 4) {
            cerr << "Not enough corners for calibration" << endl;
            return;
        }

        CV_Assert(all_charuco_ids.size() > 0 && (all_charuco_ids.size() == all_charuco_corners.size()));

        // Join object points of charuco corners in a single vector for calibrateCamera() function

        all_obj_points.resize(all_charuco_ids.size());
        for (unsigned int i = 0; i < all_charuco_ids.size(); i++) {
            unsigned int nCorners = (unsigned int) all_charuco_ids[i].size();
            CV_Assert(nCorners > 0 && nCorners == all_charuco_corners[i].size());
            all_obj_points[i].reserve(nCorners);

            for (unsigned int j = 0; j < nCorners; j++) {
                int point_id = all_charuco_ids[i][j];
                CV_Assert(point_id >= 0 && point_id < (int) charuco_board_->chessboardCorners.size());
                all_obj_points[i].push_back(charuco_board_->chessboardCorners[point_id]);
            }
        }
    }

public:
    bool is_calib_started_;

private:
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    cv::Ptr<cv::aruco::CharucoBoard> charuco_board_;
    cv::Ptr<cv::aruco::Board> board_;

    const cv::Mat *p_image_;

    vector< int > marker_ids_;
    vector< vector< Point2f > > marker_corners_;
    vector< vector< Point2f > > rejected_markers_;

    vector< vector< vector< Point2f > > > all_corners_;
    vector< vector< int > > all_ids_;
    vector< Mat > all_imgs_;
};

#endif //CAPTURE_IMGS_DETECT_CHARUCO_H
