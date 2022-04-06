//
// Created by cg on 10/24/19.
//

#include "cvkit/cv/corner_detector.h"

#include <cmath>

namespace cg {

    CornerDetector::CornerDetector(int n_rows, int n_cols, double fast_threshold, double detection_threshold) :
            grid_n_rows_(n_rows), grid_n_cols_(n_cols),
            fast_threshold_(fast_threshold),
            detection_threshold_(detection_threshold) {
        occupancy_grid_.clear();
        occupancy_grid_.resize(grid_n_rows_ * grid_n_cols_, false);
    }

    void CornerDetector::set_grid_size(int n_rows, int n_cols) {
        grid_n_rows_ = n_rows;
        grid_n_cols_ = n_cols;
        occupancy_grid_.resize(grid_n_rows_ * grid_n_cols_, false);
    }

    int CornerDetector::sub2ind(const cg::Point2f &sub) {
        return static_cast<int>(sub.y / grid_height_) * grid_n_cols_ + static_cast<int>(sub.x / grid_width_);
    }

    void CornerDetector::zero_occupancy_grid() {
        std::fill(occupancy_grid_.begin(), occupancy_grid_.end() - 1, false);
    }

    void CornerDetector::set_grid_position(const cg::Point2f &pos) {
        occupancy_grid_[sub2ind(pos)] = true;
    }

    // Function from rpg_vikit - no need to clone whole repo
    // https://github.com/uzh-rpg/rpg_vikit
    float CornerDetector::shiTomasiScore(const cg::YImg8 &img, int u, int v) {
//        assert(img.type() == CV_8UC1);

        float dXX = 0.0;
        float dYY = 0.0;
        float dXY = 0.0;
        const int halfbox_size = 15;
        const int box_size = 2 * halfbox_size;
        const int box_area = box_size * box_size;
        const int x_min = u - halfbox_size;
        const int x_max = u + halfbox_size;
        const int y_min = v - halfbox_size;
        const int y_max = v + halfbox_size;

        if (x_min < 1 || x_max >= img.cols() - 1 || y_min < 1 || y_max >= img.rows() - 1)
            return 0.0; // patch is too close to the boundary

        const int stride = img.cols(); // img.step.p[0];
        for (int y = y_min; y < y_max; ++y) {
            const uint8_t *ptr_left = img.data() + stride * y + x_min - 1;
            const uint8_t *ptr_right = img.data() + stride * y + x_min + 1;
            const uint8_t *ptr_top = img.data() + stride * (y - 1) + x_min;
            const uint8_t *ptr_bottom = img.data() + stride * (y + 1) + x_min;
            for (int x = 0; x < box_size; ++x, ++ptr_left, ++ptr_right, ++ptr_top, ++ptr_bottom) {
                float dx = *ptr_right - *ptr_left;
                float dy = *ptr_bottom - *ptr_top;
                dXX += dx * dx;
                dYY += dy * dy;
                dXY += dx * dy;
            }
        }

        // Find and return smaller eigenvalue:
        dXX = dXX / (2.0 * box_area);
        dYY = dYY / (2.0 * box_area);
        dXY = dXY / (2.0 * box_area);
        return 0.5 * (dXX + dYY - std::sqrt((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY)));
    }

    void CornerDetector::detect_features(const cg::YImg8 &image, std::vector<cg::Point2f> &features, std::vector<double> &nm_scores) {
        grid_height_ = (image.rows() / grid_n_rows_) + 1;
        grid_width_  = (image.cols()/ grid_n_cols_) + 1;

        features.clear();
        std::vector<double> score_table(grid_n_rows_ * grid_n_cols_);
        std::vector<cg::Point2f> feature_table(grid_n_rows_ * grid_n_cols_);
        std::vector<fast::fast_xy> fast_corners;

#ifdef __SSE2__
        fast::fast_corner_detect_10_sse2(
                (fast::fast_byte *) image.data(), image.cols(),
                image.rows(), image.cols(), 20, fast_corners);
#else
        fast::fast_corner_detect_10(
                (fast::fast_byte*) image.data, image.cols,
                image.rows, image.cols, 20, fast_corners);
#endif

        std::vector<int> scores, nm_corners;
        fast::fast_corner_score_10((fast::fast_byte *) image.data(), image.cols(), fast_corners, fast_threshold_, scores);
        fast::fast_nonmax_3x3(fast_corners, scores, nm_corners);

        // ALEX: Updated loop
        for (auto it:nm_corners) {
            fast::fast_xy &xy = fast_corners.at(it);
            if (xy.x >= grid_n_cols_ * grid_width_ ||
                xy.y >= grid_n_rows_ * grid_height_)
                continue;
            const int k = sub2ind(cg::Point2f(xy.x, xy.y));
            if (occupancy_grid_[k])
                continue;
            const float score = shiTomasiScore(image, xy.x, xy.y);
            if (score > score_table[k]) {
                score_table[k] = static_cast<double>(score);
                feature_table[k] = cg::Point2f(xy.x, xy.y);
            }
        }

        // Create feature for every corner that has high enough corner score
        // ALEX: Replaced corner object from original code
        for (int i = 0; i < score_table.size(); i++) {
            if (score_table[i] > detection_threshold_) {
                cg::Point2f pos = feature_table[i];
                features.push_back(cg::Point2f(pos.x, pos.y));
                nm_scores.push_back(score_table[i]);
            }
        }
        zero_occupancy_grid();
    }
}

