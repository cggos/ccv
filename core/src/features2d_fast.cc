#include "cvkit/features2d_fast.h"
#include "cvkit/nonmax_suppression.h"

namespace cg {

    cg::Point2D<int> FAST::fast_pixel_ring[16]=
            {
                    cg::Point2D<int>(0, 3),
                    cg::Point2D<int>(1, 3),
                    cg::Point2D<int>(2, 2),
                    cg::Point2D<int>(3, 1),
                    cg::Point2D<int>(3, 0),
                    cg::Point2D<int>(3, -1),
                    cg::Point2D<int>(2, -2),
                    cg::Point2D<int>(1, -3),
                    cg::Point2D<int>(0, -3),
                    cg::Point2D<int>(-1, -3),
                    cg::Point2D<int>(-2, -2),
                    cg::Point2D<int>(-3, -1),
                    cg::Point2D<int>(-3, 0),
                    cg::Point2D<int>(-3, 1),
                    cg::Point2D<int>(-2, 2),
                    cg::Point2D<int>(-1, 3),
            };

    FAST::FAST(Features2D::Type type) : type_(type) {
        switch (type) {
            case Features2D::FAST_5_8:
                pattern_size_ = 8;
                break;
            case Features2D::FAST_9_16:
                pattern_size_ = 16;
                margin_ = 3;
                break;
        }
    }

    int FAST::old_style_corner_score(const cg::Image<unsigned char> &img, cg::Point2D<int> c, const int *pointer_dir, int barrier)
    {
        //The score for a positive feature is sum of the difference between the pixels
        //and the barrier if the difference is positive. Negative is similar.
        //The score is the max of those two.
        //
        // B = {x | x = points on the Bresenham circle around c}
        // Sp = { I(x) - t | x E B , I(x) - t > 0 }
        // Sn = { t - I(x) | x E B, t - I(x) > 0}
        //
        // Score = max sum(Sp), sum(Sn)

        const unsigned char *imp = &img[c];

        int cb = *imp + barrier;
        int c_b = *imp - barrier;
        int sp=0, sn = 0;

        for(int i=0; i<16; i++)
        {
            int p = imp[pointer_dir[i]];

            if(p > cb)
                sp += p-cb;
            else if(p < c_b)
                sn += c_b-p;
        }

        if(sp > sn)
            return sp;
        else
            return sn;
    }

    void FAST::compute_fast_score_old(const cg::Image<unsigned char> &img, const std::vector<cg::Point2D<int> > &corners, int barrier, std::vector<int> &scores)
    {
        int	pointer_dir[16];
        for(int i=0; i < 16; i++)
            pointer_dir[i] = fast_pixel_ring[i].x + fast_pixel_ring[i].y * img.size().width;

        scores.resize(corners.size());

        for(unsigned int i=0; i < corners.size(); i++)
            scores[i] = old_style_corner_score(img, corners[i], pointer_dir, barrier);
    }

    void FAST::fast_nonmax(const cg::Image<unsigned char> &img, const std::vector<cg::Point2D<int> >& corners, int barrier, std::vector<cg::Point2D<int> > &max_corners)
    {
        std::vector<int> scores;
        compute_fast_score_old(img, corners, barrier, scores);
        NonMaxSuppression::nonmax_suppression_t<int, cg::Point2D<int>, collect_pos>(corners, scores, max_corners);
    }

    void FAST::detect_1(const unsigned char *img_data, unsigned int width, unsigned int height,
                        std::vector<KeyPoint> &key_ponits, int threshold) {
        threshold = std::min(std::max(threshold, 0), 255);

        KeyPoint key_point;
        for (unsigned int h = margin_; h < height - margin_; ++h) {
            for (unsigned int w = margin_; w < width - margin_; ++w) {
                unsigned int pos_base = width * h + w;
                unsigned char intensity = img_data[pos_base];
                int brighter = intensity + threshold;
                int darker = intensity - threshold;
                unsigned int pos_i;
                unsigned char begin = 0;
                unsigned char end = 15;
                unsigned char count_contiguous = 1;
                for (unsigned char i = begin; i <= end; ++i) {
                    if (i == 0) {
                        unsigned int pos_0 = get_bresenmancicle_pos(pos_base, width, 0);
                        pos_i = get_bresenmancicle_pos(pos_base, width, ++i);
                        while ((img_data[pos_0] > brighter && img_data[pos_i] > brighter) ||
                               (img_data[pos_0] < darker && img_data[pos_i] < darker)) {
                            pos_i = get_bresenmancicle_pos(pos_base, width, ++i);
                        }
                        begin = i;
                        if (i < 9) {
                            bool is_corner = true;
                            for (; end > end - i; end--) {
                                pos_i = get_bresenmancicle_pos(pos_base, width, end);
                                if ((img_data[pos_0] > brighter && img_data[pos_i] > brighter) ||
                                    (img_data[pos_0] < darker && img_data[pos_i] < darker)) {
                                } else {
                                    is_corner = false;
                                    break;
                                }
                            }
                            if (is_corner) {
                                key_point.pt.x = w;
                                key_point.pt.y = h;
                                key_ponits.push_back(key_point);
                            }
                        }
                    }
                    pos_i = get_bresenmancicle_pos(pos_base, width, ++i);
                    if ((img_data[begin] > brighter && img_data[pos_i] > brighter) ||
                        (img_data[begin] < darker && img_data[pos_i] < darker)) {
                        ++count_contiguous;
                    } else {
                        if (count_contiguous < 9) {
                            count_contiguous = 1;
                        }
                    }
                    if (count_contiguous >= 9) {
                        key_point.pt.x = w;
                        key_point.pt.y = h;
                        key_ponits.push_back(key_point);
                    }
                }
            }
        }
    }

    void FAST::detect_2(const unsigned char *img_data, unsigned int width, unsigned int height,
                        std::vector<cg::Point2D<int> > &vCorners, int threshold) {
        vCorners.resize(0);

        int offset[16] = {0};
        for (int k = 0; k < 16; k++)
            offset[k] = get_bresenmancicle_pos(0, width, k);

        int flag[32] = {0};
        for (unsigned int y = margin_; y < height - margin_; y++) {
            for (unsigned int x = margin_; x < width - margin_; x++) {
                register const unsigned char *const ptr = img_data + y * width + x;
                register const int cb = *ptr + threshold;
                register const int c_b = *ptr - threshold;
                for (int k = 0; k < 16; k++) {
                    if (ptr[offset[k]] > cb) {
                        flag[k] = 1;
                        flag[k + 16] = 1;

                    } else if (ptr[offset[k]] < c_b) {
                        flag[k] = -1;
                        flag[k + 16] = -1;
                    } else {
                        flag[k] = 0;
                        flag[k + 16] = 0;
                    }

                }
                int temp = 0;
                int count = 0;
                for (int k = 0; k < 31; k++) {
                    temp = flag[k] * flag[k + 1];
                    if (temp > 0) {
                        count++;
                    } else {
                        count = 0;
                    }
                    if (count >= 8) {
                        vCorners.push_back(cg::Point2D<int>(x,y));
                        break;
                    }
                }
            }
        }
    }

}
