#ifndef CGOCV_Feature2dFAST_H
#define CGOCV_Feature2dFAST_H

#include <vector>

#include "cvkit/cv/features2d.h"

namespace cg {

    class FAST : public Features2D {

    public:
        FAST(Features2D::Type type);

        ~FAST() {}

        static void fast_nonmax(const cg::Image<unsigned char> &img, const std::vector<cg::Point2D<int> >& corners, int barrier, std::vector<cg::Point2D<int> > &max_corners);

        void detect_1(const unsigned char *img_data,
                      unsigned int width,
                      unsigned int height,
                      std::vector<KeyPoint> &key_ponits,
                      int threshold = 50);

        void detect_2(const unsigned char *img_data,
                      unsigned int width,
                      unsigned int height,
                      std::vector<cg::Point2D<int> > &vCorners,
                      int threshold = 50);

    private:

        // The two collectors which either return just the ImageRef or the <ImageRef,int> pair
        struct collect_pos
        {
            static inline cg::Point2D<int> collect(const cg::Point2D<int> &pos, int ) {
                return pos;
            }
        };

        Features2D::Type type_;
        unsigned short margin_;
        unsigned short pattern_size_;

        inline unsigned int get_bresenmancicle_pos(unsigned int pos_base, unsigned int width, unsigned short i) {
            int bresenham_circle[][2] =
                    {
                            {-3, 0},
                            {-3, -1},
                            {-2, -2},
                            {-1, -3},
                            {0,  -3},
                            {1,  -3},
                            {2,  -2},
                            {3,  -1},
                            {3,  0},
                            {3,  1},
                            {2,  2},
                            {1,  3},
                            {0,  3},
                            {-1, 3},
                            {-2, 2},
                            {-3, 1}
                    };
            return pos_base + width * bresenham_circle[i][1] + bresenham_circle[i][0];
        }

        static cg::Point2D<int> fast_pixel_ring[16];

        static int old_style_corner_score(const cg::Image<unsigned char> &img, cg::Point2D<int> c, const int *pointer_dir, int barrier);
        static void compute_fast_score_old(const cg::Image<unsigned char> &img, const std::vector<cg::Point2D<int> > &corners, int barrier, std::vector<int> &scores);

    };

}

#endif //CGOCV_Feature2dFAST_H
