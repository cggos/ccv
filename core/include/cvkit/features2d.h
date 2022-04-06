#ifndef CGOCV_Features2D_H
#define CGOCV_Features2D_H

#include "cvkit/image.h"

namespace cg {

    class Features2D {
    public:
        struct KeyPoint {
            Point2D<int> pt;
            float size;
            float response;
        };

        enum Type {
            FAST_5_8,
            FAST_9_16
        };

    public:
        Features2D() {}

        ~Features2D() {}

        static double shi_tomasi_score(const Image<unsigned char> &image, unsigned int halfbox_size, Point2D<int> pt);
        template <class _F, class _T>
        static _F sum_of_squared_distance(Image<_T> &img1, Image<_T> &img2);
        static int zero_mean_sum_of_squared_distance(Image<unsigned char> &img1, Image<unsigned char> &img2, int nTemplateSum, int nTemplateSumSq);
    };

    template <class _F, class _T>
    _F Features2D::sum_of_squared_distance(Image<_T> &img1, Image<_T> &img2)
    {
        auto max_ssd = (_F)9999;

        if(img1.empty() || img2.empty())
            return max_ssd+1;

        cg::Size size1 = img1.size();
        cg::Size size2 = img2.size();
        if(size1 != size2)
            return max_ssd+1;

        _F nSumSqDiff = 0;
        for(unsigned int nRow = 0; nRow < size1.height; nRow++) {
            for (unsigned int nCol = 0; nCol < size1.width; nCol++) {
                _F nDiff = (int) img1(nRow, nCol) - img2(nRow, nCol);
                nSumSqDiff += nDiff * nDiff;
            }
        }
        return nSumSqDiff;
    }

}

#endif //CGOCV_Features2D_H
