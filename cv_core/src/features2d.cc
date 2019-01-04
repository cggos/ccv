#include "cgocv/features2d.h"

#include <cmath>

namespace cg {

    double Features2D::shi_tomasi_score(const Image<unsigned char> &image, unsigned int halfbox_size, Point2D<int> point) {
        double dXX = 0.0;
        double dYY = 0.0;
        double dXY = 0.0;

        Point2D<int> ptStart = point - Point2D<int>(halfbox_size, halfbox_size);
        Point2D<int> ptEnd   = point + Point2D<int>(halfbox_size, halfbox_size);

        Point2D<int> pt;
        for(pt.y = ptStart.y; pt.y<=ptEnd.y; pt.y++)
            for(pt.x = ptStart.x; pt.x<=ptEnd.x; pt.x++)
            {
                double dx = image[pt + Point2D<int>(1,0)] - image[pt - Point2D<int>(1,0)];
                double dy = image[pt + Point2D<int>(0,1)] - image[pt - Point2D<int>(0,1)];
                dXX += dx*dx;
                dYY += dy*dy;
                dXY += dx*dy;
            }

        int nPixels = (ptEnd - ptStart + Point2D<int>(1,1)).area();
        dXX = dXX / (2.0 * nPixels);
        dYY = dYY / (2.0 * nPixels);
        dXY = dXY / (2.0 * nPixels);

        // Find and return smaller eigenvalue:
        return 0.5 * (dXX + dYY - std::sqrt( (dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY) ));
    }

    int Features2D::zero_mean_sum_of_squared_distance(Image<unsigned char> &img1, Image<unsigned char> &img2, int nTemplateSum, int nTemplateSumSq)
    {
        unsigned int max_ssd = 9999;

        if(img1.empty() || img2.empty())
            return max_ssd+1;

        cg::Size size1 = img1.size();
        cg::Size size2 = img2.size();
        if(size1 != size2)
            return max_ssd+1;

        int nImageSumSq = 0;
        int nImageSum = 0;
        int nCrossSum = 0;
        for(unsigned int nRow = 0; nRow < size1.height; nRow++) {
            for (unsigned int nCol = 0; nCol < size1.width; nCol++) {
                int n = (int) img1(nRow, nCol);
                nImageSum += n;
                nImageSumSq += n * n;
                nCrossSum += n * img2(nRow, nCol);
            }
        }

        int SA = nTemplateSum;
        int SB = nImageSum;
        int N = size1.area();

        return ((2*SA*SB - SA*SA - SB*SB)/N + nImageSumSq + nTemplateSumSq - 2*nCrossSum);
    }
}
