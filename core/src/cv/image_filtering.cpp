//
// Created by cg on 11/2/19.
//

#include "cvkit/cv/image_filtering.h"

#include <cmath>

namespace cg {

    double *generate_gaussian_template(unsigned int m, double sigma) {
        unsigned int n = m;
        double *gaussian_template = new double[m * n];
        double sum = 0.0;
        for (int y = 0; y < m; y++) {
            for (int x = 0; x < n; x++) {
                double r2 = std::pow((double) x - m / 2, 2) + std::pow((double) y - n / 2, 2);
                double sigma2Inv = 1.f / (2 * std::pow(sigma, 2));
                double exp = std::exp(-r2 * sigma2Inv);
                sum += *(gaussian_template + y * m + x) = sigma2Inv / M_PI * exp;
            }
        }
        double sumInv = 0.0;
        if (sum > 1e-6)
            sumInv = 1.0 / sum;
        for (int y = 0; y < m; y++)
            for (int x = 0; x < n; x++)
                *(gaussian_template + y * m + x) *= sumInv;
        return gaussian_template;
    }

    void gaussian_blur(const YImg8 &img_src, YImg8 &img_dst, unsigned int m, double sigma) {
        double *gaussian_template = generate_gaussian_template(m, sigma);
        unsigned int k = m / 2;
        for (unsigned int h = 0; h < img_src.rows(); ++h) {
            for (unsigned int w = 0; w < img_src.cols(); ++w) {
                if (h >= k && h < img_src.rows() - k && w >= k && w < img_src.cols() - k) {
                    float sum = 0;
                    for (unsigned int y = 0; y < m; ++y) {
                        for (unsigned int x = 0; x < m; ++x) {
                            sum += img_src(h - k + y, w - k + x) * *(gaussian_template + y * m + x);
                        }
                    }
                    img_dst(h, w) = (int) sum;
                } else {
                    img_dst(h, w) = img_src(h, w); // for borders
                }
            }
        }
        delete gaussian_template;
    }

    void pyr_down(const YImg8 &img_src, YImg8 &img_dst) {
        img_dst = YImg8(img_src.size() / 2);
        for (int i = 0; i < img_src.rows(); i += 2)
            for (int j = 0; j < img_src.cols(); j += 2)
                img_dst(i / 2, j / 2) = img_src(i, j);
    }
}

