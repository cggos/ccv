//
// Created by cg on 11/2/19.
//

#ifndef MSCKF_IMAGE_FILTERING_H
#define MSCKF_IMAGE_FILTERING_H

#include "ccv/cv/yimg.h"

namespace cg {

    /**
     * @brief generate gaussian template
     * @param m
     * @param sigma
     * @return pointer pointing to gaussian template generated
     *       \f[
     *          f(x,y) = \frac{1}{2\pi{\sigma}^2}
     *                   e^
     *                   {
     *                     -\frac{
     *                             (x-\frac{m}{2})^2 + (y-\frac{n}{2})^2
     *                           }
     *                           {2{\sigma}^2}
     *                   }
     *       \f]
     */
    inline double *generate_gaussian_template(unsigned int m, double sigma = 0.84089642);

    void gaussian_blur(const YImg8 &img_src, YImg8 &img_dst, unsigned int m, double sigma = 0.84089642);

    void pyr_down(const YImg8 &img_src, YImg8 &img_dst);
}

#endif //MSCKF_IMAGE_FILTERING_H
