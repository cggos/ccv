//
// Created by cg on 9/17/19.
//

#ifndef VIKIT_CG_MATH_BASICS_H
#define VIKIT_CG_MATH_BASICS_H

namespace cg {

    int uniform_integer(int min, int max);

    /// from cv::RNG wihich uses Multiply-With-Carry algorithm
    void rng_mwc_init(unsigned long int state);
    unsigned int rng_mwc_next();

    // boost::math::chi_squared chi_squared_dist(i);
    // chi_squared_test_table[i] = boost::math::quantile(chi_squared_dist, 0.05);
    extern double chi_square_table_p95[99]; // P-value: 0.95, DoF: 1~99,
}

#endif //VIKIT_CG_MATH_BASICS_H
