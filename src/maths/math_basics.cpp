//
// Created by cg on 9/21/19.
//

#include "ccv/maths/math_basics.h"

#include <cmath>
#include <random>

namespace cg {

    unsigned long int g_state;

    int uniform_integer(int min, int max) {
        std::random_device rd;  //Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
        std::uniform_int_distribution<> dis(min, max);
        return dis(gen);
    }

    void rng_mwc_init(unsigned long int state) { g_state = state; }

    unsigned int rng_mwc_next() {
        g_state = (unsigned long int)(unsigned)g_state* /*CV_RNG_COEFF*/ 4164903690U + (unsigned)(g_state >> 32);
        return (unsigned)g_state;
    }

    double chi_square_table_p95[99] = {
            0.00393, 0.10259, 0.35185, 0.71072, 1.14548,
            1.63538, 2.16735, 2.73264, 3.32511, 3.94030,
            4.57481, 5.22603, 5.89186, 6.57063, 7.26094,
            7.96165, 8.67176, 9.39046, 10.11701, 10.85081,
            11.59131, 12.33801, 13.09051, 13.84843, 14.61141,
            15.37916, 16.15140, 16.92788, 17.70837, 18.49266,
            19.28057, 20.07191, 20.86653, 21.66428, 22.46502,
            23.26861, 24.07494, 24.88390, 25.69539, 26.50930,
            27.32555, 28.14405, 28.96472, 29.78748, 30.61226,
            31.43900, 32.26762, 33.09808, 33.93031, 34.76425,
            35.59986, 36.43709, 37.27589, 38.11622, 38.95803,
            39.80128, 40.64593, 41.49195, 42.33931, 43.18796,
            44.03787, 44.88902, 45.74138, 46.59491, 47.44958,
            48.30538, 49.16227, 50.02023, 50.87924, 51.73928,
            52.60031, 53.46233, 54.32531, 55.18923, 56.05407,
            56.91982, 57.78645, 58.65394, 59.52229, 60.39148,
            61.26148, 62.13229, 63.00389, 63.87626, 64.74940,
            65.62328, 66.49790, 67.37323, 68.24928, 69.12603,
            70.00346, 70.88157, 71.76034, 72.63977, 73.51984,
            74.40054, 75.28186, 76.16379, 77.04633
    };
}
