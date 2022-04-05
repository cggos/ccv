//
// Created by cg on 11/20/19.
//

#include <stdint.h>
#include <stdio.h>
#include <float.h>
#include <cmath>

#include "maths/math_basics.h"
#include "maths/matrix.h"

namespace cg {

    int givens(float *a, float *b, int n, float c, float s) {
        if (n < 4)
            return 0;
        int k = 0;

        for (; k <= n; k++) {
            float a0 = a[k];
            float b0 = b[k];
            float t0 = (a0 * c) + (b0 * s);
            float t1 = (b0 * c) - (a0 * s);
            a[k] = t0;
            b[k] = t1;
        }

        return k;
    }

    void jacobi_svd(float *At, size_t astep, float *_W, float *Vt, size_t vstep, int m, int n, int n1, double minval, float eps) {
        int i, j, k, iter, max_iter = std::max(m, 30);
        float c, s;
        double sd;
        astep /= sizeof(At[0]);
        vstep /= sizeof(Vt[0]);

        double *W = (double *) malloc(n * sizeof(double));

        for (i = 0; i < n; i++) {
            for (k = 0, sd = 0; k < m; k++) {
                float t = At[i * astep + k];
                sd += (double) t * t;
            }
            W[i] = sd;

            if (Vt) {
                for (k = 0; k < n; k++)
                    Vt[i * vstep + k] = 0;
                Vt[i * vstep + i] = 1;
            }
        }

        for (iter = 0; iter < max_iter; iter++) {
            bool changed = false;

            for (i = 0; i < n - 1; i++)
                for (j = i + 1; j < n; j++) {
                    float *Ai = At + i * astep, *Aj = At + j * astep;
                    double a = W[i], p = 0, b = W[j];

                    for (k = 0; k < m; k++)
                        p += (double) Ai[k] * Aj[k];

                    if (std::abs(p) <= eps * std::sqrt((double) a * b))
                        continue;

                    p *= 2;
                    double beta = a - b, gamma = hypot((double) p, beta);
                    if (beta < 0) {
                        double delta = (gamma - beta) * 0.5;
                        s = (float) std::sqrt(delta / gamma);
                        c = (float) (p / (gamma * s * 2));
                    } else {
                        c = (float) std::sqrt((gamma + beta) / (gamma * 2));
                        s = (float) (p / (gamma * c * 2));
                    }

                    a = b = 0;
                    for (k = 0; k < m; k++) {
                        float t0 = c * Ai[k] + s * Aj[k];
                        float t1 = -s * Ai[k] + c * Aj[k];
                        Ai[k] = t0;
                        Aj[k] = t1;

                        a += (double) t0 * t0;
                        b += (double) t1 * t1;
                    }
                    W[i] = a;
                    W[j] = b;

                    changed = true;

                    if (Vt) {
                        float *Vi = Vt + i * vstep, *Vj = Vt + j * vstep;
                        // 2019.11.19 SHENYANFEI
                        // k = givens(Vi, Vj, n, c, s);
                        // for (; k < n; k++)
                        for (k = 0; k < n; k++) {
                            float t0 = c * Vi[k] + s * Vj[k];
                            float t1 = -s * Vi[k] + c * Vj[k];
                            Vi[k] = t0;
                            Vj[k] = t1;
                        }
                    }
                }

            if (!changed)
                break;
        }

        for (i = 0; i < n; i++) {
            for (k = 0, sd = 0; k < m; k++) {
                float t = At[i * astep + k];
                sd += (double) t * t;
            }
            W[i] = std::sqrt(sd);
        }

        for (i = 0; i < n - 1; i++) {
            j = i;
            for (k = i + 1; k < n; k++) {
                if (W[j] < W[k])
                    j = k;
            }
            if (i != j) {
                std::swap(W[i], W[j]);
                if (Vt) {
                    for (k = 0; k < m; k++)
                        std::swap(At[i * astep + k], At[j * astep + k]);

                    for (k = 0; k < n; k++)
                        std::swap(Vt[i * vstep + k], Vt[j * vstep + k]);
                }
            }
        }

        for (i = 0; i < n; i++)
            _W[i] = (float) W[i];

        if (!Vt)
            return;

//        cv::RNG rng(0x12345678); /* 如果要这个函数和Opencv保持一致，这个函数需要替代，建议使用 RNG_MT19937 算法 0x12345678*/

        cg::rng_mwc_init(0x12345678);

        for (i = 0; i < n1; i++) {
            sd = i < n ? W[i] : 0;

            for (int ii = 0; ii < 100 && sd <= minval; ii++) {
                const float val0 = (float) (1. / m);
                for (k = 0; k < m; k++) {
//                     float val = (rng.next() & 256) != 0 ? val0 : -val0;
                    float val = (cg::rng_mwc_next() & 256) != 0 ? val0 : -val0;
                    At[i * astep + k] = val;
                }
                for (iter = 0; iter < 2; iter++) {
                    for (j = 0; j < i; j++) {
                        sd = 0;
                        for (k = 0; k < m; k++)
                            sd += At[i * astep + k] * At[j * astep + k];
                        float asum = 0;
                        for (k = 0; k < m; k++) {
                            float t = (float) (At[i * astep + k] - sd * At[j * astep + k]);
                            At[i * astep + k] = t;
                            asum += std::abs(t);
                        }
                        asum = asum > eps * 100 ? 1 / asum : 0;
                        for (k = 0; k < m; k++)
                            At[i * astep + k] *= asum;
                    }
                }
                sd = 0;
                for (k = 0; k < m; k++) {
                    float t = At[i * astep + k];
                    sd += (double) t * t;
                }
                sd = std::sqrt(sd);
            }

            s = (float) (sd > minval ? 1 / sd : 0.);
            for (k = 0; k < m; k++)
                At[i * astep + k] *= s;
        }
    }

    void svd_fulluv(cg::Matrix &Input, cg::Matrix &Output_w, cg::Matrix &Output_u, cg::Matrix &Output_vt) {
        int m = Input.rows(), n = Input.cols();
        cg::Matrix input_data;

        bool at = false;
        if (m < n) {
            std::swap(m, n);
            at = true;
        }
        int urows = m;

        /* 准备缓冲区, 如果做优化的话，所有的内存都要求 16 字节对齐，但是这个版本并没有考虑这个事情 */
        float *temp_a = (float *) malloc(m * m * sizeof(float));
        float *temp_w = (float *) malloc(n * sizeof(float));
        float *temp_v = (float *) malloc(n * n * sizeof(float));
        float *temp_u = temp_a;

        int step_u = m * sizeof(float);
        int step_v = n * sizeof(float);

        memset(temp_u, 0x00, m * m * sizeof(float));

        if (!at) {
            input_data = Input.transpose();
        } else {
            input_data = Input;
        }

        int index = 0;
        for (int j = 0; j < input_data.rows(); j++) {
            for (int i = 0; i < input_data.cols(); i++) {
                temp_a[index++] = input_data(j, i);
            }
        }

        jacobi_svd(temp_a, step_u, temp_w, temp_v, step_v, m, n, urows, FLT_MIN, FLT_EPSILON * 2);

        cg::Matrix W_(n, 1, temp_w);
        Output_w = W_;

        if (!at) {
            cg::Matrix vt_(n, n, temp_v);
            cg::Matrix u_(m, m, temp_u);
            Output_u = u_.transpose();
            Output_vt = vt_;
        } else {
            cg::Matrix vt_(m, m, temp_u);
            cg::Matrix u_(n, n, temp_v);
            Output_u = u_.transpose();
            Output_vt = vt_;
        }

        free(temp_a);
        free(temp_w);
        free(temp_v);
        return;
    }
}