//
// Created by cg on 11/20/19.
//

#ifndef CCV_MATH_SVD_FULLUV_H
#define CCV_MATH_SVD_FULLUV_H

#include "matrix.h"

namespace cg {

void svd_fulluv(cg::Matrix& Input, cg::Matrix& Output_w, cg::Matrix& Output_u, cg::Matrix& Output_vt);

}

#endif  // CCV_MATH_SVD_FULLUV_H
