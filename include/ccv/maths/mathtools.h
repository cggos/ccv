#ifndef CCV_MATHTOOLS_H
#define CCV_MATHTOOLS_H

namespace cg {

int MatrixInverse3(float M[3][3], float MInv[3][3]);
void MatrixMultiply(float A[3][3], float B[3][3], float C[3][3]);

}  // namespace cg

#endif  // CCV_MATHTOOLS_H
