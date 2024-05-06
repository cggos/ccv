#include "ccv/maths/mathtools.h"

namespace cg {

int MatrixInverse3(float M[3][3], float MInv[3][3]) {
  float det = M[0][0] * (M[1][1] * M[2][2] - M[2][1] * M[1][2]) - M[0][1] * (M[1][0] * M[2][2] - M[2][0] * M[1][2]) +
              M[0][2] * (M[1][0] * M[2][1] - M[2][0] * M[1][1]);

  if (det < 1e-10) return -1;

  float detInv = 1.f / det;

  MInv[0][0] = (M[1][1] * M[2][2] - M[2][1] * M[1][2]) * detInv;
  MInv[1][0] = -(M[1][0] * M[2][2] - M[2][0] * M[1][2]) * detInv;
  MInv[2][0] = (M[1][0] * M[2][1] - M[2][0] * M[1][1]) * detInv;

  MInv[0][1] = -(M[0][1] * M[2][2] - M[2][1] * M[0][2]) * detInv;
  MInv[1][1] = (M[0][0] * M[2][2] - M[2][0] * M[0][2]) * detInv;
  MInv[2][1] = -(M[0][0] * M[2][1] - M[2][0] * M[0][1]) * detInv;

  MInv[0][2] = (M[0][1] * M[1][2] - M[1][1] * M[0][2]) * detInv;
  MInv[1][2] = -(M[0][0] * M[1][2] - M[1][0] * M[0][2]) * detInv;
  MInv[2][2] = (M[0][0] * M[1][1] - M[1][0] * M[0][1]) * detInv;

  return 0;
}

void MatrixMultiply(float A[3][3], float B[3][3], float C[3][3]) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      C[i][j] = 0;
      for (int k = 0; k < 3; ++k) {
        C[i][j] += A[i][k] * B[k][j];
      }
    }
  }
}

}  // namespace cg
