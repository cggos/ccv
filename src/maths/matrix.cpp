#include "ccv/maths/matrix.h"

#include <math.h>

using namespace std;

namespace cg {

#define SWAP(a, b) \
  {                \
    temp = a;      \
    a = b;         \
    b = temp;      \
  }
#define SIGN(a, b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
static FLOAT sqrarg;
#define SQR(a) ((sqrarg = (a)) == 0.0 ? 0.0 : sqrarg * sqrarg)
static FLOAT maxarg1, maxarg2;
#define FMAX(a, b) (maxarg1 = (a), maxarg2 = (b), (maxarg1) > (maxarg2) ? (maxarg1) : (maxarg2))
static int32_t iminarg1, iminarg2;
#define IMIN(a, b) (iminarg1 = (a), iminarg2 = (b), (iminarg1) < (iminarg2) ? (iminarg1) : (iminarg2))

Matrix::Matrix() : m_(0), n_(0), val_(0) {}

Matrix::Matrix(const int32_t m, const int32_t n) { allocate_memory(m, n); }

Matrix::Matrix(const int32_t m, const int32_t n, const FLOAT *val) {
  allocate_memory(m, n);
  int32_t k = 0;
  for (int32_t i = 0; i < m; i++)
    for (int32_t j = 0; j < n; j++) val_[i][j] = val[k++];
}

Matrix::Matrix(const int32_t m, const int32_t n, const float *val) {
  allocate_memory(m, n);
  int32_t k = 0;
  for (int32_t i = 0; i < m; i++)
    for (int32_t j = 0; j < n; j++) val_[i][j] = val[k++];
}

Matrix::Matrix(const Matrix &M) {
  allocate_memory(M.m_, M.n_);
  for (int32_t i = 0; i < M.m_; i++) memcpy(val_[i], M.val_[i], M.n_ * sizeof(FLOAT));
}

Matrix::~Matrix() { release_memory(); }

Matrix &Matrix::operator=(const Matrix &M) {
  if (this != &M) {
    if (M.m_ != m_ || M.n_ != n_) {
      release_memory();
      allocate_memory(M.m_, M.n_);
    }
    if (M.n_ > 0)
      for (int32_t i = 0; i < M.m_; i++) memcpy(val_[i], M.val_[i], M.n_ * sizeof(FLOAT));
  }
  return *this;
}

FLOAT &Matrix::operator()(const int i, const int j) {
  if (i < 0 || i >= m_ || j < 0 || j >= n_) {
    std::cerr << "ERROR: Cannot get value from a (" << m_ << "x" << n_ << ") matrix." << endl;
    exit(0);
  }
  return val_[i][j];
}

const FLOAT &Matrix::operator()(const int i, const int j) const {
  if (i < 0 || i >= m_ || j < 0 || j >= n_) {
    std::cerr << "ERROR: Cannot get value from a (" << m_ << "x" << n_ << ") matrix." << endl;
    exit(0);
  }
  return val_[i][j];
}

const Matrix Matrix::get_mat(int32_t i1, int32_t j1, int32_t i2, int32_t j2) const {
  if (i2 == -1) i2 = m_ - 1;
  if (j2 == -1) j2 = n_ - 1;
  if (i1 < 0 || i2 >= m_ || j1 < 0 || j2 >= n_ || i2 < i1 || j2 < j1) {
    cerr << "ERROR: Cannot get submatrix [" << i1 << ".." << i2 << "] x [" << j1 << ".." << j2 << "]" << " of a (" << m_
         << "x" << n_ << ") matrix." << endl;
    exit(0);
  }
  Matrix M(i2 - i1 + 1, j2 - j1 + 1);
  for (int32_t i = 0; i < M.m_; i++)
    for (int32_t j = 0; j < M.n_; j++) M.val_[i][j] = val_[i1 + i][j1 + j];
  return M;
}

void Matrix::set_mat(const int32_t i1, const int32_t j1, const Matrix &M) {
  if (i1 < 0 || j1 < 0 || i1 + M.m_ > m_ || j1 + M.n_ > n_) {
    cerr << "ERROR: Cannot set submatrix [" << i1 << ".." << i1 + M.m_ - 1 << "] x [" << j1 << ".." << j1 + M.n_ - 1
         << "]" << " of a (" << m_ << "x" << n_ << ") matrix." << endl;
    exit(0);
  }
  for (int32_t i = 0; i < M.m_; i++)
    for (int32_t j = 0; j < M.n_; j++) val_[i1 + i][j1 + j] = M.val_[i][j];
}

void Matrix::set_diag(FLOAT s, int32_t i1, int32_t i2) {
  if (i2 == -1) i2 = min(m_ - 1, n_ - 1);
  for (int32_t i = i1; i <= i2; i++) val_[i][i] = s;
}

const Matrix Matrix::extract_cols(vector<int> idx) const {
  Matrix M(m_, idx.size());
  for (int32_t j = 0; j < M.n_; j++)
    if (idx[j] < n_)
      for (int32_t i = 0; i < m_; i++) M.val_[i][j] = val_[i][idx[j]];
  return M;
}

const Matrix Matrix::row(int i) const {
  Matrix M(1, n_);
  for (int32_t j = 0; j < M.n_; j++) M.val_[0][j] = val_[0][j];
  return M;
}

Matrix Matrix::eye(const int32_t m) {
  Matrix M(m, m);
  for (int32_t i = 0; i < m; i++) M.val_[i][i] = 1;
  return M;
}

void Matrix::eye() {
  for (int32_t i = 0; i < m_; i++)
    for (int32_t j = 0; j < n_; j++) val_[i][j] = 0;
  for (int32_t i = 0; i < min(m_, n_); i++) val_[i][i] = 1;
}

Matrix Matrix::identity(int32_t p, int32_t q) {
  Matrix M = eye(p);
  M.conservative_resize(p, q);
  return M;
}

Matrix Matrix::diag(const Matrix &M) {
  if (M.m_ > 1 && M.n_ == 1) {
    Matrix D(M.m_, M.m_);
    for (int32_t i = 0; i < M.m_; i++) D.val_[i][i] = M.val_[i][0];
    return D;
  } else if (M.m_ == 1 && M.n_ > 1) {
    Matrix D(M.n_, M.n_);
    for (int32_t i = 0; i < M.n_; i++) D.val_[i][i] = M.val_[0][i];
    return D;
  }
  cout << "ERROR: Trying to create diagonal matrix from vector of size (" << M.m_ << "x" << M.n_ << ")" << endl;
  exit(0);
}

void Matrix::conservative_resize(int32_t p, int32_t q) {
  if (0 == p || 0 == q) {
    *this = Matrix(p, q);
    return;
  }
  Matrix mat(p, q);
  if (p <= m_ && q <= n_) mat = get_mat(0, 0, p - 1, q - 1);
  if (p > m_ && q > n_) mat.set_mat(0, 0, *this);
  if (p <= m_ && q > n_) mat.set_mat(0, 0, get_mat(0, 0, p - 1, n_ - 1));
  if (p > m_ && q <= n_) mat.set_mat(0, 0, get_mat(0, 0, m_ - 1, q - 1));
  *this = mat;
}

const Matrix Matrix::operator+(const Matrix &M) const {
  const Matrix &A = *this;
  const Matrix &B = M;
  if (A.m_ != B.m_ || A.n_ != B.n_) {
    cerr << "ERROR: Trying to add matrices of size (" << A.m_ << "x" << A.n_ << ") and (" << B.m_ << "x" << B.n_ << ")"
         << endl;
    exit(0);
  }
  Matrix C(A.m_, A.n_);
  for (int32_t i = 0; i < m_; i++)
    for (int32_t j = 0; j < n_; j++) C.val_[i][j] = A.val_[i][j] + B.val_[i][j];
  return C;
}

const Matrix Matrix::operator-(const Matrix &M) const {
  const Matrix &A = *this;
  const Matrix &B = M;
  if (A.m_ != B.m_ || A.n_ != B.n_) {
    cerr << "ERROR: Trying to subtract matrices of size (" << A.m_ << "x" << A.n_ << ") and (" << B.m_ << "x" << B.n_
         << ")" << endl;
    exit(0);
  }
  Matrix C(A.m_, A.n_);
  for (int32_t i = 0; i < m_; i++)
    for (int32_t j = 0; j < n_; j++) C.val_[i][j] = A.val_[i][j] - B.val_[i][j];
  return C;
}

const Matrix Matrix::operator*(const Matrix &M) const {
  const Matrix &A = *this;
  const Matrix &B = M;
  if (A.n_ != B.m_) {
    cerr << "ERROR: Trying to multiply matrices of size (" << A.m_ << "x" << A.n_ << ") and (" << B.m_ << "x" << B.n_
         << ")" << endl;
    exit(0);
  }
  Matrix C(A.m_, B.n_);
  for (int32_t i = 0; i < A.m_; i++)
    for (int32_t j = 0; j < B.n_; j++)
      for (int32_t k = 0; k < A.n_; k++) C.val_[i][j] += A.val_[i][k] * B.val_[k][j];
  return C;
}

const Matrix Matrix::operator*(const FLOAT &s) const {
  Matrix C(m_, n_);
  for (int32_t i = 0; i < m_; i++)
    for (int32_t j = 0; j < n_; j++) C.val_[i][j] = val_[i][j] * s;
  return C;
}

const Matrix Matrix::operator/(const Matrix &M) const {
  const Matrix &A = *this;
  const Matrix &B = M;

  if (A.m_ == B.m_ && A.n_ == B.n_) {
    Matrix C(A.m_, A.n_);
    for (int32_t i = 0; i < A.m_; i++)
      for (int32_t j = 0; j < A.n_; j++)
        if (B.val_[i][j] != 0) C.val_[i][j] = A.val_[i][j] / B.val_[i][j];
    return C;

  } else if (A.m_ == B.m_ && B.n_ == 1) {
    Matrix C(A.m_, A.n_);
    for (int32_t i = 0; i < A.m_; i++)
      for (int32_t j = 0; j < A.n_; j++)
        if (B.val_[i][0] != 0) C.val_[i][j] = A.val_[i][j] / B.val_[i][0];
    return C;

  } else if (A.n_ == B.n_ && B.m_ == 1) {
    Matrix C(A.m_, A.n_);
    for (int32_t i = 0; i < A.m_; i++)
      for (int32_t j = 0; j < A.n_; j++)
        if (B.val_[0][j] != 0) C.val_[i][j] = A.val_[i][j] / B.val_[0][j];
    return C;

  } else {
    cerr << "ERROR: Trying to divide matrices of size (" << A.m_ << "x" << A.n_ << ") and (" << B.m_ << "x" << B.n_
         << ")" << endl;
    exit(0);
  }
}

const Matrix Matrix::operator/(const FLOAT &s) const {
  if (fabs(s) < 1e-20) {
    cerr << "ERROR: Trying to divide by zero!" << endl;
    exit(0);
  }
  Matrix C(m_, n_);
  for (int32_t i = 0; i < m_; i++)
    for (int32_t j = 0; j < n_; j++) C.val_[i][j] = val_[i][j] / s;
  return C;
}

const Matrix Matrix::operator-() const {
  Matrix C(m_, n_);
  for (int32_t i = 0; i < m_; i++)
    for (int32_t j = 0; j < n_; j++) C.val_[i][j] = -val_[i][j];
  return C;
}

const Matrix Matrix::transpose() const {
  Matrix C(n_, m_);
  for (int32_t i = 0; i < m_; i++)
    for (int32_t j = 0; j < n_; j++) C.val_[j][i] = val_[i][j];
  return C;
}

FLOAT Matrix::l2norm() const {
  FLOAT norm = 0;
  for (int32_t i = 0; i < m_; i++)
    for (int32_t j = 0; j < n_; j++) norm += val_[i][j] * val_[i][j];
  return sqrt(norm);
}

FLOAT Matrix::mean() {
  FLOAT mean = 0;
  for (int32_t i = 0; i < m_; i++)
    for (int32_t j = 0; j < n_; j++) mean += val_[i][j];
  return mean / (FLOAT)(m_ * n_);
}

Matrix Matrix::cross(const Matrix &a, const Matrix &b) {
  if (a.m_ != 3 || a.n_ != 1 || b.m_ != 3 || b.n_ != 1) {
    cerr << "ERROR: Cross product vectors must be of size (3x1)" << endl;
    exit(0);
  }
  Matrix c(3, 1);
  c.val_[0][0] = a.val_[1][0] * b.val_[2][0] - a.val_[2][0] * b.val_[1][0];
  c.val_[1][0] = a.val_[2][0] * b.val_[0][0] - a.val_[0][0] * b.val_[2][0];
  c.val_[2][0] = a.val_[0][0] * b.val_[1][0] - a.val_[1][0] * b.val_[0][0];
  return c;
}

const Matrix Matrix::inv(const Matrix &M) const {
  if (M.m_ != M.n_) {
    cerr << "ERROR: Trying to invert matrix of size (" << M.m_ << "x" << M.n_ << ")" << endl;
    exit(0);
  }
  Matrix A(M);
  Matrix B = eye(M.m_);
  B.solve(A);
  return B;
}

const Matrix Matrix::inv() const {
  if (m_ != n_) {
    cerr << "ERROR: Trying to invert matrix of size (" << m_ << "x" << n_ << ")" << endl;
    exit(0);
  }
  return inv(*this);
}

FLOAT Matrix::det() {
  if (m_ != n_) {
    cerr << "ERROR: Trying to compute determinant of a matrix of size (" << m_ << "x" << n_ << ")" << endl;
    exit(0);
  }

  Matrix A(*this);
  int32_t *idx = (int32_t *)malloc(m_ * sizeof(int32_t));
  FLOAT d = 1;
  A.lu(idx, d);
  for (int32_t i = 0; i < m_; i++) d *= A.val_[i][i];
  free(idx);
  return d;
}

bool Matrix::solve(const Matrix &M, FLOAT eps) {
  // substitutes
  const Matrix &A = M;
  Matrix &B = *this;

  if (A.m_ != A.n_ || A.m_ != B.m_ || A.m_ < 1 || B.n_ < 1) {
    cerr << "ERROR: Trying to eliminate matrices of size (" << A.m_ << "x" << A.n_ << ") and (" << B.m_ << "x" << B.n_
         << ")" << endl;
    exit(0);
  }

  // index vectors for bookkeeping on the pivoting
  int32_t *indxc = new int32_t[m_];
  int32_t *indxr = new int32_t[m_];
  int32_t *ipiv = new int32_t[m_];

  // loop variables
  int32_t i, icol, irow, j, k, l, ll;
  FLOAT big, dum, pivinv, temp;

  // initialize pivots to zero
  for (j = 0; j < m_; j++) ipiv[j] = 0;

  // main loop over the columns to be reduced
  for (i = 0; i < m_; i++) {
    big = 0.0;

    // search for a pivot element
    for (j = 0; j < m_; j++)
      if (ipiv[j] != 1)
        for (k = 0; k < m_; k++)
          if (ipiv[k] == 0)
            if (fabs(A.val_[j][k]) >= big) {
              big = fabs(A.val_[j][k]);
              irow = j;
              icol = k;
            }
    ++(ipiv[icol]);

    // We now have the pivot element, so we interchange rows, if needed, to put the pivot
    // element on the diagonal. The columns are not physically interchanged, only relabeled.
    if (irow != icol) {
      for (l = 0; l < m_; l++) SWAP(A.val_[irow][l], A.val_[icol][l])
      for (l = 0; l < n_; l++) SWAP(B.val_[irow][l], B.val_[icol][l])
    }

    indxr[i] = irow;  // We are now ready to divide the pivot row by the
    indxc[i] = icol;  // pivot element, located at irow and icol.

    // check for singularity
    if (fabs(A.val_[icol][icol]) < eps) {
      delete[] indxc;
      delete[] indxr;
      delete[] ipiv;
      return false;
    }

    pivinv = 1.0 / A.val_[icol][icol];
    A.val_[icol][icol] = 1.0;
    for (l = 0; l < m_; l++) A.val_[icol][l] *= pivinv;
    for (l = 0; l < n_; l++) B.val_[icol][l] *= pivinv;

    // Next, we reduce the rows except for the pivot one
    for (ll = 0; ll < m_; ll++)
      if (ll != icol) {
        dum = A.val_[ll][icol];
        A.val_[ll][icol] = 0.0;
        for (l = 0; l < m_; l++) A.val_[ll][l] -= A.val_[icol][l] * dum;
        for (l = 0; l < n_; l++) B.val_[ll][l] -= B.val_[icol][l] * dum;
      }
  }

  // This is the end of the main loop over columns of the reduction. It only remains to unscramble
  // the solution in view of the column interchanges. We do this by interchanging pairs of
  // columns in the reverse order that the permutation was built up.
  for (l = m_ - 1; l >= 0; l--) {
    if (indxr[l] != indxc[l])
      for (k = 0; k < m_; k++) SWAP(A.val_[k][indxr[l]], A.val_[k][indxc[l]])
  }

  // success
  delete[] indxc;
  delete[] indxr;
  delete[] ipiv;
  return true;
}

// Given a matrix a[1..n][1..n], this routine replaces it by the LU decomposition of a rowwise
// permutation of itself. a and n are input. a is output, arranged as in equation (2.3.14) above;
// indx[1..n] is an output vector that records the row permutation effected by the partial
// pivoting; d is output as ±1 depending on whether the number of row interchanges was even
// or odd, respectively. This routine is used in combination with lubksb to solve linear equations
// or invert a matrix.
bool Matrix::lu(int32_t *idx, FLOAT &d, FLOAT eps) {
  if (m_ != n_) {
    cerr << "ERROR: Trying to LU decompose a matrix of size (" << m_ << "x" << n_ << ")" << endl;
    exit(0);
  }

  int32_t i, imax, j, k;
  FLOAT big, dum, sum, temp;
  FLOAT *vv = (FLOAT *)malloc(n_ * sizeof(FLOAT));  // vv stores the implicit scaling of each row.
  d = 1.0;
  for (i = 0; i < n_; i++) {  // Loop over rows to get the implicit scaling information.
    big = 0.0;
    for (j = 0; j < n_; j++)
      if ((temp = fabs(val_[i][j])) > big) big = temp;
    if (big == 0.0) {  // No nonzero largest element.
      free(vv);
      return false;
    }
    vv[i] = 1.0 / big;  // Save the scaling.
  }
  for (j = 0; j < n_; j++) {   // This is the loop over columns of Crout’s method.
    for (i = 0; i < j; i++) {  // This is equation (2.3.12) except for i = j.
      sum = val_[i][j];
      for (k = 0; k < i; k++) sum -= val_[i][k] * val_[k][j];
      val_[i][j] = sum;
    }
    big = 0.0;  // Initialize the search for largest pivot element.
    for (i = j; i < n_; i++) {
      sum = val_[i][j];
      for (k = 0; k < j; k++) sum -= val_[i][k] * val_[k][j];
      val_[i][j] = sum;
      if ((dum = vv[i] * fabs(sum)) >= big) {
        big = dum;
        imax = i;
      }
    }
    if (j != imax) {              // Do we need to interchange rows?
      for (k = 0; k < n_; k++) {  // Yes, do so...
        dum = val_[imax][k];
        val_[imax][k] = val_[j][k];
        val_[j][k] = dum;
      }
      d = -d;            // ...and change the parity of d.
      vv[imax] = vv[j];  // Also interchange the scale factor.
    }
    idx[j] = imax;
    if (j != n_ - 1) {  // Now, finally, divide by the pivot element.
      dum = 1.0 / val_[j][j];
      for (i = j + 1; i < n_; i++) val_[i][j] *= dum;
    }
  }  // Go back for the next column in the reduction.

  // success
  free(vv);
  return true;
}

// Given a matrix M/A[1..m][1..n], this routine computes its singular value decomposition, M/A =
// U·W·V T. Thematrix U replaces a on output. The diagonal matrix of singular values W is output
// as a vector w[1..n]. Thematrix V (not the transpose V T ) is output as v[1..n][1..n].
void Matrix::svd(Matrix &U2, Matrix &W, Matrix &V) {
  Matrix U = Matrix(*this);
  U2 = Matrix(m_, m_);
  V = Matrix(n_, n_);

  FLOAT *w = (FLOAT *)malloc(n_ * sizeof(FLOAT));
  FLOAT *rv1 = (FLOAT *)malloc(n_ * sizeof(FLOAT));

  int32_t flag, i, its, j, jj, k, l, nm;
  FLOAT anorm, c, f, g, h, s, scale, x, y, z;

  g = scale = anorm = 0.0;  // Householder reduction to bidiagonal form.
  for (i = 0; i < n_; i++) {
    l = i + 1;
    rv1[i] = scale * g;
    g = s = scale = 0.0;
    if (i < m_) {
      for (k = i; k < m_; k++) scale += fabs(U.val_[k][i]);
      if (scale) {
        for (k = i; k < m_; k++) {
          U.val_[k][i] /= scale;
          s += U.val_[k][i] * U.val_[k][i];
        }
        f = U.val_[i][i];
        g = -SIGN(sqrt(s), f);
        h = f * g - s;
        U.val_[i][i] = f - g;
        for (j = l; j < n_; j++) {
          for (s = 0.0, k = i; k < m_; k++) s += U.val_[k][i] * U.val_[k][j];
          f = s / h;
          for (k = i; k < m_; k++) U.val_[k][j] += f * U.val_[k][i];
        }
        for (k = i; k < m_; k++) U.val_[k][i] *= scale;
      }
    }
    w[i] = scale * g;
    g = s = scale = 0.0;
    if (i < m_ && i != n_ - 1) {
      for (k = l; k < n_; k++) scale += fabs(U.val_[i][k]);
      if (scale) {
        for (k = l; k < n_; k++) {
          U.val_[i][k] /= scale;
          s += U.val_[i][k] * U.val_[i][k];
        }
        f = U.val_[i][l];
        g = -SIGN(sqrt(s), f);
        h = f * g - s;
        U.val_[i][l] = f - g;
        for (k = l; k < n_; k++) rv1[k] = U.val_[i][k] / h;
        for (j = l; j < m_; j++) {
          for (s = 0.0, k = l; k < n_; k++) s += U.val_[j][k] * U.val_[i][k];
          for (k = l; k < n_; k++) U.val_[j][k] += s * rv1[k];
        }
        for (k = l; k < n_; k++) U.val_[i][k] *= scale;
      }
    }
    anorm = FMAX(anorm, (fabs(w[i]) + fabs(rv1[i])));
  }
  for (i = n_ - 1; i >= 0; i--) {  // Accumulation of right-hand transformations.
    if (i < n_ - 1) {
      if (g) {
        for (j = l; j < n_; j++)  // Double division to avoid possible underflow.
          V.val_[j][i] = (U.val_[i][j] / U.val_[i][l]) / g;
        for (j = l; j < n_; j++) {
          for (s = 0.0, k = l; k < n_; k++) s += U.val_[i][k] * V.val_[k][j];
          for (k = l; k < n_; k++) V.val_[k][j] += s * V.val_[k][i];
        }
      }
      for (j = l; j < n_; j++) V.val_[i][j] = V.val_[j][i] = 0.0;
    }
    V.val_[i][i] = 1.0;
    g = rv1[i];
    l = i;
  }
  for (i = IMIN(m_, n_) - 1; i >= 0; i--) {  // Accumulation of left-hand transformations.
    l = i + 1;
    g = w[i];
    for (j = l; j < n_; j++) U.val_[i][j] = 0.0;
    if (g) {
      g = 1.0 / g;
      for (j = l; j < n_; j++) {
        for (s = 0.0, k = l; k < m_; k++) s += U.val_[k][i] * U.val_[k][j];
        f = (s / U.val_[i][i]) * g;
        for (k = i; k < m_; k++) U.val_[k][j] += f * U.val_[k][i];
      }
      for (j = i; j < m_; j++) U.val_[j][i] *= g;
    } else
      for (j = i; j < m_; j++) U.val_[j][i] = 0.0;
    ++U.val_[i][i];
  }
  for (k = n_ - 1; k >= 0; k--) {     // Diagonalization of the bidiagonal form: Loop over singular values,
    for (its = 0; its < 30; its++) {  // and over allowed iterations.
      flag = 1;
      for (l = k; l >= 0; l--) {  // Test for splitting.
        nm = l - 1;
        if ((FLOAT)(fabs(rv1[l]) + anorm) == anorm) {
          flag = 0;
          break;
        }
        if ((FLOAT)(fabs(w[nm]) + anorm) == anorm) {
          break;
        }
      }
      if (flag) {
        c = 0.0;  // Cancellation of rv1[l], if l > 1.
        s = 1.0;
        for (i = l; i <= k; i++) {
          f = s * rv1[i];
          rv1[i] = c * rv1[i];
          if ((FLOAT)(fabs(f) + anorm) == anorm) break;
          g = w[i];
          h = pythag(f, g);
          w[i] = h;
          h = 1.0 / h;
          c = g * h;
          s = -f * h;
          for (j = 0; j < m_; j++) {
            y = U.val_[j][nm];
            z = U.val_[j][i];
            U.val_[j][nm] = y * c + z * s;
            U.val_[j][i] = z * c - y * s;
          }
        }
      }
      z = w[k];
      if (l == k) {     // Convergence.
        if (z < 0.0) {  // Singular value is made nonnegative.
          w[k] = -z;
          for (j = 0; j < n_; j++) V.val_[j][k] = -V.val_[j][k];
        }
        break;
      }
      if (its == 29) cerr << "ERROR in SVD: No convergence in 30 iterations" << endl;
      x = w[l];  // Shift from bottom 2-by-2 minor.
      nm = k - 1;
      y = w[nm];
      g = rv1[nm];
      h = rv1[k];
      f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
      g = pythag(f, 1.0);
      f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;
      c = s = 1.0;  // Next QR transformation:
      for (j = l; j <= nm; j++) {
        i = j + 1;
        g = rv1[i];
        y = w[i];
        h = s * g;
        g = c * g;
        z = pythag(f, h);
        rv1[j] = z;
        c = f / z;
        s = h / z;
        f = x * c + g * s;
        g = g * c - x * s;
        h = y * s;
        y *= c;
        for (jj = 0; jj < n_; jj++) {
          x = V.val_[jj][j];
          z = V.val_[jj][i];
          V.val_[jj][j] = x * c + z * s;
          V.val_[jj][i] = z * c - x * s;
        }
        z = pythag(f, h);
        w[j] = z;  // Rotation can be arbitrary if z = 0.
        if (z) {
          z = 1.0 / z;
          c = f * z;
          s = h * z;
        }
        f = c * g + s * y;
        x = c * y - s * g;
        for (jj = 0; jj < m_; jj++) {
          y = U.val_[jj][j];
          z = U.val_[jj][i];
          U.val_[jj][j] = y * c + z * s;
          U.val_[jj][i] = z * c - y * s;
        }
      }
      rv1[l] = 0.0;
      rv1[k] = f;
      w[k] = x;
    }
  }

  // sort singular values and corresponding columns of u and v
  // by decreasing magnitude. Also, signs of corresponding columns are
  // flipped so as to maximize the number of positive elements.
  int32_t s2, inc = 1;
  FLOAT sw;
  FLOAT *su = (FLOAT *)malloc(m_ * sizeof(FLOAT));
  FLOAT *sv = (FLOAT *)malloc(n_ * sizeof(FLOAT));
  do {
    inc *= 3;
    inc++;
  } while (inc <= n_);
  do {
    inc /= 3;
    for (i = inc; i < n_; i++) {
      sw = w[i];
      for (k = 0; k < m_; k++) su[k] = U.val_[k][i];
      for (k = 0; k < n_; k++) sv[k] = V.val_[k][i];
      j = i;
      while (w[j - inc] < sw) {
        w[j] = w[j - inc];
        for (k = 0; k < m_; k++) U.val_[k][j] = U.val_[k][j - inc];
        for (k = 0; k < n_; k++) V.val_[k][j] = V.val_[k][j - inc];
        j -= inc;
        if (j < inc) break;
      }
      w[j] = sw;
      for (k = 0; k < m_; k++) U.val_[k][j] = su[k];
      for (k = 0; k < n_; k++) V.val_[k][j] = sv[k];
    }
  } while (inc > 1);
  for (k = 0; k < n_; k++) {  // flip signs
    s2 = 0;
    for (i = 0; i < m_; i++)
      if (U.val_[i][k] < 0.0) s2++;
    for (j = 0; j < n_; j++)
      if (V.val_[j][k] < 0.0) s2++;
    if (s2 > (m_ + n_) / 2) {
      for (i = 0; i < m_; i++) U.val_[i][k] = -U.val_[i][k];
      for (j = 0; j < n_; j++) V.val_[j][k] = -V.val_[j][k];
    }
  }

  // create vector and copy singular values
  W = Matrix(min(m_, n_), 1, w);

  // extract mxm submatrix U
  U2.set_mat(0, 0, U.get_mat(0, 0, m_ - 1, min(m_ - 1, n_ - 1)));

  // release temporary memory
  free(w);
  free(rv1);
  free(su);
  free(sv);
}

void Matrix::ldlt01(cg::Matrix &L, cg::Matrix &D) {
  if (this->cols() != this->rows()) return;

  Matrix A = *this;
  int n = A.rows();

  FLOAT v[n];
  FLOAT d[n];

  L = Matrix(n, n);
  D = Matrix(n, n);

  for (int i = 0; i < n; i++) {
    if (i > 0) {
      FLOAT tmp = 0.0;
      for (int j = 0; j < i - 1; j++) {
        v[j] = L(i, j) * d[j];
        tmp += L(i, j) * v[j];
      }
      d[i] = v[i] = A(i, i) - tmp;
      if (i < n - 1) {
        for (int j = i + 1; j < n; j++) {
          tmp = 0.0;
          for (int k = 0; k < i - 1; k++) tmp += L(j, k) * v[k];
          L(j, i) = (A(j, i) - tmp) / v[i];
        }
      }
    } else {
      d[0] = v[0] = A(0, 0);
      for (int j = 1; j < n; j++) L(j, 0) = A(j, 0) / v[0];
    }
  }

  for (int i = 0; i < n; i++) {
    D(i, i) = d[i];
  }
  L = L + Matrix::eye(n);
}

void Matrix::ldlt(cg::Matrix &L, cg::Matrix &D) const {
  if (this->cols() != this->rows()) return;

  Matrix A = *this;
  int size = A.rows();

  double r[size];

  L = Matrix(size, size);
  D = Matrix(size, size);

  for (int k = 0; k < size; k++) {
    for (int p = 0; p <= k - 1; p++) {
      r[p] = A(p, p) * A(k, p);
    }
    for (int p = 0; p <= k - 1; p++) {
      A(k, k) -= A(k, p) * r[p];
    }
    for (int i = k + 1; i < size; i++) {
      double sum = 0.0;
      for (int p = 0; p < k; p++) {
        sum += A(i, p) * r[p];
      }
      A(i, k) = (A(i, k) - sum) / A(k, k);
    }
  }

  for (int i = 0; i < size; i++) {
    for (int j = 0; j < i; j++) {
      L(i, j) = A(i, j);
    }
  }

  for (int i = 0; i < size; i++) {
    L(i, i) = 1;
    D(i, i) = A(i, i);
  }
}

ostream &operator<<(ostream &out, const Matrix &M) {
  if (M.m_ == 0 || M.n_ == 0) {
    out << "[empty matrix]";
  } else {
    char buffer[1024];
    for (int32_t i = 0; i < M.m_; i++) {
      for (int32_t j = 0; j < M.n_; j++) {
        sprintf(buffer, "%12.7f ", M.val_[i][j]);
        out << buffer;
      }
      if (i < M.m_ - 1) out << endl;
    }
  }
  return out;
}

void Matrix::allocate_memory(const int32_t m, const int32_t n) {
  m_ = abs(m);
  n_ = abs(n);
  if (m_ == 0 || n_ == 0) {
    val_ = 0;
    return;
  }
  //        val = (FLOAT **) malloc(m * sizeof(FLOAT *));
  //        val[0] = (FLOAT *) calloc(m * n, sizeof(FLOAT));
  //        for (int32_t i = 1; i < m; i++)
  //            val[i] = val[i - 1] + n;

  val_ = new FLOAT *[m_];
  val_[0] = new FLOAT[m_ * n_]();
  for (int32_t i = 1; i < m_; i++) val_[i] = val_[i - 1] + n_;
}

void Matrix::release_memory() {
  if (val_ != nullptr) {
    //            free(val[0]);
    //            free(val);

    delete[] val_[0];
    delete[] val_;
  }
}

FLOAT Matrix::pythag(FLOAT a, FLOAT b) {
  FLOAT absa, absb;
  absa = fabs(a);
  absb = fabs(b);
  if (absa > absb)
    return absa * sqrt(1.0 + SQR(absb / absa));
  else
    return (absb == 0.0 ? 0.0 : absb * sqrt(1.0 + SQR(absa / absb)));
}

}  // namespace cg