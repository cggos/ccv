//
// Created by gordon on 17-12-19.
//

#ifndef CGOCV_IMAGE_H
#define CGOCV_IMAGE_H

#include "cvkit/cv/common_cv.h"
#include "cvkit/cv/camera.h"

namespace cg {

    template <class _T>
    class Image {
    public:
        Image() : size_(0,0), data_(nullptr) {}

        Image(unsigned int w, unsigned int h) {
            size_.width = w;
            size_.height = h;
            data_ = new _T[area()];
        }

        Image(const Size &size) {
            size_ = size;
            data_ = new _T[area()];
        }

        Image(const Image &img){
            size_ = img.size_;
            data_ = new _T[area()];
            memcpy(data_, img.data_, area()*sizeof(_T));
        }

        Image &operator=(const Image &rhs) {
            if (this == &rhs)
                return *this;

            if(data_!= nullptr){
                delete []data_;
                data_ = nullptr;
            }

            size_ = rhs.size_;
            data_ = new _T[area()];
            memcpy(data_, rhs.data_, area()*sizeof(_T));

            return *this;
        }

        _T &operator()(const int row, const int col) const {
            return data_[row * size_.width + col];
        }

        _T &operator[](const Point2D<int> &pt) const {
            return data_[pt.y * size_.width + pt.x];
        }

        inline _T *data() const {
            return data_;
        }

        inline _T *end() const {
            return &data_[size_.height * size_.width - 1] + 1;
        }

        inline bool empty() const {
            return data_ == nullptr;
        }

        inline Size size() const {
            return size_;
        }

        inline unsigned int area() const {
            return size_.area();
        }

        inline bool is_in_image(const Point2D<int> &pt, int border) const {
            return pt.x >= border && pt.x <= (int) size_.width - border &&
                   pt.y >= border && pt.y <= (int) size_.height - border;
        }

        bool copy(Image &img_dst){
            if(img_dst.empty())
                return false;
            if(img_dst.size() != size())
                return false;
            memcpy(img_dst.data(), data(), area()*sizeof(_T));
            return true;
        }

        bool copy(Image &img_dst, const Size &size, const Point2D<int> &pt){
            if(img_dst.size() == Size(0,0))
                return false;
            if(size.width > img_dst.size().width || size.height > img_dst.size().height)
                return false;
            for(unsigned int h=pt.y; h<pt.y+size.height; ++h){
                for(unsigned int w=pt.x; w<pt.x+size.width; ++w){
                    img_dst(h-pt.y, w-pt.x) = data_[h*size_.width + w];
                }
            }
            return true;
        }

        bool copy(Image &img_dst, unsigned int width, unsigned int height, const Point2D<int> &pt){
            if(img_dst.size() == Size(0,0))
                return false;
            if(width > img_dst.size().width || height > img_dst.size().height)
                return false;
            for(unsigned int h=pt.y; h<pt.y+height; ++h){
                for(unsigned int w=pt.x; w<pt.x+width; ++w){
                    img_dst(h-pt.y, w-pt.x) = data_[h*size_.width + w];
                }
            }
            return true;
        }

        void half_sample(Image<unsigned char> &img_dst) {//Mean Pyramid
            img_dst = Image<unsigned char>(size_ / 2);

            const unsigned char *top = data_;
            const unsigned char *bottom = top + size_.width;
            const unsigned char *end = top + size_.width * size_.height;

            int ow = img_dst.size_.width;
            int skip = size_.width + (size_.width % 2);
            unsigned char *p = img_dst.data_;
            while (bottom < end) {
                for (int j = 0; j < ow; j++) {
                    *p = static_cast<unsigned char>((top[0] + top[1] + bottom[0] + bottom[1]) * 0.25f + 0.5f);
                    p++;
                    top += 2;
                    bottom += 2;
                }
                top += skip;
                bottom += skip;
            }
        }

        void undistort_image(Image &img_dst, unsigned int width, unsigned int height,
                const CameraModel &model_cam, const DistortModel &model_dis ) {

            float cx = model_cam.cx;
            float cy = model_cam.cy;
            float fx = model_cam.fx;
            float fy = model_cam.fy;
            float k1 = model_dis.k1;
            float k2 = model_dis.k2;
            float p1 = model_dis.p1;
            float p2 = model_dis.p2;

            for (int v = 0; v < height; v++) {
                for (int u = 0; u < width; u++) {

                    double u_distorted = 0, v_distorted = 0;

                    double x = (u-cx)/fx;
                    double y = (v-cy)/fy;

                    double x2 = x*x, y2 = y*y, xy = x*y, r2 = x2 + y2;
                    double x_radial = x * (1 + k1*r2 + k2*r2*r2);
                    double y_radial = y * (1 + k1*r2 + k2*r2*r2);
                    double x_tangential = 2*p1*xy + p2*(r2 + 2*x2);
                    double y_tangential = 2*p2*xy + p1*(r2 + 2*y2);
                    double xd = x_radial + x_tangential;
                    double yd = y_radial + y_tangential;

                    u_distorted = xd*fx + cx;
                    v_distorted = yd*fy + cy;

                    // 赋值 (最近邻插值)
                    if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < width && v_distorted < height) {
                        img_dst(v, u) = (*this)((int) v_distorted, (int) u_distorted);
                    } else {
                        img_dst(v, u) = 0;
                    }
                }
            }
        }

        /**
         * @brief
         * @param x
         * @param y
         * @param result
         * @param interpolation_type
         * @details
         * (1) INTERPOLATION_NEAREST:  \n
         * (2) INTERPOLATION_BILINEAR:  \n
         * V方向线性插值：
         * \f[
         * \begin{align}
         *   f(i,j+v)   &= [f(i,j+1)-f(i,j)]v + f(i,j) \\
         *   f(i+1,j+v) &= [f(i+1,j+1)-f(i+1,j)]v + f(i+1,j)
         * \end{align}
         * \f]
         * V方向线性插值：
         * \f[
         *   f(i+u,j+v) = [f(i+1,j+v)-f(i,j+v)]u+f(i,j+v)
         * \f]
         * 最终：
         * \f[
         *   f(i+u,j+v) = [(1-u)(1-v)]f(i,j) + (1-u)vf(i,j+1) + u(1-v)f(i+1,j) + uvf(i+1,j+1)
         * \f]
         */
        inline void sample_interpolation(double x, double y, _T &result, INTERPOLATION_TYPE interpolation_type = INTERPOLATION_BILINEAR) {
            switch (interpolation_type) {

                case INTERPOLATION_NEAREST: {
                    int lx = static_cast<int>(std::round(x));
                    int ly = static_cast<int>(std::round(y));
                    result = (*this)(ly, lx);
                }
                    break;

                case INTERPOLATION_BILINEAR: {
                    const int lx = std::floor(x);
                    const int ly = std::floor(y);

                    _T f00 = (*this)(ly, lx);
                    _T f01 = (*this)(ly + 1, lx);
                    _T f10 = (*this)(ly, lx + 1);
                    _T f11 = (*this)(ly + 1, lx + 1);

                    double alpha = x - lx;
                    double beta  = y - ly;

                    result = (1 - beta) * ((1 - alpha) * f00 + alpha * f10) + beta * ((1 - alpha) * f01 + alpha * f11);
                }
                    break;
            }

        }

        int zoom_with_interpolation(Image &img_dst, float scaleX=1.0f, float scaleY=1.f,
                                    INTERPOLATION_TYPE interpolation_type = INTERPOLATION_BILINEAR){
            if(scaleX < 0 || scaleY < 0)
                return -1;

            int dst_w = (int)std::ceil(size_.width  * scaleX);
            int dst_h = (int)std::ceil(size_.height * scaleY);

            img_dst = Image<unsigned char>(dst_w, dst_h);

            double xf = 0.0, yf = 0.0;
            for(int j=0; j<dst_h; ++j) {
                for (int i = 0; i < dst_w; ++i) {
                    _T &data = img_dst(j, i);
                    xf = i / scaleX;
                    yf = j / scaleY;
                    sample_interpolation(xf, yf, data, interpolation_type);
                }
            }
            return 0;
        }

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
        inline static double *generate_gaussian_template(unsigned int m, double sigma = 0.84089642) {
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

        static void gaussian_blur(Image &img_dst, unsigned int m, double sigma) {
            double *gaussian_template = generate_gaussian_template(m, sigma);
            Size size = img_dst.size();
            Image imgTemp(size);
            img_dst.copy(imgTemp);
            unsigned int k = m / 2;
            for (unsigned int h = k; h < size.height - k; ++h) {
                for (unsigned int w = k; w < size.width - k; ++w) {
                    _T sum = 0;
                    for (unsigned int y = 0; y < m; ++y) {
                        for (unsigned int x = 0; x < m; ++x) {
                            sum += img_dst(h - k + y, w - k + x) * *(gaussian_template + y * m + x);
                        }
                    }
                    img_dst(h, w) = sum;
                }
            }
        }

        virtual ~Image() {
            if (data_ != nullptr) {
                delete[] data_;
                data_ = nullptr;
            }
        }

    private:
        Size size_;
        _T *data_;
    };

    int  MatrixRotateImage(int cx, int cy, float degree, float M[3][3]);
    void ImgProjectiveTransform(const unsigned char  *imgSrc, unsigned int  widthSrc, unsigned int  heightSrc,
                       float H[3][3], unsigned char **imgDst, unsigned int *widthDst, unsigned int *heightDst);
    void ImgProjectiveTransform_Fixed(const unsigned char  *imgSrc, unsigned int widthSrc, unsigned int heightSrc,
                         float H[3][3],     unsigned char **imgDst, unsigned int widthDst, unsigned int heightDst);
}

#endif //CGOCV_IMAGE_H
