
#include "good_features_to_track.h"

#include <opencv2/imgproc/imgproc.hpp>

#define CV_CPU_HAS_SUPPORT_SSE2 true
#include "opencv2/core/hal/intrin.hpp"
#include "tic_toc.h"

#define TODO 0

#if TODO

/****************************************************************************************\
                                         Box Filter
\****************************************************************************************/

template<typename T, typename ST>
struct RowSum : public BaseRowFilter
{
    RowSum( int _ksize, int _anchor ) :
        BaseRowFilter()
    {
        ksize = _ksize;
        anchor = _anchor;
    }

    virtual void operator()(const uchar* src, uchar* dst, int width, int cn)
    {
        const T* S = (const T*)src;
        ST* D = (ST*)dst;
        int i = 0, k, ksz_cn = ksize*cn;

        width = (width - 1)*cn;
        if( ksize == 3 )
        {
            for( i = 0; i < width + cn; i++ )
            {
                D[i] = (ST)S[i] + (ST)S[i+cn] + (ST)S[i+cn*2];
            }
        }
        else if( ksize == 5 )
        {
            for( i = 0; i < width + cn; i++ )
            {
                D[i] = (ST)S[i] + (ST)S[i+cn] + (ST)S[i+cn*2] + (ST)S[i + cn*3] + (ST)S[i + cn*4];
            }
        }
        else if( cn == 1 )
        {
            ST s = 0;
            for( i = 0; i < ksz_cn; i++ )
                s += (ST)S[i];
            D[0] = s;
            for( i = 0; i < width; i++ )
            {
                s += (ST)S[i + ksz_cn] - (ST)S[i];
                D[i+1] = s;
            }
        }
        else if( cn == 3 )
        {
            ST s0 = 0, s1 = 0, s2 = 0;
            for( i = 0; i < ksz_cn; i += 3 )
            {
                s0 += (ST)S[i];
                s1 += (ST)S[i+1];
                s2 += (ST)S[i+2];
            }
            D[0] = s0;
            D[1] = s1;
            D[2] = s2;
            for( i = 0; i < width; i += 3 )
            {
                s0 += (ST)S[i + ksz_cn] - (ST)S[i];
                s1 += (ST)S[i + ksz_cn + 1] - (ST)S[i + 1];
                s2 += (ST)S[i + ksz_cn + 2] - (ST)S[i + 2];
                D[i+3] = s0;
                D[i+4] = s1;
                D[i+5] = s2;
            }
        }
        else if( cn == 4 )
        {
            ST s0 = 0, s1 = 0, s2 = 0, s3 = 0;
            for( i = 0; i < ksz_cn; i += 4 )
            {
                s0 += (ST)S[i];
                s1 += (ST)S[i+1];
                s2 += (ST)S[i+2];
                s3 += (ST)S[i+3];
            }
            D[0] = s0;
            D[1] = s1;
            D[2] = s2;
            D[3] = s3;
            for( i = 0; i < width; i += 4 )
            {
                s0 += (ST)S[i + ksz_cn] - (ST)S[i];
                s1 += (ST)S[i + ksz_cn + 1] - (ST)S[i + 1];
                s2 += (ST)S[i + ksz_cn + 2] - (ST)S[i + 2];
                s3 += (ST)S[i + ksz_cn + 3] - (ST)S[i + 3];
                D[i+4] = s0;
                D[i+5] = s1;
                D[i+6] = s2;
                D[i+7] = s3;
            }
        }
        else
            for( k = 0; k < cn; k++, S++, D++ )
            {
                ST s = 0;
                for( i = 0; i < ksz_cn; i += cn )
                    s += (ST)S[i];
                D[0] = s;
                for( i = 0; i < width; i += cn )
                {
                    s += (ST)S[i + ksz_cn] - (ST)S[i];
                    D[i+cn] = s;
                }
            }
    }
};

template<typename ST, typename T>
struct ColumnSum :
        public BaseColumnFilter
{
    ColumnSum( int _ksize, int _anchor, double _scale ) :
        BaseColumnFilter()
    {
        ksize = _ksize;
        anchor = _anchor;
        scale = _scale;
        sumCount = 0;
    }

    virtual void reset() { sumCount = 0; }

    virtual void operator()(const uchar** src, uchar* dst, int dststep, int count, int width)
    {
        int i;
        ST* SUM;
        bool haveScale = scale != 1;
        double _scale = scale;

        if( width != (int)sum.size() )
        {
            sum.resize(width);
            sumCount = 0;
        }

        SUM = &sum[0];
        if( sumCount == 0 )
        {
            memset((void*)SUM, 0, width*sizeof(ST));

            for( ; sumCount < ksize - 1; sumCount++, src++ )
            {
                const ST* Sp = (const ST*)src[0];

                for( i = 0; i < width; i++ )
                    SUM[i] += Sp[i];
            }
        }
        else
        {
            CV_Assert( sumCount == ksize-1 );
            src += ksize-1;
        }

        for( ; count--; src++ )
        {
            const ST* Sp = (const ST*)src[0];
            const ST* Sm = (const ST*)src[1-ksize];
            T* D = (T*)dst;
            if( haveScale )
            {
                for( i = 0; i <= width - 2; i += 2 )
                {
                    ST s0 = SUM[i] + Sp[i], s1 = SUM[i+1] + Sp[i+1];
                    D[i] = saturate_cast<T>(s0*_scale);
                    D[i+1] = saturate_cast<T>(s1*_scale);
                    s0 -= Sm[i]; s1 -= Sm[i+1];
                    SUM[i] = s0; SUM[i+1] = s1;
                }

                for( ; i < width; i++ )
                {
                    ST s0 = SUM[i] + Sp[i];
                    D[i] = saturate_cast<T>(s0*_scale);
                    SUM[i] = s0 - Sm[i];
                }
            }
            else
            {
                for( i = 0; i <= width - 2; i += 2 )
                {
                    ST s0 = SUM[i] + Sp[i], s1 = SUM[i+1] + Sp[i+1];
                    D[i] = saturate_cast<T>(s0);
                    D[i+1] = saturate_cast<T>(s1);
                    s0 -= Sm[i]; s1 -= Sm[i+1];
                    SUM[i] = s0; SUM[i+1] = s1;
                }

                for( ; i < width; i++ )
                {
                    ST s0 = SUM[i] + Sp[i];
                    D[i] = saturate_cast<T>(s0);
                    SUM[i] = s0 - Sm[i];
                }
            }
            dst += dststep;
        }
    }

    double scale;
    int sumCount;
    std::vector<ST> sum;
};


cv::Ptr<cv::BaseRowFilter> getRowSumFilter(int srcType, int sumType, int ksize, int anchor) {
    int sdepth = CV_MAT_DEPTH(srcType), ddepth = CV_MAT_DEPTH(sumType);
    CV_Assert(CV_MAT_CN(sumType) == CV_MAT_CN(srcType));

    if (anchor < 0)
        anchor = ksize / 2;

    if (sdepth == CV_32F && ddepth == CV_64F)
        return makePtr<RowSum<float, double> >(ksize, anchor);

    CV_Error_(CV_StsNotImplemented,
              ("Unsupported combination of source format (=%d), and buffer format (=%d)",
               srcType, sumType));

    return cv::Ptr<BaseRowFilter>();
}

cv::Ptr<cv::BaseColumnFilter> getColumnSumFilter(int sumType, int dstType, int ksize,
                                                 int anchor, double scale) {
    int sdepth = CV_MAT_DEPTH(sumType), ddepth = CV_MAT_DEPTH(dstType);
    CV_Assert(CV_MAT_CN(sumType) == CV_MAT_CN(dstType));

    if (anchor < 0)
        anchor = ksize / 2;

    if (ddepth == CV_32F && sdepth == CV_64F)
        return cv::makePtr<ColumnSum<double, float> >(ksize, anchor, scale);

    CV_Error_(CV_StsNotImplemented,
              ("Unsupported combination of sum format (=%d), and destination format (=%d)",
               sumType, dstType));

    return cv::Ptr<BaseColumnFilter>();
}

cv::Ptr<FilterEngine> createBoxFilter(int srcType, int dstType, cv::Size ksize,
                                      cv::Point anchor, bool normalize, int borderType) {
    int sdepth = CV_MAT_DEPTH(srcType);
    int cn = CV_MAT_CN(srcType), sumType = CV_64F;
    if (sdepth == CV_8U && CV_MAT_DEPTH(dstType) == CV_8U &&
        ksize.width * ksize.height <= 256)
        sumType = CV_16U;
    else if (sdepth <= CV_32S && (!normalize ||
                                  ksize.width * ksize.height <= (sdepth == CV_8U ? (1 << 23) : sdepth == CV_16U ? (1 << 15) : (1 << 16))))
        sumType = CV_32S;
    sumType = CV_MAKETYPE(sumType, cn); // CV_64F C3

    cv::Ptr<BaseRowFilter> rowFilter = getRowSumFilter(srcType, sumType, ksize.width, anchor.x);
    cv::Ptr<BaseColumnFilter> columnFilter = getColumnSumFilter(sumType,
                                                                dstType, ksize.height, anchor.y, normalize ? 1. / (ksize.width * ksize.height) : 1);

    return cv::makePtr<FilterEngine>(cv::Ptr<BaseFilter>(), rowFilter, columnFilter,
                                     srcType, dstType, sumType, borderType);
}

void boxFilter(const cv::Mat &_src, cv::Mat &_dst, int ddepth,
               cv::Size ksize, cv::Point anchor,
               bool normalize, int borderType) {
    const cv::Mat &src = _src;
    int stype = src.type(), sdepth = CV_MAT_DEPTH(stype), cn = CV_MAT_CN(stype);
    if (ddepth < 0)
        ddepth = sdepth;
    _dst.create(src.size(), CV_MAKETYPE(ddepth, cn));
    if (borderType != cv::BORDER_CONSTANT && normalize && (borderType & cv::BORDER_ISOLATED) != 0) {
        if (src.rows == 1)
            ksize.height = 1;
        if (src.cols == 1)
            ksize.width = 1;
    }
#ifdef HAVE_TEGRA_OPTIMIZATION
    if (tegra::useTegra() && tegra::box(src, dst, ksize, anchor, normalize, borderType))
        return;
#endif

    // CV_IPP_RUN_FAST(ipp_boxfilter(src, _dst, ksize, anchor, normalize, borderType));

    cv::Point ofs;
    cv::Size wsz(src.cols, src.rows);
    if (!(borderType & cv::BORDER_ISOLATED))
        src.locateROI(wsz, ofs);
    borderType = (borderType & ~cv::BORDER_ISOLATED);

    cv::Ptr<FilterEngine> f = createBoxFilter(src.type(), _dst.type(), ksize, anchor, normalize, borderType);
    f->apply(src, _dst, wsz, ofs);
}

#endif

static void calcMinEigenVal(const cv::Mat &_cov, cv::Mat &_dst) {
    int i, j;
    cv::Size size = _cov.size();
#if CV_TRY_AVX
    bool haveAvx = CV_CPU_HAS_SUPPORT_AVX;
#endif

#if CV_SIMD128
    bool haveSimd = true;  // cv::hasSIMD128();
#endif

    if (_cov.isContinuous() && _dst.isContinuous()) {
        size.width *= size.height;
        size.height = 1;
    }

    for (i = 0; i < size.height; i++) {
        const float *cov = _cov.ptr<float>(i);
        float *dst = _dst.ptr<float>(i);
#if CV_TRY_AVX
        if (haveAvx)
            j = calcMinEigenValLine_AVX(cov, dst, size.width);
        else
#endif  // CV_TRY_AVX
            j = 0;

#if CV_SIMD128
        if (haveSimd) {
            cv::v_float32x4 half = cv::v_setall_f32(0.5f);
            for (; j <= size.width - cv::v_float32x4::nlanes; j += cv::v_float32x4::nlanes) {
                cv::v_float32x4 v_a, v_b, v_c, v_t;
                cv::v_load_deinterleave(cov + j * 3, v_a, v_b, v_c);
                v_a *= half;
                v_c *= half;
                v_t = v_a - v_c;
                v_t = cv::v_muladd(v_b, v_b, (v_t * v_t));
                cv::v_store(dst + j, (v_a + v_c) - cv::v_sqrt(v_t));
            }
        }
#endif  // CV_SIMD128

        for (; j < size.width; j++) {
            float a = cov[j * 3] * 0.5f;
            float b = cov[j * 3 + 1];
            float c = cov[j * 3 + 2] * 0.5f;
            dst[j] = (float)((a + c) - std::sqrt((a - c) * (a - c) + b * b));
        }
    }
}

void cornerEigenValsVecs(const cv::Mat &src, cv::Mat &eigenv, int block_size,
                         int aperture_size, int op_type, double,
                         int borderType) {
#ifdef HAVE_TEGRA_OPTIMIZATION
    if (tegra::useTegra() && tegra::cornerEigenValsVecs(src, eigenv, block_size, aperture_size, op_type, k, borderType))
        return;
#endif

#if CV_TRY_AVX
    bool haveAvx = CV_CPU_HAS_SUPPORT_AVX;
#endif

#if CV_SIMD128
    bool haveSimd = true;  // cv::hasSIMD128();
#endif

    int depth = src.depth();
    double scale = (double)(1 << ((aperture_size > 0 ? aperture_size : 3) - 1)) * block_size;
    if (aperture_size < 0)
        scale *= 2.0;
    if (depth == CV_8U)
        scale *= 255.0;
    scale = 1.0 / scale;

    CV_Assert(src.type() == CV_8UC1 || src.type() == CV_32FC1);

    TicToc t_sobel;
    cv::Mat Dx, Dy;
    if (aperture_size > 0) {
        Sobel(src, Dx, CV_32F, 1, 0, aperture_size, scale, 0, borderType);
        Sobel(src, Dy, CV_32F, 0, 1, aperture_size, scale, 0, borderType);
    } else {
        Scharr(src, Dx, CV_32F, 1, 0, scale, 0, borderType);
        Scharr(src, Dy, CV_32F, 0, 1, scale, 0, borderType);
    }
    printf("Sobel: %f ms\n", t_sobel.toc());

    cv::Size size = src.size();
    cv::Mat cov(size, CV_32FC3);
    int i, j;

    TicToc t_for;
    for (i = 0; i < size.height; i++) {
        float *cov_data = cov.ptr<float>(i);
        const float *dxdata = Dx.ptr<float>(i);
        const float *dydata = Dy.ptr<float>(i);

#if CV_TRY_AVX
        if (haveAvx)
            j = cornerEigenValsVecsLine_AVX(dxdata, dydata, cov_data, size.width);
        else
#endif  // CV_TRY_AVX
            j = 0;

#if CV_SIMD128
        if (haveSimd) {
            for (; j <= size.width - cv::v_float32x4::nlanes; j += cv::v_float32x4::nlanes) {
                cv::v_float32x4 v_dx = cv::v_load(dxdata + j);
                cv::v_float32x4 v_dy = cv::v_load(dydata + j);

                cv::v_float32x4 v_dst0, v_dst1, v_dst2;
                v_dst0 = v_dx * v_dx;
                v_dst1 = v_dx * v_dy;
                v_dst2 = v_dy * v_dy;

                cv::v_store_interleave(cov_data + j * 3, v_dst0, v_dst1, v_dst2);
            }
        }
#endif  // CV_SIMD128

        for (; j < size.width; j++) {
            float dx = dxdata[j];
            float dy = dydata[j];

            cov_data[j * 3] = dx * dx;
            cov_data[j * 3 + 1] = dx * dy;
            cov_data[j * 3 + 2] = dy * dy;
        }
    }
    printf("cornerEigenValsVecs for: %f ms\n", t_for.toc());

    TicToc t_box;
    boxFilter(cov, cov, cov.depth(), cv::Size(block_size, block_size), cv::Point(-1, -1), false, borderType);
    printf("boxFilter: %f ms\n", t_box.toc());

    TicToc t_min_eigen;
    if (op_type == MINEIGENVAL)
        calcMinEigenVal(cov, eigenv);
    // else if( op_type == HARRIS )
    //     calcHarris( cov, eigenv, k );
    // else if( op_type == EIGENVALSVECS )
    //     calcEigenValsVecs( cov, eigenv );

    printf("calcMinEigenVal: %f ms\n", t_min_eigen.toc());
}

void cornerMinEigenVal(const cv::Mat &_src, cv::Mat &_dst, int blockSize, int ksize, int borderType) {
    // CV_INSTRUMENT_REGION()

    // CV_OCL_RUN(_src.dims() <= 2 && _dst.isUMat(),
    //            ocl_cornerMinEigenValVecs(_src, _dst, blockSize, ksize, 0.0, borderType, MINEIGENVAL))

    // #ifdef HAVE_IPP
    //     int kerSize = (ksize < 0)?3:ksize;
    //     bool isolated = (borderType & BORDER_ISOLATED) != 0;
    //     int borderTypeNI = borderType & ~BORDER_ISOLATED;
    // #endif
    //     CV_IPP_RUN(((borderTypeNI == BORDER_REPLICATE && (!_src.isSubmatrix() || isolated)) &&
    //             (kerSize == 3 || kerSize == 5) && (blockSize == 3 || blockSize == 5)) && IPP_VERSION_X100 >= 800,
    //         ipp_cornerMinEigenVal( _src, _dst, blockSize, ksize, borderType ));

    _dst.create(_src.size(), CV_32FC1);

    TicToc t_eigen;
    cornerEigenValsVecs(_src, _dst, blockSize, ksize, MINEIGENVAL, 0, borderType);
    printf("cornerEigenValsVecs: %f ms\n", t_eigen.toc());
}

void good_features_to_track(const cv::Mat &_image, std::vector<cv::Point2f> &_corners,
                            int maxCorners, double qualityLevel, double minDistance,
                            const cv::Mat &_mask, int blockSize, int gradientSize,
                            bool useHarrisDetector, double harrisK) {
    assert(qualityLevel > 0 && minDistance >= 0 && maxCorners >= 0);
    assert(_mask.empty() || (_mask.type() == CV_8UC1 && _mask.rows == _image.rows && _mask.cols == _image.cols));

    // CV_OCL_RUN(_image.dims() <= 2 && _image.isUMat(),
    //            ocl_goodFeaturesToTrack(_image, _corners, maxCorners, qualityLevel, minDistance,
    //                                 _mask, blockSize, gradientSize, useHarrisDetector, harrisK))

    const cv::Mat &image = _image;
    cv::Mat eig, tmp;
    if (image.empty()) {
        _corners.clear();
        return;
    }

    // // Disabled due to bad accuracy
    // CV_OVX_RUN(false && useHarrisDetector && _mask.empty() &&
    //            !ovx::skipSmallImages<VX_KERNEL_HARRIS_CORNERS>(image.cols, image.rows),
    //            openvx_harris(image, _corners, maxCorners, qualityLevel, minDistance, blockSize, gradientSize, harrisK))

    TicToc t_001;
    if (useHarrisDetector)
        cornerHarris(image, eig, blockSize, gradientSize, harrisK);
    else
        cornerMinEigenVal(image, eig, blockSize, gradientSize);
    printf("t_001: %f ms\n", t_001.toc());

    TicToc t_002;
    double maxVal = 0;
    minMaxLoc(eig, 0, &maxVal, 0, 0, _mask);
    threshold(eig, eig, maxVal * qualityLevel, 0, cv::THRESH_TOZERO);
    dilate(eig, tmp, cv::Mat());
    printf("t_002: %f ms\n", t_002.toc());

    cv::Size imgsize = image.size();
    std::vector<const float *> tmpCorners;

    TicToc t_003;
    // collect list of pointers to features - put them into temporary image
    const cv::Mat &mask = _mask;
    for (int y = 1; y < imgsize.height - 1; y++) {
        const float *eig_data = (const float *)eig.ptr(y);
        const float *tmp_data = (const float *)tmp.ptr(y);
        const uchar *mask_data = mask.data ? mask.ptr(y) : 0;

        for (int x = 1; x < imgsize.width - 1; x++) {
            float val = eig_data[x];
            if (val != 0 && val == tmp_data[x] && (!mask_data || mask_data[x]))
                tmpCorners.push_back(eig_data + x);
        }
    }
    printf("t_003: %f ms\n", t_003.toc());

    std::vector<cv::Point2f> corners;
    size_t i, j, total = tmpCorners.size(), ncorners = 0;

    if (total == 0) {
        _corners.clear();
        return;
    }

    std::sort(tmpCorners.begin(), tmpCorners.end(), greaterThanPtr());

    TicToc t_004;
    if (minDistance >= 1) {
        // Partition the image into larger grids
        int w = image.cols;
        int h = image.rows;

        const int cell_size = cvRound(minDistance);
        const int grid_width = (w + cell_size - 1) / cell_size;
        const int grid_height = (h + cell_size - 1) / cell_size;

        std::vector<std::vector<cv::Point2f> > grid(grid_width * grid_height);

        minDistance *= minDistance;

        for (i = 0; i < total; i++) {
            int ofs = (int)((const uchar *)tmpCorners[i] - eig.ptr());
            int y = (int)(ofs / eig.step);
            int x = (int)((ofs - y * eig.step) / sizeof(float));

            bool good = true;

            int x_cell = x / cell_size;
            int y_cell = y / cell_size;

            int x1 = x_cell - 1;
            int y1 = y_cell - 1;
            int x2 = x_cell + 1;
            int y2 = y_cell + 1;

            // boundary check
            x1 = std::max(0, x1);
            y1 = std::max(0, y1);
            x2 = std::min(grid_width - 1, x2);
            y2 = std::min(grid_height - 1, y2);

            for (int yy = y1; yy <= y2; yy++) {
                for (int xx = x1; xx <= x2; xx++) {
                    std::vector<cv::Point2f> &m = grid[yy * grid_width + xx];

                    if (m.size()) {
                        for (j = 0; j < m.size(); j++) {
                            float dx = x - m[j].x;
                            float dy = y - m[j].y;

                            if (dx * dx + dy * dy < minDistance) {
                                good = false;
                                goto break_out;
                            }
                        }
                    }
                }
            }

        break_out:

            if (good) {
                grid[y_cell * grid_width + x_cell].push_back(cv::Point2f((float)x, (float)y));

                corners.push_back(cv::Point2f((float)x, (float)y));
                ++ncorners;

                if (maxCorners > 0 && (int)ncorners == maxCorners)
                    break;
            }
        }
    } else {
        for (i = 0; i < total; i++) {
            int ofs = (int)((const uchar *)tmpCorners[i] - eig.ptr());
            int y = (int)(ofs / eig.step);
            int x = (int)((ofs - y * eig.step) / sizeof(float));

            corners.push_back(cv::Point2f((float)x, (float)y));
            ++ncorners;
            if (maxCorners > 0 && (int)ncorners == maxCorners)
                break;
        }
    }
    printf("t_004: %f ms\n", t_004.toc());

    _corners = corners;
}
