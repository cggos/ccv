#ifndef CGOCV_CommonCV_H
#define CGOCV_CommonCV_H

#include <stdio.h>
#include <fstream>
#include <cstring>
#include <cmath>

namespace cg {

#define DELETE_NEW_OBJ(obj) if(nullptr!=obj){delete obj;obj=nullptr;}

#ifdef __USER_DEBUG__
    #if __cplusplus < 201103L
#define DEBUG_LOG(format,...) printf("File: %s, Line: %05d: "format"\n", __FILE__, __LINE__, ##__VA_ARGS__)
#else
#define DEBUG_LOG(format,...) printf("File: %s, Line: %05d: " format "\n", __FILE__, __LINE__, ##__VA_ARGS__)
#endif
#else
#define DEBUG_LOG(format,...)
#endif

    enum {
        RET_FAILED = -1,
        RET_SUCESS = 0
    };

    enum INTERPOLATION_TYPE {
        INTERPOLATION_NEAREST,
        INTERPOLATION_BILINEAR
    };

    struct Size {
        unsigned int width;
        unsigned int height;

        Size() {}

        Size(unsigned int w, unsigned int h) : width(w), height(h) {}

        Size(const Size &size) : width(size.width), height(size.height) {}

        bool operator==(const Size &rhs) const {
            return width == rhs.width && height == rhs.height;
        }

        bool operator!=(const Size &rhs) const {
            return width != rhs.width || height != rhs.height;
        }

        Size &operator=(const Size &rhs) {
            if(this == &rhs)
                return *this;
            width  = rhs.width;
            height = rhs.height;
            return *this;
        }

        Size operator+(const Size &rhs) const {
            Size size;
            size.width  = width  + rhs.width;
            size.height = height + rhs.height;
            return size;
        }

        Size operator-(const Size &rhs) const {
            Size size;
            size.width  = width  - rhs.width;
            size.height = height - rhs.height;
            return size;
        }

        Size operator*(int n) const {
            Size size;
            size.width  = width  * n;
            size.height = height * n;
            return size;
        }

        Size operator/(int n) const {
            Size size;
            size.width  = width  / n;
            size.height = height / n;
            return size;
        }

        friend std::ostream &operator<<(std::ostream &os, const Size &size){
            return os << "[" << size.width << "," << size.height << "]";
        }

        inline unsigned int area() const {
            return width * height;
        }
    };

    template <class _T>
    struct Point2D {
        _T x;
        _T y;

        Point2D() {}

        Point2D(_T x, _T y) : x(x), y(y) {}

        Point2D(const Size &size) : x(size.width), y(size.height) {}

        bool operator==(const Point2D &rhs) const {
            return x == rhs.x && y == rhs.y;
        }

        Point2D &operator=(const Size &size){
            x = _T(size.width);
            y = _T(size.height);
            return *this;
        }

        Point2D operator+(const Point2D &pt_rhs) const {
            Point2D pt;
            pt.x = x + pt_rhs.x;
            pt.y = y + pt_rhs.y;
            return pt;
        }

        Point2D operator-(const Point2D &pt_rhs) const {
            Point2D pt;
            pt.x = x - pt_rhs.x;
            pt.y = y - pt_rhs.y;
            return pt;
        }

        Point2D operator+(const Size &size_rhs) const {
            Point2D pt;
            pt.x = x + size_rhs.width;
            pt.y = y + size_rhs.height;
            return pt;
        }

        Point2D operator-(const Size &size_rhs) const {
            Point2D pt;
            pt.x = x - size_rhs.width;
            pt.y = y - size_rhs.height;
            return pt;
        }

        Point2D operator*(int n) const {
            Point2D pt;
            pt.x = x * n;
            pt.y = y * n;
            return pt;
        }

        Point2D operator/(int n) const {
            Point2D pt;
            pt.x = x / n;
            pt.y = y / n;
            return pt;
        }

        void operator/=(int n) {
            x /= n;
            y /= n;
        }

        friend std::ostream &operator<<(std::ostream &os, const Point2D &pt) {
            return os << "[" << pt.x << "," << pt.y << "]";
        }

        inline _T area() const {
            return x * y;
        }

        inline _T mag_squared() const {
            return x*x + y*y;
        }

        inline Point2D<int> pt_rounded() const {
            return Point2D<int>(
                    static_cast<int>(x > 0.0 ? x + 0.5 : x - 0.5),
                    static_cast<int>(y > 0.0 ? y + 0.5 : y - 0.5));
        }
    };

    struct RGB {
        float r;
        float g;
        float b;

        RGB() {}

        RGB(float red, float green, float blue) : r(red), g(green), b(blue) {}

        RGB &operator=(const RGB &rhs) {
            if(this == &rhs)
                return *this;
            r = rhs.r;
            g = rhs.g;
            b = rhs.b;
            return *this;
        }
    };

}
#endif //CGOCV_CommonCV_H
