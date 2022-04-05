//
// Created by cg on 9/16/19.
//

#ifndef VIKIT_CG_TYPES_H
#define VIKIT_CG_TYPES_H

#include <iostream>

namespace cg {

    struct Size {
        unsigned int width;
        unsigned int height;

        Size() {width = height = 0;}

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


    template <typename _T>
    struct Point2D {
        _T x;
        _T y;
  
        Point2D() : x(0), y(0) {}
        
        Point2D(_T x, _T y) : x(x), y(y) {}
        
        Point2D(const Point2D &p) : x(p.x), y(p.y) {}
 
        bool operator==(const Point2D &rhs) const { return  x == rhs.x && y == rhs.y; }
        
        bool operator!=(const Point2D &rhs) const { return x != rhs.x || y != rhs.y; }

        Point2D &operator=(const Point2D &rhs) {
            if(this == &rhs)
                return *this;
            x = rhs.x;
            y = rhs.y;
            return *this;
        }
 
        Point2D operator+(const Point2D &rhs) const { return Point2D(x+rhs.x, y+rhs.y); }

        Point2D operator-(const Point2D &rhs) const { return Point2D(x-rhs.x, y-rhs.y); }

        Point2D operator*(int n)   const { return Point2D(x * n, y * n); }

        Point2D operator/(int n)   const { return Point2D(x / n, y / n); }

        Point2D operator*(float n) const { return Point2D(x * n, y * n); }

        Point2D operator/(float n) const { return Point2D(x / n, y / n); }

        void operator+=(const Point2D &rhs)   { this->x += rhs.x; this->y += rhs.y; }

        void operator-=(const Point2D &rhs)   { this->x -= rhs.x; this->y -= rhs.y; }

        void operator*=(int n)   { this->x *= n; this->y *= n; }

        void operator*=(float n) { this->x *= n; this->y *= n; }

        void operator/=(float n) { this->x /= n; this->y /= n; }

        Point2D operator-() { return Point2D(-x, -y); }

        _T dot(const Point2D &pt) const { return this->x * pt.x + this->y * pt.y; }

        friend std::ostream &operator<<(std::ostream &os, const Point2D &p) {
            return os << "[" << p.x << ", " << p.y << "]";
        }
    };

    typedef Point2D<int>    Point2i;
    typedef Point2D<float>  Point2f;
    typedef Point2D<double> Point2d;

    Point2f operator*(int n, const Point2f &pt);

    template <typename _T>
    struct Point3D {
        _T x;
        _T y;
        _T z;

        Point3D() : x(0), y(0), z(0) {}

        Point3D(_T x, _T y, _T z) : x(x), y(y), z(z) {}

        Point3D(const Point3D &p) : x(p.x), y(p.y), z(p.z) {}

        bool operator==(const Point3D &rhs) const { return  x == rhs.x && y == rhs.y && z == rhs.z; }

        bool operator!=(const Point3D &rhs) const { return x != rhs.x || y != rhs.y || z != rhs.z; }

        Point3D &operator=(const Point3D &rhs) {
            if(this == &rhs)
                return *this;
            x = rhs.x;
            y = rhs.y;
            z = rhs.z;
            return *this;
        }

        Point3D operator+(const Point3D &rhs) const { return Point3D(x+rhs.x, y+rhs.y, z+rhs.z); }

        Point3D operator-(const Point3D &rhs) const { return Point3D(x-rhs.x, y-rhs.y, z-rhs.z); }

        Point3D operator*(int n)   const { return Point3D(x * n, y * n, z * n); }

        Point3D operator/(int n)   const { return Point3D(x / n, y / n, z / n); }

        Point3D operator*(float n) const { return Point3D(x * n, y * n, z * n); }

        Point3D operator/(float n) const { return Point3D(x / n, y / n, z / n); }

        void operator*=(int n)   { this->x *= n; this->y *= n; this->z *= n; }

        void operator*=(float n) { this->x *= n; this->y *= n; this->z *= n; }

        Point3D operator-() { return Point3D(-x, -y, -z); }

        _T dot(const Point3D &pt) const { return this->x * pt.x + this->y * pt.y + this->z * pt.z; }

        friend std::ostream &operator<<(std::ostream &os, const Point3D &p) {
            return os << "[" << p.x << ", " << p.y << ", " << p.z << "]";
        }
    };

    typedef Point3D<int>    Point3i;
    typedef Point3D<float>  Point3f;
    typedef Point3D<double> Point3d;
}

#endif //VIKIT_CG_TYPES_H
