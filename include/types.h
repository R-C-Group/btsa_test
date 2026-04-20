#pragma once

#include <Eigen/Dense>
#include <vector>
#include <set>
#include <memory>
#include <iostream>

typedef Eigen::Matrix<double, 2, 2> Mat2;
typedef Eigen::Matrix<double, 3, 3> Mat3;
typedef Eigen::Matrix<float, 3, 3> Mat3f;
typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<float, 3, 1> Vec3f;
typedef Eigen::Matrix<double, 4, 1> Vec4;
typedef Eigen::Matrix<float, 4, 1> Vec4f;
typedef Eigen::Matrix<double, 1, 3> Row3;
typedef Eigen::Matrix<double, 1, 4> Row4;
typedef Eigen::Matrix<double, 1, 6> Row6;
typedef Eigen::Matrix<double, 1, 2> Row2;
typedef Eigen::Matrix<double, 6, 6> Mat6;
typedef Eigen::Matrix<double, 4, 4> Mat4;
typedef Eigen::Matrix<float, 4, 4> Mat4f;
typedef Eigen::Matrix<double, 9, 9> Mat9;
typedef Eigen::Matrix<double, 12, 12> Mat12;
typedef Eigen::Matrix<double, 2, 1> Vec2;
typedef Eigen::Matrix<double, 6, 1> Vec6;
typedef Eigen::Matrix<double, 7, 1> Vec7;
typedef Eigen::Matrix<double, 8, 1> Vec8;
typedef Eigen::Matrix<double, 1, 8> Row8;
typedef Eigen::Matrix<double, 9, 1> Vec9;
typedef Eigen::Matrix<double, 1, 9> Row9;
typedef Eigen::Matrix<double, 1, 12> Row12;
typedef Eigen::Matrix<double, 12, 1> Vec12;
typedef Eigen::Matrix<double, 3, 6> Mat3_6;
typedef Eigen::Matrix<double, 3, 2> Mat3_2;
typedef Eigen::Matrix<double, 3, 4> Mat3_4;
typedef Eigen::Matrix<double, 3, 12> Mat3_12;
typedef Eigen::Matrix<double, 2, 6> Mat2_6;
typedef Eigen::Matrix<double, 9, 6> Mat9_6;
typedef Eigen::Matrix<double, 3, 9> Mat3_9;
typedef Eigen::Matrix<double, 6, 9> Mat6_9;
typedef Eigen::Matrix<double, 2, 8> Mat2_8;
typedef Eigen::Matrix<double, 2, 9> Mat2_9;
typedef Eigen::Matrix<double, 2, 3> Mat2_3;
typedef Eigen::Matrix<double, 9, 3> Mat9_3;
typedef Eigen::Matrix<double, 9, 8> Mat9_8;
typedef Eigen::Matrix<double, 12, 3> Mat12_3;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecX;
typedef Eigen::Matrix<double, 1, Eigen::Dynamic> RowX;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatX;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatXRow;
typedef Eigen::Matrix<double, 3, Eigen::Dynamic> Mat3X;
typedef Eigen::Matrix<double, 2, Eigen::Dynamic> Mat2X;

template <typename T>
struct PointTemplated
{
    T x = 0.0;
    T y = 0.0;
    T z = 0.0;
    double t = 0.0;
    float i = 0.0;
    int channel = 0;
    int type = 0;
    int scan_id = 0;
    unsigned char dynamic = 2;


    PointTemplated(const T _x, const T _y, const T _z, const double _t, const float _intensity=0.0, const int _channel=0, const int _type=0, const int _scan_id=0, const unsigned char _dynamic=2)
        : x(_x)
        , y(_y)
        , z(_z)
        , t(_t)
        , i(_intensity)
        , channel(_channel)
        , type(_type)
        , scan_id(_scan_id)
        , dynamic(_dynamic)
    {
    }
    PointTemplated(const Vec3& pt, const double _t, const double _intensity=0.0, const int _channel=0, const int _type=0, const int _scan_id=0, const unsigned char _dynamic=2)
        : x(pt[0])
        , y(pt[1])
        , z(pt[2])
        , t(_t)
        , i(_intensity)
        , channel(_channel)
        , type(_type)
        , scan_id(_scan_id)
        , dynamic(_dynamic)
    {
    }

    PointTemplated()
        : x(0.0)
        , y(0.0)
        , z(0.0)
        , t(0.0)
        , i(0.0)
    {
    }
    Eigen::Matrix<T,3,1> vec3() const
    {
        return Eigen::Matrix<T,3,1>(x, y, z);
    }
    Vec3f vec3f() const
    {
        return Vec3f((float)x, (float)y, (float)z);
    }
    Vec3 vec3d() const
    {
        return Vec3((double)x, (double)y, (double)z);
    }

    void setVec3(const Vec3& pt)
    {
        x = pt[0];
        y = pt[1];
        z = pt[2];
    }
};

typedef PointTemplated<double> Point;
typedef PointTemplated<float> Pointf;