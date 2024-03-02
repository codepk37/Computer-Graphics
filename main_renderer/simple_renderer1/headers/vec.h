#pragma once

// Implementations taken from PBRT v3 (https://github.com/mmp/pbrt-v3/blob/master/src/core/geometry.h)

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <utility>
#include <set>


template <typename T>
inline bool isNaN(const T x)
{
    return std::isnan(x);
}
template <>
inline bool isNaN(const int x)
{
    return false;
}

template <typename T>
class Vector2
{
public:
    // Vector2 Public Methods
    Vector2() { x = y = 0; }
    Vector2(T xx, T yy) : x(xx), y(yy) {}
    bool HasNaNs() const { return isNaN(x) || isNaN(y); }

    Vector2<T> operator+(const Vector2<T> &v) const //v1+v2 =>v3
    {
        return Vector2(x + v.x, y + v.y);
    }

    Vector2<T> &operator+=(const Vector2<T> &v) //v1+=v2
    {
        x += v.x;
        y += v.y;
        return *this;
    }
    Vector2<T> operator-(const Vector2<T> &v) const
    {
        return Vector2(x - v.x, y - v.y);
    }

    Vector2<T> &operator-=(const Vector2<T> &v)
    {
        x -= v.x;
        y -= v.y;
        return *this;
    }
    bool operator==(const Vector2<T> &v) const { return x == v.x && y == v.y; }
    bool operator!=(const Vector2<T> &v) const { return x != v.x || y != v.y; }
    template <typename U>
    Vector2<T> operator*(U d) const   //v1*k =>v3
    {
        return Vector2<T>(d * x, d * y);
    }

    template <typename U>
    Vector2<T> &operator*=(U d) //v1*=k
    {
        x *= d;
        y *= d;
        return *this;
    }
    template <typename U>
    Vector2<T> operator/(U d) const  //v1/k =>v2
    {
        double inv = (double)1 / d;
        return Vector2<T>(x * inv, y * inv);
    }

    template <typename U>
    Vector2<T> &operator/=(U d)  //v1/=k
    {
        double inv = (double)1 / d;
        x *= inv;
        y *= inv;
        return *this;
    }
    Vector2<T> operator-() const { return Vector2<T>(-x, -y); }
    T operator[](int i) const
    {
        if (i == 0)
            return x;
        return y;
    }

    T &operator[](int i)
    {
        if (i == 0)
            return x;
        return y;
    }
    double LengthSquared() const { return x * x + y * y; }
    double Length() const { return std::sqrt(LengthSquared()); }

    // Vector2 Public Data
    T x, y;
};

template <typename T>
class Vector3
{
public:
    // Vector3 Public Methods
    T operator[](int i) const
    {
        if (i == 0)
            return x;
        if (i == 1)
            return y;
        return z;
    }
    T &operator[](int i)   //v[0],v[1],v[2]
    {
        if (i == 0)
            return x;
        if (i == 1)
            return y;
        return z;
    }
    Vector3() { x = y = z = 0; }
    Vector3(T x, T y, T z) : x(x), y(y), z(z) {}
    bool HasNaNs() const { return isNaN(x) || isNaN(y) || isNaN(z); }

    Vector3<T> operator+(const Vector3<T> &v) const
    {
        return Vector3(x + v.x, y + v.y, z + v.z);
    }
    Vector3<T> &operator+=(const Vector3<T> &v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }
    Vector3<T> operator-(const Vector3<T> &v) const
    {
        return Vector3(x - v.x, y - v.y, z - v.z);
    }
    Vector3<T> &operator-=(const Vector3<T> &v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }
    bool operator==(const Vector3<T> &v) const
    {
        return x == v.x && y == v.y && z == v.z;
    }
    bool operator!=(const Vector3<T> &v) const
    {
        return x != v.x || y != v.y || z != v.z;
    }
    template <typename U>
    Vector3<T> operator*(U s) const
    {
        return Vector3<T>(s * x, s * y, s * z);
    }
    template <typename U>
    Vector3<T> &operator*=(U s)
    {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }
    template <typename U>
    Vector3<T> operator/(U d) const
    {
        double inv = (double)1 / d;
        return Vector3<T>(x * inv, y * inv, z * inv);
    }

    template <typename U>
    Vector3<T> &operator/=(U d)
    {
        double inv = (double)1 / d;
        x *= inv;
        y *= inv;
        z *= inv;
        return *this;
    }
    Vector3<T> operator-() const { return Vector3<T>(-x, -y, -z); }
    double LengthSquared() const { return x * x + y * y + z * z; }
    double Length() const { return std::sqrt(LengthSquared()); }

    // Vector3 Public Data
    T x, y, z;
};

typedef Vector2<float> Vector2f;
typedef Vector2<int> Vector2i;
typedef Vector2<double> Vector2d;
typedef Vector3<float> Vector3f;
typedef Vector3<int> Vector3i;
typedef Vector3<double> Vector3d;

template <typename T, typename U>
inline Vector3<T> operator*(U s, const Vector3<T> &v)
{
    return v * s;
}
template <typename T>
Vector3<T> Abs(const Vector3<T> &v)    //abs(v): abs(v.x),abs(v.y),abs(v.z)
{
    return Vector3<T>(std::abs(v.x), std::abs(v.y), std::abs(v.z));
}

template <typename T>
inline T Dot(const Vector3<T> &v1, const Vector3<T> &v2)  //Dot(v1,v2)=> scaler :v1.x * v2.x + v1.y * v2.y + v1.z * v2.z
{
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;  
}

template <typename T>
inline T AbsDot(const Vector3<T> &v1, const Vector3<T> &v2)
{
    return std::abs(Dot(v1, v2));    //AbsDot(v1,v2) :abs(Dot(v1, v2))=>scaler
}

template <typename T>
inline Vector3<T> Cross(const Vector3<T> &v1, const Vector3<T> &v2) //Cross(v1,v2)=> v3
{
    // double v1x = v1.x, v1y = v1.y, v1z = v1.z;
    // double v2x = v2.x, v2y = v2.y, v2z = v2.z;
    T v1x = v1.x, v1y = v1.y, v1z = v1.z; // double
    T v2x = v2.x, v2y = v2.y, v2z = v2.z; // double
    return Vector3<T>((v1y * v2z) - (v1z * v2y), (v1z * v2x) - (v1x * v2z),
                      (v1x * v2y) - (v1y * v2x));
}

template <typename T>
inline Vector3<T> Normalize(const Vector3<T> &v)  //Normalize(v1):v / v.Length() => unit vector
{
    return v / v.Length();
}

template <typename T>
inline double Dot(const Vector2<T> &v1, const Vector2<T> &v2)
{
    return v1.x * v2.x + v1.y * v2.y;
}

template <typename T>
inline double AbsDot(const Vector2<T> &v1, const Vector2<T> &v2)
{
    return std::abs(Dot(v1, v2));
}

template <typename T>
inline Vector2<T> Normalize(const Vector2<T> &v)
{
    return v / v.Length();
}
template <typename T>
Vector2<T> Abs(const Vector2<T> &v)
{
    return Vector2<T>(std::abs(v.x), std::abs(v.y));
}



template <typename T>
inline T min(T a, T b) 
{
    if(a<b){return a;}
    else{return b;}
}


template <typename T>
inline T max(T a, T b) 
{
    if(a>b){return a;}
    else{return b;}
}



template <typename T>
inline Vector3<T> fmin(const Vector3<T> v1, const Vector3<T> v2) 
{
    return Vector3<T>( min(v1.x,v2.x),min(v1.y,v2.y),min(v1.z,v2.z));

}

template <typename T>
inline Vector3<T> fmax(const Vector3<T> v1, const Vector3<T> v2) 
{
    return Vector3<T>( max(v1.x,v2.x),max(v1.y,v2.y),max(v1.z,v2.z));

}


template <typename T>
void swap(T& a, T& b) {
    T temp = std::move(a);
    a = std::move(b);
    b = std::move(temp);
}


//////////////////////////////////





template <typename T>
class Vector {
public:
    Vector(T val) : value(val) {}

    Vector operator+(const Vector& v) const {
        return Vector(value + v.value);
    }

    Vector& operator+=(const Vector& v) {
        value += v.value;
        return *this;
    }

    Vector operator-(const Vector& v) const {
        return Vector(value - v.value);
    }

    Vector& operator-=(const Vector& v) {
        value -= v.value;
        return *this;
    }

    bool operator==(const Vector& v) const {
        return value == v.value;
    }

    bool operator!=(const Vector& v) const {
        return value != v.value;
    }

    Vector operator*(T s) const {
        return Vector(s * value);
    }

    Vector& operator*=(T s) {
        value *= s;
        return *this;
    }

    Vector operator/(T d) const {
        // Add checks for division by zero if needed
        return Vector(value / d);
    }

    Vector& operator/=(T d) {
        // Add checks for division by zero if needed
        value /= d;
        return *this;
    }

    T getValue() const {
        return value;
    }

private:
    T value;
};



// Specialization for double
// template <>
// class Vector<double> {
// public:
//     Vector(double val) : value(val) {}

//     double getValue() const {
//         return value;
//     }

// private:
//     double value;
// };