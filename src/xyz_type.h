#pragma once

#include <cmath>

struct xyz_t {
public:
    // Assignment
    xyz_t& operator=(float k) { *this = {k, k, k}; return *this; }

    // Equality operators
    bool operator==(const xyz_t& v) const { return x == v.x &&  y == v.y && z == v.z; }
    bool operator!=(const xyz_t& v) const { return x != v.x ||  y != v.y || z != v.z; }

    // Index operators
    float operator[](size_t pos) const { return (pos == 0) ? x : (pos == 1) ? y : z; } //<! Index operator
    float& operator[](size_t pos) { return (pos == 0) ? x : (pos == 1) ? y : z; } //<! Index operator

    // Unary operators
    xyz_t operator+() const { return *this; } //<! Unary plus
    xyz_t operator-() const { return xyz_t{-x, -y, -z }; } //<! Unary negation

    xyz_t operator+=(const xyz_t& v) { x += v.x; y += v.y; z += v.z; return *this; } //<! Addition
    xyz_t operator-=(const xyz_t& v) { x -= v.x; y -= v.y; z -= v.z; return *this; } //<! Subtraction
    xyz_t operator*=(float k) { x*=k; y*=k; z*=k; return *this; } //<! Multiplication by a scalar
    xyz_t operator/=(float k) { const float r = 1.0F/k; x*=r; y*=r; z*=r; return *this; } //<! Division by a scalar

    // Binary operators
    xyz_t operator+(const xyz_t& v) const { return xyz_t{x + v.x, y + v.y, z + v.z}; } //<! Addition
    xyz_t operator-(const xyz_t& v) const { return xyz_t{x - v.x, y - v.y, z - v.z}; } //<! Subtraction
    xyz_t operator*(float k) const { return xyz_t{x*k, y*k, z*k}; } //<! Multiplication by a scalar
    friend xyz_t operator*(float k, const xyz_t& v) { return v*k; } //<! Pre-multiplication by a scalar
    xyz_t operator/(float k) const { const float r = 1.0F/k; return *this*r; } //<! Division by a scalar
    float dot(const xyz_t& v) const { return  x*v.x + y*v.y + z*v.z; } //!< Vector dot product
    xyz_t cross(const xyz_t& v) const { return xyz_t{y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x}; } //!< Vector cross product
    float distance_squared(const xyz_t& v) const { return (x-v.x)*(x-v.x) + (y-v.y)*(y-v.y) + (z-v.z)*(z-v.z); } //!< Distance between two points squared
    float distance(const xyz_t& v) const { return sqrtf(distance_squared(v)); } //!< Distance between two points

    // Other functions
    float magnitude_squared() const { return x*x + y*y + z*z; } //<! The square of the magnitude
    float magnitude() const { return sqrtf(magnitude_squared()); } //<! The  magnitude
    float squared_norm() const { return magnitude_squared(); } //<! The square of the magnitude (using Eigen library naming)
    float norm() const { return magnitude(); } //<! The  magnitude (using Eigen library naming)
    xyz_t normalized() const; //<! Return the normalized vector
    xyz_t normalize() { *this=normalized(); return *this; } //<! Normalize, in-place
    xyz_t absolute() const { return{std::fabs(x), std::fabs(y), std::fabs(z) };  }//<! Return the vector consisting of the absolute value of all components
    xyz_t absolute_in_place() { *this=absolute(); return *this; } //<! Absolute value of all components, in-place
    static float clamp(float value, float min, float max) { return value < min ? min : value > max ? max : value; } //<! clamp helper function
    static xyz_t clamp(const xyz_t& v, float min, float max) { return xyz_t{clamp(v.x, min, max), clamp(v.y, min, max), clamp(v.z, min, max)}; } //<! Return clampped value
    xyz_t clamp_in_place(float min, float max) { x = clamp(x, min, max); y = clamp(y, min, max); z = clamp(z, min, max); return *this; } //<! clamp, in-place

    void set_zero() { x = 0.0F; y = 0.0F; z = 0.0F; }
    void set_ones() { x = 1.0F; y = 1.0F; z = 1.0F; }
    void set_constant(float value) { x = value; y = value; z = value; }
    float sum() const { return x + y + z; }
    float mean() const { return sum()*(1.0F/3.0F); }
    float prod() const { return x*y*z; }
public:
    float x;
    float y;
    float z;
};
