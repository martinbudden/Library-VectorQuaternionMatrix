#pragma once

#include <cmath>

struct xyz_t {
public:
    // Assignment
    xyz_t& operator=(float k) { *this = {k, k, k}; return *this; }

    // Equality operators
    inline bool operator==(const xyz_t& v) const { return x == v.x &&  y == v.y && z == v.z; }
    inline bool operator!=(const xyz_t& v) const { return x != v.x ||  y != v.y || z != v.z; }

    // Unary operators
    inline xyz_t operator+() const { return *this; } //<! Unary plus
    inline xyz_t operator-() const { return xyz_t{-x, -y, -z }; } //<! Unary negation

    inline xyz_t operator+=(const xyz_t& v) { x += v.x; y += v.y; z += v.z; return *this; } //<! Addition
    inline xyz_t operator-=(const xyz_t& v) { x -= v.x; y -= v.y; z -= v.z; return *this; } //<! Subtraction
    inline xyz_t operator*=(float k) { x*=k; y*=k; z*=k; return *this; } //<! Multiplication by a scalar
    inline xyz_t operator/=(float k) { const float r = 1.0F/k; x*=r; y*=r; z*=r; return *this; } //<! Division by a scalar

    // Binary operators
    inline xyz_t operator+(const xyz_t& v) const { return xyz_t{x + v.x, y + v.y, z + v.z}; } //<! Addition
    inline xyz_t operator-(const xyz_t& v) const { return xyz_t{x - v.x, y - v.y, z - v.z}; } //<! Subtraction
    inline xyz_t operator*(float k) const { return xyz_t{x*k, y*k, z*k}; } //<! Multiplication by a scalar
    inline friend xyz_t operator*(float k, const xyz_t& v) { return v*k; } //<! Pre-multiplication by a scalar
    inline xyz_t operator/(float k) const { const float r = 1.0F/k; return *this*r; } //<! Division by a scalar
    inline float dot(const xyz_t& v) const { return  x*v.x + y*v.y + z*v.z; } //!< Vector dot product
    xyz_t cross(const xyz_t& v) const { return xyz_t{y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x}; } //!< Vector cross product

    // Other functions
    inline float magnitudeSquared() const { return x*x + y*y + z*z; } //<! The square of the magnitude
    float magnitude() const { return sqrtf(magnitudeSquared()); } //<! The  magnitude
    xyz_t normalize() const; //<! Return the normalized vector
    inline xyz_t normalizeInPlace() { *this=normalize(); return *this; } //<! Normalize, in-place
    inline xyz_t absolute() const { return{std::fabs(x), std::fabs(y), std::fabs(z) };  }//<! Return the vector consisting of the absolute value of all components
    inline xyz_t absoluteInPlace() { *this=absolute(); return *this; } //<! Absolute value of all components, in-place
    static inline float clamp(float value, float min, float max) { return value < min ? min : value > max ? max : value; } //<! clamp helper function
    static inline xyz_t clamp(const xyz_t& v, float min, float max) { return xyz_t{clamp(v.x, min, max), clamp(v.y, min, max), clamp(v.z, min, max)}; } //<! Return clampped value
    inline xyz_t clampInPlace(float min, float max) { x = clamp(x, min, max); y = clamp(y, min, max); z = clamp(z, min, max); return *this; } //<! clamp, in-place
public:
    float x;
    float y;
    float z;
};
