#pragma once

#include <cmath>

struct xy_t {
public:
    // Assignment
    xy_t& operator=(float k) { *this = {k, k }; return *this; }

    // Equality operators
    inline bool operator==(const xy_t& v) const { return x == v.x &&  y == v.y; }
    inline bool operator!=(const xy_t& v) const { return x != v.x ||  y != v.y; }

    // Unary operators
    inline xy_t operator+() const { return *this; } //<! Unary plus
    inline xy_t operator-() const { return xy_t{-x, -y }; } //<! Unary negation

    inline xy_t operator+=(const xy_t& v) { x += v.x; y += v.y; return *this; } //<! Addition
    inline xy_t operator-=(const xy_t& v) { x -= v.x; y -= v.y; return *this; } //<! Subtraction
    inline xy_t operator*=(float k) { x*=k; y*=k; return *this; } //<! Multiplication by a scalar
    inline xy_t operator/=(float k) { const float r = 1.0F/k; x*=r; y*=r; return *this; } //<! Division by a scalar

    // Binary operators
    inline xy_t operator+(const xy_t& v) const { return xy_t{x + v.x, y + v.y}; } //<! Addition
    inline xy_t operator-(const xy_t& v) const { return xy_t{x - v.x, y - v.y}; } //<! Subtraction
    inline xy_t operator*(float k) const { return xy_t{x*k, y*k}; } //<! Multiplication by a scalar
    inline friend xy_t operator*(float k, const xy_t& v) { return v*k; } //<! Pre-multiplication by a scalar
    inline xy_t operator/(float k) const { const float r = 1.0F/k; return *this*r; } //<! Division by a scalar
    inline float dot(const xy_t& v) const { return  x*v.x + y*v.y; } //!< Vector dot product

    // Other functions
    inline float magnitudeSquared() const { return x*x + y*y; } //<! The square of the magnitude
    inline float magnitude() const { return sqrtf(magnitudeSquared()); } //<! The  magnitude
    xy_t normalize() const; //<! Return the normalized vector
    inline xy_t normalizeInPlace() { *this=normalize(); return *this; } //<! Normalize, in-place
    static inline float clip(float value, float min, float max) { return value < min ? min : value > max ? max : value; } //<! Clip helper function
    static inline xy_t clip(const xy_t& v, float min, float max) { return xy_t{clip(v.x, min, max), clip(v.y, min, max)}; } //<! Return clipped value
    inline xy_t clipInPlace(float min, float max) { x = clip(x, min, max); y = clip(y, min, max); return *this; } //<! Clip, in-place
public:
    float x;
    float y;
};
