#pragma once

#include <cmath>

struct xy_t {
public:
    // Assignment
    xy_t& operator=(float k) { *this = {k, k }; return *this; }

    // Equality operators
    bool operator==(const xy_t& v) const { return x == v.x &&  y == v.y; }
    bool operator!=(const xy_t& v) const { return x != v.x ||  y != v.y; }

    // Index operators
    float operator[](size_t pos) const { return (pos == 0) ? x : y; } //<! Index operator
    float& operator[](size_t pos) { return (pos == 0) ? x : y; } //<! Index operator

    // Unary operators
    xy_t operator+() const { return *this; } //<! Unary plus
    xy_t operator-() const { return xy_t{-x, -y }; } //<! Unary negation

    xy_t operator+=(const xy_t& v) { x += v.x; y += v.y; return *this; } //<! Addition
    xy_t operator-=(const xy_t& v) { x -= v.x; y -= v.y; return *this; } //<! Subtraction
    xy_t operator*=(float k) { x*=k; y*=k; return *this; } //<! Multiplication by a scalar
    xy_t operator/=(float k) { const float r = 1.0F/k; x*=r; y*=r; return *this; } //<! Division by a scalar

    // Binary operators
    xy_t operator+(const xy_t& v) const { return xy_t{x + v.x, y + v.y}; } //<! Addition
    xy_t operator-(const xy_t& v) const { return xy_t{x - v.x, y - v.y}; } //<! Subtraction
    xy_t operator*(float k) const { return xy_t{x*k, y*k}; } //<! Multiplication by a scalar
    friend xy_t operator*(float k, const xy_t& v) { return v*k; } //<! Pre-multiplication by a scalar
    xy_t operator/(float k) const { const float r = 1.0F/k; return *this*r; } //<! Division by a scalar
    float dot(const xy_t& v) const { return  x*v.x + y*v.y; } //!< Vector dot product
    float distanceSquared(const xy_t& v) const { return (x-v.x)*(x-v.x) + (y-v.y)*(y-v.y); } //!< Distance between two points squared
    float distance(const xy_t& v) const { return sqrtf(distanceSquared(v)); } //!< Distance between two points

    // Other functions
    float magnitudeSquared() const { return x*x + y*y; } //<! The square of the magnitude
    float magnitude() const { return sqrtf(magnitudeSquared()); } //<! The  magnitude
    float squaredNorm() const { return magnitudeSquared(); } //<! The square of the magnitude (using Eigen library naming)
    float norm() const { return magnitude(); } //<! The  magnitude (using Eigen library naming)
    xy_t normalized() const; //<! Return the normalized vector
    xy_t normalize() { *this=normalized(); return *this; } //<! Normalize, in-place (using Eigen library naming)
    xy_t absolute() const { return{std::fabs(x), std::fabs(y) };  }//<! Return the vector consisting of the absolute value of all components
    xy_t absoluteInPlace() { *this=absolute(); return *this; } //<! Absolute value of all components, in-place
    static float clamp(float value, float min, float max) { return value < min ? min : value > max ? max : value; } //<! Clamp helper function
    static xy_t clamp(const xy_t& v, float min, float max) { return xy_t{clamp(v.x, min, max), clamp(v.y, min, max)}; } //<! Return clamped value
    xy_t clampInPlace(float min, float max) { x = clamp(x, min, max); y = clamp(y, min, max); return *this; } //<! Clamp, in-place

    void setZero() { x = 0.0F; y = 0.0F; }
    void setOnes() { x = 1.0F; y = 1.0F; }
    void setConstant(float value) { x = value; y = value; }
    float sum() const { return x + y; } 
    float mean() const { return sum()*0.5F; } 
    float prod() const { return x*y; } 
public:
    float x;
    float y;
};
