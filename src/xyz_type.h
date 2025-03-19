#pragma once

struct xyz_t {
public:
    // Equality operators
    inline bool operator==(const xyz_t& v) const { return x == v.x &&  y == v.y && z == v.z; }
    inline bool operator!=(const xyz_t& v) const { return x != v.x ||  y != v.y || z != v.z; }

    // Unary operators
    inline xyz_t operator+() const { return *this; } //<! Unary plus
    inline xyz_t operator-() const { return xyz_t{-x, -y, -z }; } //<! Unary negation

    inline xyz_t operator+=(const xyz_t& v) { x += v.x; y += v.y; z += v.z; return *this; }
    inline xyz_t operator-=(const xyz_t& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    inline xyz_t operator*=(float k) { x*=k; y*=k; z*=k; return *this; } //<! Multiplication by a scalar
    inline xyz_t operator/=(float k) { const float r = 1.0F/k; x*=r; y*=r; z*=r; return *this; } //<! Division by a scalar

    // Binary operators
    inline xyz_t operator+(const xyz_t& v) const { return xyz_t{x + v.x, y + v.y, z + v.z}; }
    inline xyz_t operator-(const xyz_t& v) const { return xyz_t{x - v.x, y - v.y, z - v.z}; }
    inline xyz_t operator*(float k) const { return xyz_t{x*k, y*k, z*k}; } //<! Multiplication by a scalar
    inline friend xyz_t operator*(float k, const xyz_t& v) { return v*k; } //<! Pre-multiplication by a scalar
    inline xyz_t operator/(float k) const { const float r = 1.0F/k; return *this*r; } //<! Division by a scalar
    inline float dot_product(const xyz_t& v) const { return  x*v.x + y*v.y + z*v.z; } //!< Vector dot product
    inline xyz_t cross_product(const xyz_t& v) const { return xyz_t{y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x}; } //!< Vector cross product

    // Other functions
    inline float magnitudeSquared() const { return x*x + y*y + z*z; } //<! The square of the magnitude
    inline float magnitude() const { return sqrtf(x*x + y*y + z*z); } //<! The  magnitude
    static inline float clip(float value, float min, float max) { return value < min ? min : value > max ? max : value; } //<! Clip helper function
    static inline xyz_t clip(const xyz_t& v, float min, float max) { return xyz_t{clip(v.x, min, max), clip(v.y, min, max), clip(v.z, min, max)}; } //<! Return clipped value
    inline xyz_t clip(float min, float max) { x = clip(x, min, max); y = clip(y, min, max); z = clip(z, min, max); return *this; } //<! Clip in place
public:
    float x;
    float y;
    float z;
};
