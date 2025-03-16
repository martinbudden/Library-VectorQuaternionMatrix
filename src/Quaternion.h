#pragma once

#include <cmath>
#include <xyz_type.h>

class Quaternion {
public:
    Quaternion() : w(1.0F), x(0.0F), y(0.0F), z(0.0F) {}
    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}
public:
    static Quaternion fromEulerAnglesRadians(float rollRadians, float pitchRadians, float yawRadians);
    static Quaternion fromEulerAnglesRadians(float rollRadians, float pitchRadians);
public:
    static constexpr float radiansToDegrees = static_cast<float>(180.0 / M_PI);
    static constexpr float degreesToRadians = static_cast<float>(M_PI / 180.0);
public:
    inline float getW() const { return w; }
    inline float getX() const { return x; }
    inline float getY() const { return y; }
    inline float getZ() const { return z; }
    inline void getWXYZ(float& w_, float& x_, float&y_, float& z_) const { w_ = w; x_ = x; y_ = y; z_ = z; }
public:
    // Equality operators
    bool operator==(const Quaternion& q) const { return w == q.w && x == q.x &&  y == q.y && z == q.z; }
    bool operator!=(const Quaternion& q) const { return w != q.w || x != q.x ||  y != q.y || z != q.z; }

    // Unary operations
    inline Quaternion operator+() const { return *this; } //<! Unary plus
    inline Quaternion operator-() const { return Quaternion(-w, -x, -y, -z); } //<! Unary negation
    inline Quaternion conjugate() const { return Quaternion(w, -x, -y, -z); }

    inline Quaternion operator+=(const Quaternion& q) { w += q.w; x += q.x; y += q.y; z += q.z; return *this; }
    inline Quaternion operator-=(const Quaternion& q) { w -= q.w; x -= q.x; y -= q.y; z -= q.z; return *this; }
    inline Quaternion operator*=(float k) { w*=k; x*=k; y*=k; z*=k; return *this; } //<! Multiplication by a scalar
    inline Quaternion operator/=(float k) { const float r = 1.0F/k; w*=r; x*=r; y*=r; z*=r; return *this; } //<! Division by a scalar
    inline Quaternion operator*=(const Quaternion& q) {
        const float wt = w*q.w - x*q.x - y*q.y - z*q.z;
        const float xt = w*q.x + x*q.w + y*q.z - z*q.y;
        const float yt = w*q.y - x*q.z + y*q.w + z*q.x;
        z              = w*q.z + x*q.y - y*q.x + z*q.w;
        w = wt;
        x = xt;
        y = yt;
        return *this;
    }

    // Binary operations
    inline Quaternion operator+(const Quaternion& q) const { return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z); }
    inline Quaternion operator-(const Quaternion& q) const { return Quaternion(w - q.w, x - q.x, y - q.y, z - q.z); }
    inline Quaternion operator*(float k) const { return Quaternion(w*k, x*k, y*k, z*k); } //<! Multiplication by a scalar
    inline Quaternion operator/(float k) const { const float r = 1.0F/k; return *this*r; } //<! Division by a scalar
    inline Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            w*q.w - x*q.x - y*q.y - z*q.z,
            w*q.x + x*q.w + y*q.z - z*q.y,
            w*q.y - x*q.z + y*q.w + z*q.x,
            w*q.z + x*q.y - y*q.x + z*q.w
        );
    }

    inline Quaternion applyDelta(float delta) {
        // equivalent to *= Quaternion(cos(delta/2), 0, 0, sin(delta/2))
        const float c = cosf(delta/2);
        const float s = sinf(delta/2);
        const float wt = c*w - s*z;
        const float xt = c*x - s*y;
        const float yt = c*y + s*x;
        z = c*z + s*w;
        w = wt;
        x = xt;
        y = yt;
        return *this;
    }
    
    xyz_t rotate(const xyz_t& v) const;

    // Non-member operations
    inline friend Quaternion operator*(float k, const Quaternion& q) { return q*k; } //<! Pre-multiplication by a scalar
public:
    inline float magnitude_squared() const { return w*w + x*x + y*y +z*z; } //<! The square of the magnitude

    // Conversion functions
    static inline float asinfClipped(float angleRadians) {
        if (angleRadians <= -static_cast<float>(static_cast<float>(M_PI_2))) { return {-static_cast<float>(M_PI_2)}; }
        if (angleRadians >=  static_cast<float>(M_PI_2)) { return {static_cast<float>(M_PI_2)}; }
        return asinf(angleRadians);
    }
    inline float calculateRollRadians() const  { return atan2f(w*x + y*z, 0.5F - x*x - y*y); }
    inline float calculatePitchRadians() const { return asinfClipped(2.0F*(w*y - x*z)); }
    inline float calculateYawRadians() const   { return atan2f(w*z + x*y, 0.5F - y*y - z*z); } // alternatively atan2f(2*(w*z + x*y), w*w + x*x - y*y - z*z)

    inline float calculateRollDegrees() const  { return radiansToDegrees * calculateRollRadians(); }
    inline float calculatePitchDegrees() const { return radiansToDegrees * calculatePitchRadians(); }
    inline float calculateYawDegrees() const   { return radiansToDegrees * calculateYawRadians(); }
protected:
    float w;
    float x;
    float y;
    float z;
};
