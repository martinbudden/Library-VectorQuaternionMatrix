#pragma once

#include <xyz_type.h>

class Quaternion {
public:
    Quaternion() : w(1.0F), x(0.0F), y(0.0F), z(0.0F) {}
    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}
public:
    static constexpr float radiansToDegrees = static_cast<float>(180.0 / M_PI);
    static constexpr float degreesToRadians = static_cast<float>(M_PI / 180.0);
public:
    static Quaternion fromEulerAnglesRadians(float rollRadians, float pitchRadians, float yawRadians);
    static Quaternion fromEulerAnglesRadians(float rollRadians, float pitchRadians);
    static Quaternion fromEulerAnglesDegrees(float rollDegrees, float pitchDegrees, float yawDegrees);
    static Quaternion fromEulerAnglesDegrees(float rollDegrees, float pitchDegrees);
public:
    inline float getW() const { return w; }
    inline float getX() const { return x; }
    inline float getY() const { return y; }
    inline float getZ() const { return z; }
    inline void getWXYZ(float& w_, float& x_, float& y_, float& z_) const { w_ = w; x_ = x; y_ = y; z_ = z; }
    inline void setToIdentity() { w = 1.0F; x = 0.0F; y = 0.0F; z = 0.0F; }
    inline void set(float w_, float x_, float y_, float z_) { w = w_; x = x_; y = y_; z = z_; }
public:
    // Equality operators
    bool operator==(const Quaternion& q) const { return w == q.w && x == q.x &&  y == q.y && z == q.z; }
    bool operator!=(const Quaternion& q) const { return w != q.w || x != q.x ||  y != q.y || z != q.z; }

    // Unary operations
    inline Quaternion operator+() const { return *this; } //<! Unary plus
    inline Quaternion operator-() const { return Quaternion(-w, -x, -y, -z); } //<! Unary negation
    inline Quaternion conjugate() const { return Quaternion(w, -x, -y, -z); } //<! Conjugate

    inline Quaternion operator+=(const Quaternion& q) { w += q.w; x += q.x; y += q.y; z += q.z; return *this; } //<! Addition
    inline Quaternion operator-=(const Quaternion& q) { w -= q.w; x -= q.x; y -= q.y; z -= q.z; return *this; } //<! Subtraction
    inline Quaternion operator*=(float k) { w*=k; x*=k; y*=k; z*=k; return *this; } //<! Multiplication by a scalar
    inline friend Quaternion operator*(float k, const Quaternion& q) { return q*k; } //<! Pre-multiplication by a scalar
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
    inline Quaternion operator+(const Quaternion& q) const { return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z); } //<! Addition
    inline Quaternion operator-(const Quaternion& q) const { return Quaternion(w - q.w, x - q.x, y - q.y, z - q.z); } //<! Subtraction
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

    /*!
    Rotate about the x-axis,
    equivalent to *= Quaternion(cos(theta/2), sin(theta/2), 0, 0)
    */
    inline Quaternion rotateX(float theta) {
        const float c = cosf(theta/2);
        const float s = sinf(theta/2);
        const float wt =  w*c - x*s;
        x              =  w*s + x*c;
        const float yt =  y*c + z*s;
        z              = -y*s + z*c;
        w = wt;
        y = yt;
        return *this;
    }
    /*!
    Rotate about the y-axis.
    equivalent to *= Quaternion(cos(theta/2), 0, sin(theta/2), 0)
    */
    inline Quaternion rotateY(float theta) {
        const float c = cosf(theta/2);
        const float s = sinf(theta/2);
        const float wt = w*c - y*s;
        const float xt = x*c - z*s;
        y              = w*s + y*c;
        z              = x*s - z*c;
        w = wt;
        x = xt;
        return *this;
    }
    /*!
    Rotate about the z-axis,
    equivalent to *= Quaternion(cos(theta/2), 0, 0, sin(theta/2))
    */
    inline Quaternion rotateZ(float theta) {
        const float c = cosf(theta/2);
        const float s = sinf(theta/2);
        const float wt = w*c - z*s;
        const float xt = x*c - y*s;
        y              = x*s + y*c;
        z              = w*s + z*c;
        w = wt;
        x = xt;
        return *this;
    }
    xyz_t rotate(const xyz_t& v) const; //<! Rotate a vector
public:
    inline float magnitudeSquared() const { return w*w + x*x + y*y +z*z; } //<! The square of the magnitude
    inline float magnitude() const { return sqrtf(magnitudeSquared()); } //<! The magnitude
    Quaternion normalize() const; //<! Return the normalized quaternion
    inline Quaternion normalizeInPlace() { *this=normalize(); return *this; } //<! Normalize, in-place

    // Euler angle calculations. Note that these are computationally expensive.
    float calculateRollRadians() const;
    float calculatePitchRadians() const;
    float calculateYawRadians() const;

    inline float calculateRollDegrees() const  { return radiansToDegrees * calculateRollRadians(); }
    inline float calculatePitchDegrees() const { return radiansToDegrees * calculatePitchRadians(); }
    inline float calculateYawDegrees() const   { return radiansToDegrees * calculateYawRadians(); }

    // Functions to calculate the sin, cos, and tan of the Euler angles.
    // Sometimes this can avoid the computationally expensive calculation of the angles themselves.
    float sinRoll() const;
    //! clip sin(rollAngle) to +/-1.0F when pitch angle outside range [-90 degrees, 90 degrees]
    float sinRollClipped() const;
    float cosRoll() const;
    inline float tanRoll() const { return (w*x + y*z)/(0.5F - x*x - y*y); }
    inline float sinPitch() const { return 2.0F*(w*y - x*z); }
    //! clip sin(pitchAngle) to +/-1.0F when pitch angle outside range [-90 degrees, 90 degrees]
    float sinPitchClipped() const { const float d = w*w - y*y; return std::signbit(d) ? std::copysignf(1.0F, sinPitch()) : sinPitch(); }
    float cosPitch() const;
    float tanPitch() const;
    float sinYaw() const;
    float cosYaw() const;
    inline float tanYaw() const { return (w*z + x*y)/(0.5F - y*y - z*z); }

public:
    // implementation functions, made public for test code
    static float arcsinClippedf(float x);
    static float arcsinRestrictedXf(float x);
    static float arcsinApproximatef(float x);
    static float arccosApproximatef(float x);
    static float arctanApproximatef(float x);
    static float arctan2Approximatef(float y, float x);
protected:
    float w;
    float x;
    float y;
    float z;
};
