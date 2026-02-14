#pragma once

#include "xyz_type.h"

/*!
Combined accelerometer and gyroscope values.
Gyroscope values in radians per second(RPS).
*/
struct acc_gyro_rps_t {
    xyz_t gyro_rps;
    xyz_t acc;
};


class Quaternion {
public:
    Quaternion() : w(1.0F), x(0.0F), y(0.0F), z(0.0F) {}
    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}
    Quaternion(const xyz_t& src, const xyz_t& dst);
public:
    static constexpr float M_PI_F = 3.14159265358979323846F;
    static constexpr float M_PI_2_F = M_PI_F / 2.0F;
    static constexpr float RADIANS_TO_DEGREES = 180.0F / M_PI_F;
    static constexpr float DEGREES_TO_RADIANS = M_PI_F / 180.0F;
public:
    static Quaternion from_euler_angles_radians(float rollRadians, float pitchRadians, float yawRadians);
    static Quaternion from_euler_angles_radians(float rollRadians, float pitchRadians);
    static Quaternion from_euler_angles_degrees(float rollDegrees, float pitchDegrees, float yawDegrees);
    static Quaternion from_euler_angles_degrees(float rollDegrees, float pitchDegrees);
public:
    float getW() const { return w; }
    float getX() const { return x; }
    float getY() const { return y; }
    float getZ() const { return z; }
    void getWXYZ(float& w_, float& x_, float& y_, float& z_) const { w_ = w; x_ = x; y_ = y; z_ = z; }
    void set_to_identity() { w = 1.0F; x = 0.0F; y = 0.0F; z = 0.0F; }
    void set(float w_, float x_, float y_, float z_) { w = w_; x = x_; y = y_; z = z_; }
public:
    // Equality operators
    bool operator==(const Quaternion& q) const { return w == q.w && x == q.x &&  y == q.y && z == q.z; }
    bool operator!=(const Quaternion& q) const { return w != q.w || x != q.x ||  y != q.y || z != q.z; }

    // Unary operations
    Quaternion operator+() const { return *this; } //<! Unary plus
    Quaternion operator-() const { return Quaternion(-w, -x, -y, -z); } //<! Unary negation
    Quaternion conjugate() const { return Quaternion(w, -x, -y, -z); } //<! Conjugate

    Quaternion operator+=(const Quaternion& q) { w += q.w; x += q.x; y += q.y; z += q.z; return *this; } //<! Addition
    Quaternion operator-=(const Quaternion& q) { w -= q.w; x -= q.x; y -= q.y; z -= q.z; return *this; } //<! Subtraction
    Quaternion operator*=(float k) { w*=k; x*=k; y*=k; z*=k; return *this; } //<! Multiplication by a scalar
    friend Quaternion operator*(float k, const Quaternion& q) { return q*k; } //<! Pre-multiplication by a scalar
    Quaternion operator/=(float k) { const float r = 1.0F/k; w*=r; x*=r; y*=r; z*=r; return *this; } //<! Division by a scalar
    Quaternion operator*=(const Quaternion& q) {
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
    Quaternion operator+(const Quaternion& q) const { return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z); } //<! Addition
    Quaternion operator-(const Quaternion& q) const { return Quaternion(w - q.w, x - q.x, y - q.y, z - q.z); } //<! Subtraction
    Quaternion operator*(float k) const { return Quaternion(w*k, x*k, y*k, z*k); } //<! Multiplication by a scalar
    Quaternion operator/(float k) const { const float r = 1.0F/k; return *this*r; } //<! Division by a scalar
    Quaternion operator*(const Quaternion& q) const {
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
    Quaternion rotate_x(float theta) {
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
    Quaternion rotate_y(float theta) {
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
    Quaternion rotate_z(float theta) {
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
    float magnitude_squared() const { return w*w + x*x + y*y +z*z; } //<! The square of the magnitude
    float magnitude() const { return sqrtf(magnitude_squared()); } //<! The magnitude
    float squared_norm() const { return magnitude_squared(); } //<! The square of the magnitude (using Eigen library naming)
    float norm() const { return magnitude(); } //<! The  magnitude (using Eigen library naming)
    Quaternion normalized() const; //<! Return the normalized quaternion
    Quaternion normalize() { *this=normalized(); return *this; } //<! Normalize, in-place (using Eigen library naming)
    Quaternion normalize_in_place() { *this=normalized(); return *this; } //<! Normalize, in-place

    xyz_t imaginary() const { return xyz_t{x, y, z}; } //<! The imaginary part of the quaternion
    xyz_t direction_cosine_matrix_z() const { return xyz_t{2.0F*(w*y + x*z), 2.0F*(y*z - w*x), w*w - x*x - y*y + z*z }; } //!< Last column of the equivalent rotation matrix, but calculated more efficiently than a full conversion

    // Euler angle calculations. Note that these are computationally expensive.
    float calculate_roll_radians() const;
    float calculate_pitch_radians() const;
    float calculate_yaw_radians() const;

    float calculate_roll_degrees() const  { return RADIANS_TO_DEGREES * calculate_roll_radians(); }
    float calculate_pitch_degrees() const { return RADIANS_TO_DEGREES * calculate_pitch_radians(); }
    float calculate_yaw_degrees() const   { return RADIANS_TO_DEGREES * calculate_yaw_radians(); }

    // Functions to calculate the sin, cos, and tan of the Euler angles.
    // Sometimes this can avoid the computationally expensive calculation of the angles themselves.
    float sin_roll() const;
    float sin_roll_clipped() const;
    float cos_roll() const;
    float tan_roll() const { return (w*x + y*z)/(0.5F - x*x - y*y); }
    float sin_pitch() const { return 2.0F*(w*y - x*z); }
    //! clip sin(pitchAngle) to +/-1.0F when pitch angle outside range [-90 degrees, 90 degrees]
    float sin_pitch_clipped() const { const float d = w*w - y*y; return std::signbit(d) ? std::copysignf(1.0F, sin_pitch()) : sin_pitch(); }
    float cos_pitch() const;
    float tan_pitch() const;
    float sin_yaw() const;
    float cos_yaw() const;
    float tan_yaw() const { return (w*z + x*y)/(0.5F - y*y - z*z); }
public:
    float w;
    float x;
    float y;
    float z;
};
