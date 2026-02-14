#include "fast_trigonometry.h"
#include "matrix3x3.h"
#include "quaternion.h"
#include "xy_type.h"

#include <cstdint>

/*!
Reciprocal square root
Implementation of [fast inverse square root](http://en.wikipedia.org/wiki/Fast_inverse_square_root)
using [Pizer’s optimisation](https://pizer.wordpress.com/2008/10/12/fast-inverse-square-root/) and
using `union` rather than `reinterpret_cast` to avoid breaking strict-aliasing rules.

The Xtensa floating point coprocessor (used on the ESP32) has some hardware support for reciprocal square root: it has
an RSQRT0.S (single-precision reciprocal square root initial step) instruction.
However benchmarking shows that FAST_RECIPROCAL_SQUARE_ROOT is approximately 3.5 times faster than `1.0F / sqrtf()`
*/
inline float reciprocal_sqrtf(float x)
{
#if defined(LIBRARY_VECTOR_QUATERNION_MATRIX_USE_FAST_RECIPROCAL_SQUARE_ROOT) || defined(LIBRARY_VECTOR_QUATERNION_MATRIX_USE_FAST_RECIPROCAL_SQUARE_ROOT_TWO_ITERATIONS)
    union {
        float f;
        int32_t i;
    } u { .f = x };

// NOLINTBEGIN(cppcoreguidelines-pro-type-union-access)
    u.i = 0x5f1f1412 - (u.i >> 1); // Initial estimate for Newton–Raphson method
    // single iteration gives accuracy to 4.5 significant figures
    u.f *= 1.69000231F - 0.714158168F * x * u.f * u.f; // First iteration
#if defined(LIBRARY_VECTOR_QUATERNION_MATRIX_USE_FAST_RECIPROCAL_SQUARE_ROOT_TWO_ITERATIONS)
    // two iterations gives floating point accuracy to within 2 significant bits, and will pass platformio's Unity TEST_ASSERT_EQUAL_FLOAT
    u.f *= 1.5F - (0.5F * x * u.f * u.f); // Second iteration
#endif

    return u.f;
// NOLINTEND(cppcoreguidelines-pro-type-union-access)
#else
    return 1.0F / sqrtf(x);
#endif
}

xyz_t Quaternion::rotate(const xyz_t& v) const
{
    const float x2 = x*x;
    const float y2 = y*y;
    const float z2 = z*z;
    // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    return xyz_t {
        2.0F*(v.x*(0.5F - y2 - z2) + v.y*(x*y - w*z)      + v.z*(w*y + x*z)),
        2.0F*(v.x*(w*z + x*y)      + v.y*(0.5F - x2 - z2) + v.z*(y*z - w*x)),
        2.0F*(v.x*(x*z - w*y)      + v.y*(w*x + y*z)      + v.z*(0.5F - x2 - y2))
    };
    // NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
}

/*!
Create a quaternion representing the rotation from the source to the destination vector.
src and dst do not need to be normalized.
*/
Quaternion::Quaternion(const xyz_t& src, const xyz_t& dst)
{
    static constexpr float epsilon = 1E-5F;

    xyz_t cross_product = src.cross(dst);
    const float dot_product = src.dot(dst);

    if (cross_product.magnitude() < epsilon && dot_product < 0.0F) {
        // handle corner cases with 180 degree rotations
        // if the two vectors are parallel, cross product is zero
        // if they point opposite, the dot product is negative
        w = 0.0F;
        const xyz_t source_absolute = src.absolute();
        if (source_absolute.x < source_absolute.y) {
            if (source_absolute.x < source_absolute.z) {
                cross_product = src.cross(xyz_t{1.0F, 0.0F, 0.0F});
            } else {
                cross_product = src.cross(xyz_t{0.0F, 0.0F, 1.0F});
            }
        } else {
            if (source_absolute.y < source_absolute.z) {
                cross_product = src.cross(xyz_t{0.0F, 1.0F, 0.0F});
            } else {
                cross_product = src.cross(xyz_t{0.0F, 0.0F, 1.0F});
            }
        }
    } else {
        // normal case, do half-way quaternion solution
        w = dot_product + std::sqrt(src.magnitude_squared() * dst.magnitude_squared());
    }
    x = cross_product.x;
    y = cross_product.y;
    z = cross_product.z;
    normalize_in_place();
}


/*!
Create a Quaternion from roll, pitch, and yaw Euler angles (in radians).
See:
https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_angles_(in_3-2-1_sequence)_to_quaternion_conversion
*/
Quaternion Quaternion::from_euler_angles_radians(float rollRadians, float pitchRadians, float yawRadians)
{
#if defined(LIBRARY_VECTOR_QUATERNION_MATRIX_USE_FAST_TRIGONOMETRY)
    // NOLINTBEGIN(misc-const-correctness)
    float sin_half_roll {};
    float cos_half_roll {};
    FastTrigonometry::sin_cos(0.5F*rollRadians, sin_half_roll, cos_half_roll);
    float sin_half_pitch {};
    float cos_half_pitch {};
    FastTrigonometry::sin_cos(0.5F*pitchRadians, sin_half_pitch, cos_half_pitch);
    float sin_half_yaw {};
    float cos_half_yaw {};
    FastTrigonometry::sin_cos(0.5F*yawRadians, sin_half_yaw, cos_half_yaw);
    // NOLINTEND(misc-const-correctness)
#else
    const float half_roll = 0.5F * rollRadians;
    const float half_pitch = 0.5F * pitchRadians;
    const float half_yaw = 0.5F * yawRadians;

    const float sin_half_roll = sinf(half_roll);
    const float cos_half_roll = cosf(half_roll);
    const float sin_half_pitch = sinf(half_pitch);
    const float cos_half_pitch = cosf(half_pitch);
    const float sin_half_yaw = sinf(half_yaw);
    const float cos_half_yaw = cosf(half_yaw);
#endif
    return {
        cos_half_roll * cos_half_pitch * cos_half_yaw + sin_half_roll * sin_half_pitch * sin_half_yaw,
        sin_half_roll * cos_half_pitch * cos_half_yaw - cos_half_roll * sin_half_pitch * sin_half_yaw,
        cos_half_roll * sin_half_pitch * cos_half_yaw + sin_half_roll * cos_half_pitch * sin_half_yaw,
        cos_half_roll * cos_half_pitch * sin_half_yaw - sin_half_roll * sin_half_pitch * cos_half_yaw
    };
}

/*!
Create a Quaternion from roll and pitch Euler angles (in radians), assumes yaw angle is zero.
*/
Quaternion Quaternion::from_euler_angles_radians(float rollRadians, float pitchRadians)
{
#if defined(LIBRARY_VECTOR_QUATERNION_MATRIX_USE_FAST_TRIGONOMETRY)
    // NOLINTBEGIN(misc-const-correctness)
    float sin_half_roll {};
    float cos_half_roll {};
    FastTrigonometry::sin_cos(0.5F*rollRadians, sin_half_roll, cos_half_roll);
    float sin_half_pitch {};
    float cos_half_pitch {};
    FastTrigonometry::sin_cos(0.5F*pitchRadians, sin_half_pitch, cos_half_pitch);
    // NOLINTEND(misc-const-correctness)
#else
    const float half_roll = 0.5F * rollRadians;
    const float half_pitch = 0.5F * pitchRadians;

    const float sin_half_roll = sinf(half_roll);
    const float cos_half_roll = cosf(half_roll);
    const float sin_half_pitch = sinf(half_pitch);
    const float cos_half_pitch = cosf(half_pitch);
#endif

    return {
        cos_half_roll * cos_half_pitch,
        sin_half_roll * cos_half_pitch,
        cos_half_roll * sin_half_pitch,
        -sin_half_roll * sin_half_pitch
    };
}
/*!
Create a Quaternion from roll, pitch, and yaw Euler angles (in degrees).
*/
Quaternion Quaternion::from_euler_angles_degrees(float rollDegrees, float pitchDegrees, float yawDegrees)
{
    return from_euler_angles_radians(rollDegrees*DEGREES_TO_RADIANS, pitchDegrees*DEGREES_TO_RADIANS, yawDegrees*DEGREES_TO_RADIANS);
}

/*!
Create a Quaternion from roll and pitch Euler angles (in degrees), assumes yaw angle is zero.
*/
Quaternion Quaternion::from_euler_angles_degrees(float rollDegrees, float pitchDegrees)
{
    return from_euler_angles_radians(rollDegrees*DEGREES_TO_RADIANS, pitchDegrees*DEGREES_TO_RADIANS);
}

/*!
Create Rotation Matrix from a Quaternion.

Adapted from [Converting a Rotation Matrix to a Quaternion](https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2015/01/matrix-to-quat.pdf) by Mike Day.
Note that Day's paper uses the [Shuster multiplication convention](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Alternative_conventions),
rather than the Hamilton multiplication convention used by the Quaternion class.
*/
Quaternion Matrix3x3::quaternion() const // NOLINT(readability-convert-member-functions-to-static) false positive
{
/*
_a[0] = 1 - 2(yy + zz);
_a[1] = 2(xy - wz);
_a[2] = 2(wy + xz);
_a[3] = 2(wz + xy);
_a[4] = 1 - 2(xx + zz);
_a[5] = 2(yz - wx);
_a[6] = 2(xz - wy);
_a[7] = 2(wx + yz);
_a[8] = 1 - 2(xx + yy);

_a[0] - _a[4] = 2(xx-yy)
_a[4] - _a[8] = 2(yy-zz)
_a[8] - _a[0] = 2(zz-xx)

_a[1] + _a[3] = 2(xy - wz) + 2(wz + xy) = 4xy
_a[3] - _a[1] = 2(wz + xy) - 2(xy - wz) = 4wz
_a[2] + _a[6] = 2(wy + xz) + 2(xz - wy) = 4xz
_a[2] - _a[6] = 2(wy + xz) - 2(xz - wy) = 4wy
_a[5] + _a[7] = 2(yz - wx) + 2(wx + yz) = 4yz
_a[7] - _a[5] = 2(wx + yz) - 2(yz - wx) = 4xw
*/

// NOLINTBEGIN(cppcoreguidelines-init-variables,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers) avoid false positives
    // Choose largest scale factor from 4w, 4x, 4y, and 4z, to avoid a scale factor of zero, or numerical instabilities caused by division of a small scale factor.
    if (_a[8] < 0) {
        // |(x,y)| is bigger than |(z,w)|?
        if (_a[0] > _a[4]) {
            // |x| bigger than |y|, so use x-form
            const float t = 1.0F + (_a[0] - _a[4]) - _a[8]; // 1 + 2(xx - yy) - 1 + 2(xx + yy) = 4xx
            const Quaternion q = Quaternion(_a[7] - _a[5], t, _a[1] + _a[3], _a[6] + _a[2]);
            return q * (0.5F * reciprocal_sqrtf(t)); // note brackets because we want perform the scalar multiply first, so it is only done once
        }
        // |y| bigger than |x|, so use y-form
        const float t = 1 - (_a[0] - _a[4]) - _a[8]; // 1 - 2(xx - yy) - 1 + 2(xx + yy) = 4yy
        const Quaternion q = Quaternion(_a[2] - _a[6], _a[1] + _a[3], t, _a[5] + _a[7]);
        return q * (0.5F * reciprocal_sqrtf(t)); // note brackets because we want perform the scalar multiply first, so it is only done once
    }

    // |(z,w)| bigger than |(x,y)|
    if (_a[0] < -_a[4]) {
        // |z| bigger than |w|, so use z-form
        const float t = 1.0F - _a[0] - (_a[4] - _a[8]); // 1 - (1 - 2*(yy + zz)) - (2(yy - zz)) = 4zz
        const Quaternion q = Quaternion(_a[3] - _a[1], _a[2] + _a[6], _a[5] + _a[7], t);
        return q * (0.5F * reciprocal_sqrtf(t)); // note brackets because we want perform the scalar multiply first, so it is only done once
    }

    // |w| bigger than |z|, so use w-form
    // ww + xx + yy + zz = 1, since unit quaternion, so xx + yy + zz =  1 - ww
    const float t = 1.0F + _a[0] + _a[4] + _a[8]; // 1 + 1 - 2*(yy + zz) + 1 - 2(xx + zz) + 1 - 2(xx + yy) =  4 - 4(xx + yy + zz) = 4 - 4(1 - ww) = 4ww
    const Quaternion q = Quaternion(t, _a[7] - _a[5], _a[2] - _a[6], _a[3] - _a[1]);
    return q * (0.5F * reciprocal_sqrtf(t)); // note brackets because we want perform the scalar multiply first, so it is only done once
// NOLINTEND(cppcoreguidelines-init-variables,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
}

/*!
Create a Rotation Matrix from roll, pitch, and yaw Euler angles (in degrees).
*/
Matrix3x3 Matrix3x3::from_euler_angles_radians(float rollRadians, float pitchRadians, float yawRadians)
{
#if defined(LIBRARY_VECTOR_QUATERNION_MATRIX_USE_FAST_TRIGONOMETRY)
    // NOLINTBEGIN(misc-const-correctness)
    float sinPhi {};
    float cosPhi {};
    FastTrigonometry::sin_cos(rollRadians, sinPhi, cosPhi);
    float sinTheta {};
    float cosTheta {};
    FastTrigonometry::sin_cos(pitchRadians, sinTheta, cosTheta);
    float sinPsi {};
    float cosPsi {};
    FastTrigonometry::sin_cos(yawRadians, sinPsi, cosPsi);
    // NOLINTEND(misc-const-correctness)
#else
    const float sinPhi = sinf(rollRadians);
    const float cosPhi = cosf(rollRadians);
    const float sinTheta = sinf(pitchRadians);
    const float cosTheta = cosf(pitchRadians);
    const float sinPsi = sinf(yawRadians);
    const float cosPsi = cosf(yawRadians);
#endif
    return Matrix3x3 {
         cosTheta*cosPsi,
        -cosPhi*sinPsi + sinPhi*sinTheta*cosPsi,
         sinPhi*sinPsi + cosPhi*sinTheta*cosPsi,
         cosTheta*sinPsi,
         cosPhi*cosPsi + sinPhi*sinTheta*sinPsi,
        -sinPhi*cosPsi + cosPhi*sinTheta*sinPsi,
        -sinTheta,
         sinPhi*cosTheta,
         cosPhi*cosTheta
    };
}

/*!
Create a Rotation Matrix from roll, pitch, and yaw Euler angles (in degrees).
*/
Matrix3x3 Matrix3x3::from_euler_angles_degrees(float rollDegrees, float pitchDegrees, float yawDegrees)
{
    return from_euler_angles_radians(rollDegrees*Quaternion::DEGREES_TO_RADIANS, pitchDegrees*Quaternion::DEGREES_TO_RADIANS, yawDegrees*Quaternion::DEGREES_TO_RADIANS);
}

/*!
Return the normalized vector
*/
xy_t xy_t::normalized() const
{
    const float r = reciprocal_sqrtf(magnitude_squared());
    return *this*r;
}

/*!
Return the normalized vector
*/
xyz_t xyz_t::normalized() const
{
    const float r = reciprocal_sqrtf(magnitude_squared());
    return *this*r;
}

/*!
Return the normalized quaternion
*/
Quaternion Quaternion::normalized() const
{
    const float r = reciprocal_sqrtf(magnitude_squared());
    return *this*r;
}

/*
See
https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_angles_(in_3-2-1_sequence)_conversion
for Quaternion to Euler angles conversion.
*/
float Quaternion::calculate_roll_radians() const
{
    return atan2f(w*x + y*z, 0.5F - x*x - y*y); // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
}

float Quaternion::calculate_pitch_radians() const
{
    return asinf(2.0F*(w*y - x*z)); // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
}

float Quaternion::calculate_yaw_radians() const
{
    return atan2f(w*z + x*y, 0.5F - y*y - z*z); // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    // alternatively
    // return atan2f(2*(w*z + x*y), w*w + x*x - y*y - z*z);
}

float Quaternion::sin_roll() const
{
    const float a = w*x + y*z;
    const float b = 0.5F - x*x - y*y;
    return a * reciprocal_sqrtf(a*a + b*b);
}
/*!
clip sin(rollAngle) to +/-1.0F when roll angle outside range [-90 degrees, 90 degrees]
*/
float Quaternion::sin_roll_clipped() const
{
    const float a = w*x + y*z;
    const float b = 0.5F - x*x - y*y;
    return std::signbit(b) ? std::copysignf(1.0F, a) : a * reciprocal_sqrtf(a*a + b*b);
}

float Quaternion::cos_roll() const
{
    const float a = w*x + y*z;
    const float b = 0.5F - x*x - y*y;
    return b * reciprocal_sqrtf(a*a + b*b);
}

float Quaternion::cos_pitch() const
{
    const float s = sin_pitch();
    return sqrtf(1.0F - s*s);
}

float Quaternion::tan_pitch() const
{
    const float s = sin_pitch();
    return s * reciprocal_sqrtf(1.0F - s*s);
}

float Quaternion::cos_yaw() const
{
    const float a = w*z + x*y;
    const float b = 0.5F - y*y - z*z;
    return b * reciprocal_sqrtf(a*a + b*b);
}

float Quaternion::sin_yaw() const
{
    const float a = w*z + x*y;
    const float b = 0.5F - y*y - z*z;
    return a * reciprocal_sqrtf(a*a + b*b);
}
