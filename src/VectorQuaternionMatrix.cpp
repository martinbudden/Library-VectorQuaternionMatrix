#include <Matrix3x3.h>
#include <Quaternion.h>
#include <cstdint>

/*
NOTE: USE_TRIGONOMETRIC_APPROXIMATIONS not fully working and should not be used
*/

/*!
Reciprocal square root
Implementation of [fast inverse square root](http://en.wikipedia.org/wiki/Fast_inverse_square_root)
using [Pizer’s optimisation](https://pizer.wordpress.com/2008/10/12/fast-inverse-square-root/) and
using `union` rather than `reinterpret_cast` to avoid breaking strict-aliasing rules.

The Xtensa floating point coprocessor (used on the ESP32) has some hardware support for reciprocal square root: it has
an RSQRT0.S (single-precision reciprocal square root initial step) instruction.
However benchmarking shows that FAST_RECIPROCAL_SQUARE_ROOT is approximately 3.5 times faster than `1.0F / sqrtf()`
*/
inline float reciprocalSqrtf(float x)
{
#if defined(USE_FAST_RECIPROCAL_SQUARE_ROOT) || defined(USE_FAST_RECIPROCAL_SQUARE_ROOT_TWO_ITERATIONS)
    union {
        float f;
        int32_t i;
    } u { .f = x };

// NOLINTBEGIN(cppcoreguidelines-pro-type-union-access)
    u.i = 0x5f1f1412 - (u.i >> 1); // Initial estimate for Newton–Raphson method
    // single iteration gives accuracy to 4.5 significant figures
    u.f *= 1.69000231F - 0.714158168F * x * u.f * u.f; // First iteration
#if defined(USE_FAST_RECIPROCAL_SQUARE_ROOT_TWO_ITERATIONS)
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
    return xyz_t {
        2.0F*(v.x*(0.5F - y2 - z2) + v.y*(x*y - w*z)      + v.z*(w*y + x*z)),
        2.0F*(v.x*(w*z + x*y)      + v.y*(0.5F - x2 - z2) + v.z*(y*z - w*x)),
        2.0F*(v.x*(x*z - w*y)      + v.y*(w*x + y*z)      + v.z*(0.5F - x2 - y2))
    };
}

/*!
Create a Quaternion from roll, pitch, and yaw Euler angles (in radians).
See:
https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_angles_(in_3-2-1_sequence)_to_quaternion_conversion
*/
Quaternion Quaternion::fromEulerAnglesRadians(float rollRadians, float pitchRadians, float yawRadians)
{
    const float halfRoll = 0.5F * rollRadians;
    const float halfPitch = 0.5F * pitchRadians;
    const float halfYaw = 0.5F * yawRadians;

    const float sinHalfRoll = sinf(halfRoll);
    const float cosHalfRoll = cosf(halfRoll);
    const float sinHalfPitch = sinf(halfPitch);
    const float cosHalfPitch = cosf(halfPitch);
    const float sinHalfYaw = sinf(halfYaw);
    const float cosHalfYaw = cosf(halfYaw);

    return {
        cosHalfRoll * cosHalfPitch * cosHalfYaw + sinHalfRoll * sinHalfPitch * sinHalfYaw,
        sinHalfRoll * cosHalfPitch * cosHalfYaw - cosHalfRoll * sinHalfPitch * sinHalfYaw,
        cosHalfRoll * sinHalfPitch * cosHalfYaw + sinHalfRoll * cosHalfPitch * sinHalfYaw,
        cosHalfRoll * cosHalfPitch * sinHalfYaw - sinHalfRoll * sinHalfPitch * cosHalfYaw
    };
}

/*!
Create a Quaternion from roll and pitch Euler angles (in radians), assumes yaw angle is zero.
*/
Quaternion Quaternion::fromEulerAnglesRadians(float rollRadians, float pitchRadians)
{
    const float halfRoll = 0.5F * rollRadians;
    const float halfPitch = 0.5F * pitchRadians;

    const float sinHalfRoll = sinf(halfRoll);
    const float cosHalfRoll = cosf(halfRoll);
    const float sinHalfPitch = sinf(halfPitch);
    const float cosHalfPitch = cosf(halfPitch);

    return {
        cosHalfRoll * cosHalfPitch,
        sinHalfRoll * cosHalfPitch,
        cosHalfRoll * sinHalfPitch,
        -sinHalfRoll * sinHalfPitch
    };
}
/*!
Create a Quaternion from roll, pitch, and yaw Euler angles (in degrees).
*/
Quaternion Quaternion::fromEulerAnglesDegrees(float rollDegrees, float pitchDegrees, float yawDegrees)
{
    return fromEulerAnglesRadians(rollDegrees*degreesToRadians, pitchDegrees*degreesToRadians, yawDegrees*degreesToRadians);
}

/*!
Create a Quaternion from roll and pitch Euler angles (in degrees), assumes yaw angle is zero.
*/
Quaternion Quaternion::fromEulerAnglesDegrees(float rollDegrees, float pitchDegrees)
{
    return fromEulerAnglesRadians(rollDegrees*degreesToRadians, pitchDegrees*degreesToRadians);
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
            return q * (0.5F * reciprocalSqrtf(t)); // note brackets because we want perform the scalar multiply first, so it is only done once
        }
        // |y| bigger than |x|, so use y-form
        const float t = 1 - (_a[0] - _a[4]) - _a[8]; // 1 - 2(xx - yy) - 1 + 2(xx + yy) = 4yy
        const Quaternion q = Quaternion(_a[2] - _a[6], _a[1] + _a[3], t, _a[5] + _a[7]);
        return q * (0.5F * reciprocalSqrtf(t)); // note brackets because we want perform the scalar multiply first, so it is only done once
    }

    // |(z,w)| bigger than |(x,y)|
    if (_a[0] < -_a[4]) {
        // |z| bigger than |w|, so use z-form
        const float t = 1.0F - _a[0] - (_a[4] - _a[8]); // 1 - (1 - 2*(yy + zz)) - (2(yy - zz)) = 4zz
        const Quaternion q = Quaternion(_a[3] - _a[1], _a[2] + _a[6], _a[5] + _a[7], t);
        return q * (0.5F * reciprocalSqrtf(t)); // note brackets because we want perform the scalar multiply first, so it is only done once
    }

    // |w| bigger than |z|, so use w-form
    // ww + xx + yy + zz = 1, since unit quaternion, so xx + yy + zz =  1 - ww
    const float t = 1.0F + _a[0] + _a[4] + _a[8]; // 1 + 1 - 2*(yy + zz) + 1 - 2(xx + zz) + 1 - 2(xx + yy) =  4 - 4(xx + yy + zz) = 4 - 4(1 - ww) = 4ww
    const Quaternion q = Quaternion(t, _a[7] - _a[5], _a[2] - _a[6], _a[3] - _a[1]);
    return q * (0.5F * reciprocalSqrtf(t)); // note brackets because we want perform the scalar multiply first, so it is only done once
// NOLINTEND(cppcoreguidelines-init-variables,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
}

/*!
Return the normalized vector
*/
xyz_t xyz_t::normalize() const
{
    const float r = reciprocalSqrtf(magnitudeSquared());
    return *this*r;
}

/*!
Return the normalized quaternion
*/
Quaternion Quaternion::normalize() const
{
    const float r = reciprocalSqrtf(magnitudeSquared());
    return *this*r;
}

float Quaternion::arcsinClippedf(float x)
{
    if (x <= -static_cast<float>(static_cast<float>(M_PI_2))) { return {-static_cast<float>(M_PI_2)}; }
    if (x >=  static_cast<float>(M_PI_2)) { return {static_cast<float>(M_PI_2)}; }
#if defined(USE_TRIGONOMETRIC_APPROXIMATIONS)
    return arcsinApproximatef(x);
#else
    return asinf(x);
#endif
}

/*
See
https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_angles_(in_3-2-1_sequence)_conversion
for Quaternion to Euler angles conversion.
*/
float Quaternion::calculateRollRadians() const
{
#if defined(USE_TRIGONOMETRIC_APPROXIMATIONS)
    return arctan2Approximatef(w*x + y*z, 0.5F - x*x - y*y);
#else
    return atan2f(w*x + y*z, 0.5F - x*x - y*y); // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
#endif
}

float Quaternion::calculatePitchRadians() const
{
    return arcsinClippedf(2.0F*(w*y - x*z)); // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
}

float Quaternion::calculateYawRadians() const
{
#if defined(USE_TRIGONOMETRIC_APPROXIMATIONS)
    return atan2f(w*z + x*y, 0.5F - y*y - z*z); // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    //return arctan2Approximatef(w*z + x*y, 0.5F - y*y - z*z); // alternatively atan2f(2*(w*z + x*y), w*w + x*x - y*y - z*z)
#else
    return atan2f(w*z + x*y, 0.5F - y*y - z*z); // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    // alternatively 
    // return atan2f(2*(w*z + x*y), w*w + x*x - y*y - z*z);
#endif
}

float Quaternion::sinRoll() const
{
    const float a = w*x + y*z;
    const float b = 0.5F - x*x - y*y;
    return a * reciprocalSqrtf(a*a + b*b);
}

float Quaternion::sinRollClipped() const
{
    const float a = w*x + y*z;
    const float b = 0.5F - x*x - y*y;
    return std::signbit(b) ? std::copysignf(1.0F, a) : a * reciprocalSqrtf(a*a + b*b);
}

float Quaternion::cosRoll() const
{
    const float a = w*x + y*z;
    const float b = 0.5F - x*x - y*y;
    return b * reciprocalSqrtf(a*a + b*b);
}

float Quaternion::cosPitch() const
{
    const float s = sinPitch();
    return sqrtf(1.0F - s*s);
}

float Quaternion::tanPitch() const
{
    const float s = sinPitch();
    return s * reciprocalSqrtf(1.0F - s*s);
}

float Quaternion::cosYaw() const
{
    const float a = w*z + x*y;
    const float b = 0.5F - y*y - z*z;
    return b * reciprocalSqrtf(a*a + b*b);
}

float Quaternion::sinYaw() const
{
    const float a = w*z + x*y;
    const float b = 0.5F - y*y - z*z;
    return a * reciprocalSqrtf(a*a + b*b);
}


/*
Trigonometric approximations
*/
float Quaternion::arctanApproximatef(float x)
{
    // NOTE: assumes x is in the range [0, 1.0F]
    static constexpr float c1 =  3.14551666E-07F;
    static constexpr float c2 =  0.99997357F;
    static constexpr float c3 =  0.14744007F;
    static constexpr float c4 =  0.30998143F;
    static constexpr float c5 =  0.050301764F;
    static constexpr float d1 =  0.14710391F;
    static constexpr float d2 =  0.64446417F;

    return (c1 + x*(c2 + x*(c3 + x*(c4 - x*c5)))) / (1.0F + x*(d1 + x*d2));
}

float Quaternion::arctan2Approximatef(float y, float x)
{
    static constexpr float PI_F = 3.141592653589793F;
    static constexpr float HALF_PI_F = 0.5F * 3.141592653589793F;

    if (x == 0.0F) {
        return y > 0.0F ? HALF_PI_F : -HALF_PI_F;
    }
    const float r = y/x;
    // atan(x) = pi/2 - atan(1/x) for x > 1
    float a = 0.0F;
    if (r > 0) {
        a = (r > 1.0F) ? HALF_PI_F - arctanApproximatef(1.0F/r) : arctanApproximatef(r);
    } else {
        a = (r < -1.0F) ? HALF_PI_F + arctanApproximatef(-1.0F/r) : -arctanApproximatef(-r);
    }
    if (x > 0.0F) {
        return a;
    }
    return (y < 0.0F) ? a - PI_F : a + PI_F;

}

float Quaternion::arcsinRestrictedXf(float x)
{
    // works for x in range [0, SQUARE_ROOT_HALF]
    // see https://wrfranklin.org/Research/Short_Notes/arcsin/top.shtml
    // numerator coefficients
    static constexpr float n1 =  0.5689111419F;
    static constexpr float n2 = -0.2644381021F;
    static constexpr float n3 = -0.4212611542F;
    static constexpr float n4 =  0.1475622352F;
    // denominator coefficients
    static constexpr float d1 =  2.006022274F;
    static constexpr float d2 = -2.343685222F;
    static constexpr float d3 =  0.3316406750F;
    static constexpr float d4 =  0.02607135626F;

    const float y = 2.0F*x - 1.0F;
    const float y2 = y*y;
    const float y3 = y2*y;

    const float ret = (n1 + n2*x + n3*y2 + n4*y3) / (d1 + d2*x + d3*y2 + d4*y3);
    return ret;
}

float Quaternion::arcsinApproximatef(float x)
{
    static constexpr float HALF_PI_F = 0.5F * 3.141592653589793F;
    static constexpr float SQUARE_ROOT_HALF_F = 0.707106781186548F;

    // for abs(x) > SQUARE_ROOT_HALF_F, arcsin(x) = HALF_PI_F - arcsin(sqrtf(1.0F - x*x))
    if(x < 0.0F) {
        return (x >= -SQUARE_ROOT_HALF_F) ? -arcsinRestrictedXf(-x) : -HALF_PI_F + arcsinRestrictedXf(sqrtf(1.0F - x*x));
    }
    return (x < SQUARE_ROOT_HALF_F) ? arcsinRestrictedXf(x) : HALF_PI_F - arcsinRestrictedXf(sqrtf(1.0F - x*x));
}

float Quaternion::arccosApproximatef(float x)
{
    static constexpr float HALF_PI_F = 0.5F * 3.141592653589793F;

    return HALF_PI_F - arcsinApproximatef(x);
}
