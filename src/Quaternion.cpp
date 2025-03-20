#include <Matrix3x3.h>
#include <Quaternion.h>

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
Create a Quaternion using roll, pitch, and yaw Euler angles.
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
Create a Quaternion using roll and pitch Euler angles, assumes yaw angle is zero.
*/
Quaternion Quaternion::fromEulerAnglesRadians(float rollRadians, float pitchRadians)
{
    const float halfRoll = 0.5F * rollRadians;
    const float halfPitch = 0.5F * pitchRadians;

    const float sinHalfPitch = sinf(halfPitch);
    const float cosHalfPitch = cosf(halfPitch);
    const float sinHalfRoll = sinf(halfRoll);
    const float cosHalfRoll = cosf(halfRoll);

    return {
        cosHalfRoll * cosHalfPitch,
        sinHalfRoll * cosHalfPitch,
        cosHalfRoll * sinHalfPitch,
        sinHalfRoll * sinHalfPitch
    };
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
_a[0] = 1 - 2*(yy + zz);
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
            return q * (0.5F / sqrtf(t));
        }
        // |y| bigger than |x|, so use y-form
        const float t = 1 - (_a[0] - _a[4]) - _a[8]; // 1 - 2(xx - yy) - 1 + 2(xx + yy) = 4yy
        const Quaternion q = Quaternion(_a[2] - _a[6], _a[1] + _a[3], t, _a[5] + _a[7]);
        return q * (0.5F / sqrtf(t));
    }

    // |(z,w)| bigger than |(x,y)|
    if (_a[0] < -_a[4]) {
        // |z| bigger than |w|, so use z-form
        const float t = 1.0F - _a[0] - (_a[4] - _a[8]); // 1 - (1 - 2*(yy + zz)) - (2(yy - zz)) = 4zz
        const Quaternion q = Quaternion(_a[3] - _a[1], _a[2] + _a[6], _a[5] + _a[7], t);
        return q * (0.5F / sqrtf(t));
    }

    // |w| bigger than |z|, so use w-form
    // ww + xx + yy + zz = 1, since unit quaternion, so xx + yy + zz =  1 - ww
    const float t = 1.0F + _a[0] + _a[4] + _a[8]; // 1 + 1 - 2*(yy + zz) + 1 - 2(xx + zz) + 1 - 2(xx + yy) =  4 - 4(xx + yy + zz) = 4 - 4(1 - ww) = 4ww
    const Quaternion q = Quaternion(t, _a[7] - _a[5], _a[2] - _a[6], _a[3] - _a[1]);
    return q * (0.5F / sqrtf(t));
// NOLINTEND(cppcoreguidelines-init-variables,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
}
