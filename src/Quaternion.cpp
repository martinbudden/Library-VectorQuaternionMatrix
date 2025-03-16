#include <Quaternion.h>


xyz_t Quaternion::rotate(const xyz_t& v) const
{
    const float x2 = x*x;
    const float y2 = y*y;
    const float z2 = z*z;
    return xyz_t {
        v.x*(0.5F - y2 - z2) + v.y*(x*y - w*z)      + v.z*(w*y + x*z),
        v.x*(w*z + x*y)      + v.y*(0.5F - x2 - z2) + v.z*(y*z - w*x),
        v.x*(x*z - w*y)      + v.y*(w*x + y*z)      + v.z*(0.5F - x2 - y2)
    } * 2.0F;
}

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
