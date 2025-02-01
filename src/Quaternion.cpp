#include <Quaternion.h>


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

    return Quaternion(
        cosHalfRoll * cosHalfPitch * cosHalfYaw + sinHalfRoll * sinHalfPitch * sinHalfYaw,
        sinHalfRoll * cosHalfPitch * cosHalfYaw - cosHalfRoll * sinHalfPitch * sinHalfYaw,
        cosHalfRoll * sinHalfPitch * cosHalfYaw + sinHalfRoll * cosHalfPitch * sinHalfYaw,
        cosHalfRoll * cosHalfPitch * sinHalfYaw - sinHalfRoll * sinHalfPitch * cosHalfYaw
    );
}

Quaternion Quaternion::fromEulerAnglesRadians(float rollRadians, float pitchRadians)
{
    const float halfRoll = 0.5F * rollRadians;
    const float halfPitch = 0.5F * pitchRadians;

    const float sinHalfPitch = sinf(halfPitch);
    const float cosHalfPitch = cosf(halfPitch);
    const float sinHalfRoll = sinf(halfRoll);
    const float cosHalfRoll = cosf(halfRoll);

    return Quaternion(
        cosHalfRoll * cosHalfPitch,
        sinHalfRoll * cosHalfPitch,
        cosHalfRoll * sinHalfPitch,
        sinHalfRoll * sinHalfPitch
    );
}
