#include <Quaternion.h>


Quaternion Quaternion::fromEulerAnglesRadians(float rollRadians, float pitchRadians, float yawRadians)
{
    const float halfRoll = 0.5F * rollRadians;
    const float halfPitch = 0.5F * pitchRadians;
    const float halfYaw = 0.5F * yawRadians;

    const float sinHalfRoll = sin(halfRoll);
    const float cosHalfRoll = cos(halfRoll);
    const float sinHalfPitch = sin(halfPitch);
    const float cosHalfPitch = cos(halfPitch);
    const float sinHalfYaw = sin(halfYaw);
    const float cosHalfYaw = cos(halfYaw);

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

    const float sinHalfPitch = sin(halfPitch);
    const float cosHalfPitch = cos(halfPitch);
    const float sinHalfRoll = sin(halfRoll);
    const float cosHalfRoll = cos(halfRoll);

    return Quaternion(
        cosHalfRoll * cosHalfPitch,
        sinHalfRoll * cosHalfPitch,
        cosHalfRoll * sinHalfPitch,
        sinHalfRoll * sinHalfPitch
    );
}
