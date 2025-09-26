#include "Quaternion.h"
#include <unity.h>

void setUp() {
}

void tearDown() {
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)
void test_quaternion_operators()
{
    const Quaternion a{2, 3, 5, 7};
    TEST_ASSERT_TRUE(a == +a);
    const Quaternion minusA{-2, -3, -5, -7};
    TEST_ASSERT_TRUE(minusA == -a);

    const Quaternion b{11, 13, 17, 23};
    TEST_ASSERT_TRUE(a != b);
    TEST_ASSERT_FALSE(a == b);

    const Quaternion a2{4, 6, 10, 14};
    TEST_ASSERT_TRUE(a2 == a*2);
    TEST_ASSERT_TRUE(a2 == 2*a);

    Quaternion a2dividedBy2 = a2;
    a2dividedBy2 /= 2;
    TEST_ASSERT_EQUAL_FLOAT(a.getW(), a2dividedBy2.getW());
    TEST_ASSERT_EQUAL_FLOAT(a.getX(), a2dividedBy2.getX());
    TEST_ASSERT_EQUAL_FLOAT(a.getY(), a2dividedBy2.getY());
    TEST_ASSERT_EQUAL_FLOAT(a.getZ(), a2dividedBy2.getZ());

    const Quaternion b2{22, 26, 34, 46};
    TEST_ASSERT_TRUE(b2 == b*2);
    TEST_ASSERT_TRUE(b2 == 2*b);
    const Quaternion b2dividedBy2 = b2 / 2;
    TEST_ASSERT_EQUAL_FLOAT(b.getW(), b2dividedBy2.getW());
    TEST_ASSERT_EQUAL_FLOAT(b.getX(), b2dividedBy2.getX());
    TEST_ASSERT_EQUAL_FLOAT(b.getY(), b2dividedBy2.getY());
    TEST_ASSERT_EQUAL_FLOAT(b.getZ(), b2dividedBy2.getZ());

    TEST_ASSERT_TRUE(b == b2/2);
    TEST_ASSERT_TRUE(b == b2dividedBy2);


    const Quaternion a_plus_b =  a + b;

    TEST_ASSERT_EQUAL(13, a_plus_b.getW());
    TEST_ASSERT_EQUAL(16, a_plus_b.getX());
    TEST_ASSERT_EQUAL(22, a_plus_b.getY());
    TEST_ASSERT_EQUAL(30, a_plus_b.getZ());
    TEST_ASSERT_TRUE(a_plus_b == a + b);

    const Quaternion a_minus_b =  a - b;

    TEST_ASSERT_EQUAL(-9, a_minus_b.getW());
    TEST_ASSERT_EQUAL(-10, a_minus_b.getX());
    TEST_ASSERT_EQUAL(-12, a_minus_b.getY());
    TEST_ASSERT_EQUAL(-16, a_minus_b.getZ());
    TEST_ASSERT_TRUE(a_minus_b == a - b);
    TEST_ASSERT_TRUE(-a_minus_b == b - a);

    TEST_ASSERT_TRUE(a2 == a_plus_b + a_minus_b);
    TEST_ASSERT_TRUE(b2 == a_plus_b - a_minus_b);

    Quaternion c = a;
    c *= b;
    TEST_ASSERT_TRUE(c == a*b);
}

void test_quaternion_functions()
{
    const Quaternion a{2, 3, 5, 7};
    const Quaternion b{11, 13, 17, 23};

    const Quaternion aC{2, -3, -5, -7};
    TEST_ASSERT_TRUE(aC == a.conjugate());

    TEST_ASSERT_EQUAL_FLOAT(87.0F, a.magnitudeSquared());
    TEST_ASSERT_EQUAL_FLOAT(sqrtf(87.0F), a.magnitude());
    TEST_ASSERT_EQUAL_FLOAT(1108.0F, b.magnitudeSquared());
    TEST_ASSERT_EQUAL_FLOAT(sqrtf(1108.0F), b.magnitude());

    const Quaternion aNE{2.0F/sqrt(87.0F), 3.0F/sqrt(87.0F), 5.0F/sqrt(87.0F), 7.0F/sqrt(87.0F)};
    const Quaternion aN = a.normalize();
#if defined(LIBRARY_VECTOR_QUATERNION_MATRIX_USE_FAST_RECIPROCAL_SQUARE_ROOT)
    TEST_ASSERT_FLOAT_WITHIN(0.00047, 1.0F, aN.magnitude());
    TEST_ASSERT_FLOAT_WITHIN(0.000081, aNE.getW(), aN.getW());
#else
    TEST_ASSERT_EQUAL_FLOAT(1.0F, aN.magnitude());
    TEST_ASSERT_EQUAL_FLOAT(aNE.getW(), aN.getW());
    TEST_ASSERT_EQUAL_FLOAT(aNE.getX(), aN.getX());
    TEST_ASSERT_EQUAL_FLOAT(aNE.getY(), aN.getY());
    TEST_ASSERT_EQUAL_FLOAT(aNE.getZ(), aN.getZ());
#endif
    Quaternion aN2 = a;
    aN2.normalizeInPlace();
    TEST_ASSERT_EQUAL_FLOAT(1.0F, aN2.magnitude());
    TEST_ASSERT_EQUAL_FLOAT(aNE.getW(), aN2.getW());
    TEST_ASSERT_EQUAL_FLOAT(aNE.getX(), aN2.getX());
    TEST_ASSERT_EQUAL_FLOAT(aNE.getY(), aN2.getY());
    TEST_ASSERT_EQUAL_FLOAT(aNE.getZ(), aN2.getZ());

    TEST_ASSERT_TRUE(a.magnitudeSquared()*a.magnitudeSquared() == (a*a).magnitudeSquared());
    TEST_ASSERT_TRUE(a.magnitudeSquared()*a.magnitudeSquared() == (a*a.conjugate()).magnitudeSquared());
}

constexpr float degrees19inRadians = 19.0F * Quaternion::degreesToRadians;
constexpr float degrees43inRadians = 43.0F * Quaternion::degreesToRadians;
constexpr float degrees45inRadians = 45.0F * Quaternion::degreesToRadians;
constexpr float degrees67inRadians = 67.0F * Quaternion::degreesToRadians;
constexpr float degrees89inRadians = 89.0F * Quaternion::degreesToRadians;
constexpr float degrees90inRadians = 90.0F * Quaternion::degreesToRadians;
constexpr float degrees91inRadians = 91.0F * Quaternion::degreesToRadians;
constexpr float degrees95inRadians = 95.0F * Quaternion::degreesToRadians;

void test_quaternion_angles()
{
    const Quaternion q0 = Quaternion::fromEulerAnglesRadians(degrees19inRadians, degrees43inRadians, degrees67inRadians);
    TEST_ASSERT_EQUAL_FLOAT(19.0, q0.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(43.0, q0.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(67.0, q0.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q0.magnitudeSquared());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q0.magnitude());

    const Quaternion q0d = Quaternion::fromEulerAnglesDegrees(19.0F, 43.0F, 67.0F);
    TEST_ASSERT_EQUAL_FLOAT(19.0, q0d.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(43.0, q0d.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(67.0, q0d.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q0d.magnitudeSquared());

    const Quaternion q1 = Quaternion::fromEulerAnglesRadians(degrees19inRadians, degrees43inRadians);
    TEST_ASSERT_EQUAL_FLOAT(19.0, q1.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(43.0, q1.calculatePitchDegrees());
    TEST_ASSERT_FLOAT_WITHIN(1.87e-05, 0.0, q1.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q1.magnitudeSquared());

    const Quaternion q2 = Quaternion::fromEulerAnglesRadians(degrees43inRadians, -degrees19inRadians, degrees67inRadians);
    TEST_ASSERT_EQUAL_FLOAT(43.0, q2.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(-19.0, q2.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(67.0, q2.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q2.magnitudeSquared());

    const Quaternion q2d = Quaternion::fromEulerAnglesDegrees(19.0F, 43.0F, -67.0F);
    TEST_ASSERT_EQUAL_FLOAT(19.0, q2d.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(43.0, q2d.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(-67.0, q2d.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q2d.magnitudeSquared());

    const Quaternion q3 = Quaternion::fromEulerAnglesDegrees(21.0F, -39.0F);
    TEST_ASSERT_EQUAL_FLOAT(21.0, q3.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(-39.0, q3.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q3.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q3.magnitudeSquared());

    const Quaternion q4 = Quaternion::fromEulerAnglesDegrees(21.0F, -39.0F, -37.0F);
    TEST_ASSERT_EQUAL_FLOAT(21.0, q4.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(-39.0, q4.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(-37.0, q4.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q4.magnitudeSquared());
}

void test_quaternion_angles_sin_cos_tan()
{
    const Quaternion q0 = Quaternion::fromEulerAnglesRadians(degrees19inRadians, degrees43inRadians, degrees67inRadians);
    TEST_ASSERT_EQUAL_FLOAT(19.0, q0.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(43.0, q0.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(67.0, q0.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q0.magnitudeSquared());

    TEST_ASSERT_EQUAL_FLOAT(sinf(degrees19inRadians), q0.sinRoll());
    TEST_ASSERT_EQUAL_FLOAT(cosf(degrees19inRadians), q0.cosRoll());
    TEST_ASSERT_EQUAL_FLOAT(tanf(degrees19inRadians), q0.tanRoll());

    TEST_ASSERT_EQUAL_FLOAT(sinf(degrees43inRadians), q0.sinPitch());
    TEST_ASSERT_EQUAL_FLOAT(cosf(degrees43inRadians), q0.cosPitch());
    TEST_ASSERT_EQUAL_FLOAT(tanf(degrees43inRadians), q0.tanPitch());

    TEST_ASSERT_EQUAL_FLOAT(sinf(degrees67inRadians), q0.sinYaw());
    TEST_ASSERT_EQUAL_FLOAT(cosf(degrees67inRadians), q0.cosYaw());
    TEST_ASSERT_EQUAL_FLOAT(tanf(degrees67inRadians), q0.tanYaw());

    const Quaternion q1 = Quaternion::fromEulerAnglesRadians(-degrees19inRadians, -degrees43inRadians, -degrees67inRadians);
    TEST_ASSERT_EQUAL_FLOAT(-19.0, q1.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(-43.0, q1.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(-67.0, q1.calculateYawDegrees()); //!!TODO: this fails using trig approximations, returns 113 rather than -67

    TEST_ASSERT_EQUAL_FLOAT(sinf(-degrees19inRadians), q1.sinRoll());
    TEST_ASSERT_EQUAL_FLOAT(cosf(-degrees19inRadians), q1.cosRoll());
    TEST_ASSERT_EQUAL_FLOAT(tanf(-degrees19inRadians), q1.tanRoll());

    TEST_ASSERT_EQUAL_FLOAT(sinf(-degrees43inRadians), q1.sinPitch());
    TEST_ASSERT_EQUAL_FLOAT(cosf(-degrees43inRadians), q1.cosPitch());
    TEST_ASSERT_EQUAL_FLOAT(tanf(-degrees43inRadians), q1.tanPitch());

    TEST_ASSERT_EQUAL_FLOAT(sinf(-degrees67inRadians), q1.sinYaw());
    TEST_ASSERT_EQUAL_FLOAT(cosf(-degrees67inRadians), q1.cosYaw());
    TEST_ASSERT_EQUAL_FLOAT(tanf(-degrees67inRadians), q1.tanYaw());
}

void test_quaternion_rotation()
{
    const Quaternion q2 = Quaternion::fromEulerAnglesRadians(degrees43inRadians, 0, 0);
    TEST_ASSERT_EQUAL_FLOAT(43.0, q2.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q2.calculatePitchDegrees());
    TEST_ASSERT_FLOAT_WITHIN(1.81E-05, 0.0, q2.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q2.magnitudeSquared());

    const Quaternion q3 = Quaternion::fromEulerAnglesRadians(-degrees19inRadians, 0, 0);
    TEST_ASSERT_EQUAL_FLOAT(-19.0, q3.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q3.calculatePitchDegrees());
    TEST_ASSERT_FLOAT_WITHIN(1.81E-05, 0.0, q3.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q3.magnitudeSquared());

    const Quaternion q2q3 = q2 * q3;
    TEST_ASSERT_EQUAL_FLOAT(1.0, q2q3.magnitudeSquared());
    TEST_ASSERT_EQUAL_FLOAT(24.0, q2q3.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q2q3.calculatePitchDegrees());
    TEST_ASSERT_FLOAT_WITHIN(1.81E-05, 0.0, q2q3.calculateYawDegrees());

    const Quaternion q4 = Quaternion::fromEulerAnglesRadians(0, -degrees67inRadians, 0);
    TEST_ASSERT_FLOAT_WITHIN(1.81E-05, 0.0, q4.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(-67.0, q4.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q4.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q4.magnitudeSquared());

    const Quaternion q5 = Quaternion::fromEulerAnglesRadians(0, degrees43inRadians, 0);
    TEST_ASSERT_EQUAL_FLOAT(0.0, q5.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(43.0, q5.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q5.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q5.magnitudeSquared());

    const Quaternion q4q5 = q4 * q5;
    TEST_ASSERT_EQUAL_FLOAT(1.0, q4q5.magnitudeSquared());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q4q5.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(-24.0, q4q5.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q4q5.calculateYawDegrees());

    const Quaternion q6 = Quaternion::fromEulerAnglesRadians(0, 0, -degrees19inRadians);
    TEST_ASSERT_EQUAL_FLOAT(0.0, q6.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q6.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(-19.0, q6.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q6.magnitudeSquared());

    const Quaternion q7 = Quaternion::fromEulerAnglesRadians(0, 0, degrees43inRadians);
    TEST_ASSERT_EQUAL_FLOAT(0.0, q7.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q7.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(43.0, q7.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q7.magnitudeSquared());

    const Quaternion q6q7 = q6 * q7;
    TEST_ASSERT_EQUAL_FLOAT(1.0, q6q7.magnitudeSquared());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q6q7.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q6q7.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(24.0, q6q7.calculateYawDegrees());
}

void test_quaternion_rotate()
{
    const Quaternion qI(1.0F, 0.0F, 0.0F, 0.0F);
    const xyz_t vx {1.0F, 0.0F, 0.0F};
    const xyz_t vy {0.0F, 1.0F, 0.0F};
    const xyz_t vz {0.0F, 0.0F, 1.0F};

    const xyz_t vxRI = qI.rotate(vx);
    TEST_ASSERT_TRUE(vx == vxRI);

    const xyz_t vyRI = qI.rotate(vy);
    TEST_ASSERT_TRUE(vy == vyRI);

    const xyz_t vzRI = qI.rotate(vz);
    TEST_ASSERT_TRUE(vz == vzRI);

    const Quaternion qX = Quaternion::fromEulerAnglesRadians(degrees90inRadians, 0, 0);
    TEST_ASSERT_TRUE(vx == qX.rotate(vx));
    TEST_ASSERT_EQUAL_FLOAT(0.0F, qX.rotate(vy).x);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-07, 0.0F, qX.rotate(vy).y);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, qX.rotate(vy).z);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, qX.rotate(vz).x);
    TEST_ASSERT_EQUAL_FLOAT(-1.0F, qX.rotate(vz).y);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-07, 0.0F, qX.rotate(vz).z);

    const Quaternion qY = Quaternion::fromEulerAnglesRadians(0, degrees90inRadians, 0);
    TEST_ASSERT_TRUE(vy == qY.rotate(vy));
    TEST_ASSERT_FLOAT_WITHIN(1.0e-07, 0.0F, qY.rotate(vx).x);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, qY.rotate(vx).y);
    TEST_ASSERT_EQUAL_FLOAT(-1.0F, qY.rotate(vx).z);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, qY.rotate(vz).x);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, qY.rotate(vz).y);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-07, 0.0F, qY.rotate(vz).z);

    const Quaternion qZ = Quaternion::fromEulerAnglesRadians(0, 0, degrees90inRadians);
    TEST_ASSERT_TRUE(vz == qZ.rotate(vz));
    TEST_ASSERT_FLOAT_WITHIN(1.0e-07, 0.0F, qZ.rotate(vx).x);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, qZ.rotate(vx).y);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, qZ.rotate(vx).z);
    TEST_ASSERT_EQUAL_FLOAT(-1.0F, qZ.rotate(vy).x);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-07, 0.0F, qZ.rotate(vy).y);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, qZ.rotate(vy).z);
}

void test_quaternion_rotate_x()
{
    const Quaternion qI(1.0F, 0.0F, 0.0F, 0.0F);
    const float delta = degrees19inRadians;
    Quaternion qD = qI;
    qD.rotateX(delta);

    const Quaternion qDExpected = Quaternion(cosf(delta/2.0F), sinf(delta/2.0F), 0, 0) * qI;
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getW(), qD.getW());
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getX(), qD.getX());
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getY(), qD.getY());
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getZ(), qD.getZ());
    TEST_ASSERT_TRUE(qDExpected == qD);

    const Quaternion q19 = Quaternion::fromEulerAnglesRadians(degrees19inRadians, 0, 0);
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getW(), q19.getW());
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getX(), q19.getX());
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getY(), q19.getY());
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getZ(), q19.getZ());
#if !defined(LIBRARY_VECTOR_QUATERNION_MATRIX_USE_FAST_TRIGONOMETRY)
    TEST_ASSERT_TRUE(qDExpected == q19);
#endif
}

void test_quaternion_rotate_y()
{
    const Quaternion qI(1.0F, 0.0F, 0.0F, 0.0F);
    const float delta = degrees19inRadians;
    Quaternion qD = qI;
    qD.rotateY(delta);

    const Quaternion qDExpected = Quaternion(cosf(delta/2.0F), 0, sinf(delta/2.0F), 0) *qI;
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getW(), qD.getW());
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getX(), qD.getX());
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getY(), qD.getY());
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getZ(), qD.getZ());
    TEST_ASSERT_TRUE(qDExpected == qD);

    const Quaternion q19 = Quaternion::fromEulerAnglesRadians(0, degrees19inRadians, 0);
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getW(), q19.getW());
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getX(), q19.getX());
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getY(), q19.getY());
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getZ(), q19.getZ());
#if !defined(LIBRARY_VECTOR_QUATERNION_MATRIX_USE_FAST_TRIGONOMETRY)
    TEST_ASSERT_TRUE(qDExpected == q19);
#endif
}

void test_quaternion_rotate_z()
{
    const Quaternion qI(1.0F, 0.0F, 0.0F, 0.0F);
    const float delta = degrees19inRadians;
    Quaternion qD = qI;
    qD.rotateZ(delta);

    const Quaternion qDExpected = Quaternion(cosf(delta/2.0F), 0, 0, sinf(delta/2.0F)) *qI;
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getW(), qD.getW());
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getX(), qD.getX());
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getY(), qD.getY());
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getZ(), qD.getZ());
    TEST_ASSERT_TRUE(qDExpected == qD);

    const Quaternion q19 = Quaternion::fromEulerAnglesRadians(0, 0, degrees19inRadians);
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getW(), q19.getW());
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getX(), q19.getX());
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getY(), q19.getY());
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getZ(), q19.getZ());
#if !defined(LIBRARY_VECTOR_QUATERNION_MATRIX_USE_FAST_TRIGONOMETRY)
    TEST_ASSERT_TRUE(qDExpected == q19);
#endif
}

void test_quaternion_rotate_enu_to_ned()
{
    // ENU roll, pitch, yaw -> NED 
    static const Quaternion qENUtoNED(0.0F, sqrtf(0.5F), sqrtf(0.5F), 0.0F);

    Quaternion orientationNED = Quaternion::fromEulerAnglesRadians(degrees19inRadians, 0.0F, 0.0F);
    orientationNED = orientationNED * qENUtoNED;
    TEST_ASSERT_EQUAL_FLOAT(180.0F, orientationNED.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(180.0F+ 19.0F, orientationNED.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(90.0F, orientationNED.calculateYawDegrees());

    orientationNED = Quaternion::fromEulerAnglesRadians(0.0F, degrees43inRadians, 0.0F) * qENUtoNED;
    TEST_ASSERT_EQUAL_FLOAT(43.0F - 180.0F, orientationNED.calculateRollDegrees());
    TEST_ASSERT_EQUAL(180.0F, orientationNED.calculatePitchDegrees()); //!!TODO: check this
    TEST_ASSERT_EQUAL_FLOAT(90.0F, orientationNED.calculateYawDegrees());

    orientationNED = Quaternion::fromEulerAnglesRadians(0.0F, 0.0F, degrees67inRadians) * qENUtoNED;
    TEST_ASSERT_EQUAL_FLOAT(180.0F, orientationNED.calculateRollDegrees());
    TEST_ASSERT_EQUAL(180.0F, orientationNED.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(90.0F + 67.0F, orientationNED.calculateYawDegrees());

    orientationNED = qENUtoNED * Quaternion::fromEulerAnglesRadians(degrees19inRadians, degrees43inRadians, degrees67inRadians);
    TEST_ASSERT_EQUAL_FLOAT(19.0F - 180.0F, orientationNED.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(180.0F + 43.0F, orientationNED.calculatePitchDegrees()); //!!TODO: check this
    TEST_ASSERT_EQUAL_FLOAT(90.F - 67.0F, orientationNED.calculateYawDegrees());
}

void test_roll_pitch()
{
    Quaternion q = Quaternion::fromEulerAnglesDegrees(19.0F, 29.0F, 47.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.8921514F, q.w);
#if defined(LIBRARY_VECTOR_QUATERNION_MATRIX_USE_FAST_TRIGONOMETRY)
    TEST_ASSERT_EQUAL_FLOAT(0.04806738F, q.x);
#else
    TEST_ASSERT_EQUAL_FLOAT(0.04806787F, q.x);
#endif
    TEST_ASSERT_EQUAL_FLOAT(0.2901808F, q.y);
    TEST_ASSERT_EQUAL_FLOAT(0.3428564F, q.z);
    TEST_ASSERT_EQUAL_FLOAT(19.0F, q.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(29.0F, q.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(47.0F, q.calculateYawDegrees());

    q = Quaternion::fromEulerAnglesDegrees(-19.0F, 29.0F, 47.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.8591951F, q.w);
    TEST_ASSERT_EQUAL_FLOAT(-0.245007F, q.x);
    TEST_ASSERT_EQUAL_FLOAT(0.1627482F, q.y);
    TEST_ASSERT_EQUAL_FLOAT(0.4186508F, q.z);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.0F, q.w*q.w - q.y*q.y);
    TEST_ASSERT_EQUAL_FLOAT(-19.0F, q.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(29.0F, q.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(47.0F, q.calculateYawDegrees());

    q = Quaternion::fromEulerAnglesDegrees(119.0F, 29.0F, 47.0F);
    TEST_ASSERT_EQUAL_FLOAT(119.0F, q.calculateRollDegrees());
    q = Quaternion::fromEulerAnglesDegrees(-119.0F, 29.0F, 47.0F);
    TEST_ASSERT_EQUAL_FLOAT(-119.0F, q.calculateRollDegrees());

    q = Quaternion::fromEulerAnglesDegrees(119.0F, 129.0F, 47.0F);
    TEST_ASSERT_EQUAL_FLOAT(129.0F, q.calculatePitchDegrees());
    q = Quaternion::fromEulerAnglesDegrees(119.0F, 179.0F, 47.0F);
    TEST_ASSERT_EQUAL_FLOAT(179.0F, q.calculatePitchDegrees());
    q = Quaternion::fromEulerAnglesDegrees(119.0F, 229.0F, 47.0F);
    TEST_ASSERT_EQUAL_FLOAT(229.0F, q.calculatePitchDegrees());
    q = Quaternion::fromEulerAnglesDegrees(119.0F, -229.0F, 47.0F);
    TEST_ASSERT_EQUAL_FLOAT(-229.0F, q.calculatePitchDegrees());

    q = Quaternion::fromEulerAnglesDegrees(119.0F, -29.0F, 47.0F);
    TEST_ASSERT_EQUAL_FLOAT(-29.0F, q.calculatePitchDegrees());

    q = Quaternion::fromEulerAnglesDegrees(119.0F, -129.0F, 47.0F);
    TEST_ASSERT_EQUAL_FLOAT(-129.0F, q.calculatePitchDegrees());

    q = Quaternion::fromEulerAnglesDegrees(119.0F, -129.0F, -47.0F);
    TEST_ASSERT_EQUAL_FLOAT(-129.0F, q.calculatePitchDegrees());
    q = Quaternion::fromEulerAnglesDegrees(-119.0F, -129.0F, -47.0F);
    TEST_ASSERT_EQUAL_FLOAT(-129.0F, q.calculatePitchDegrees());
}

void test_roll_angle_clip()
{
   TEST_ASSERT_EQUAL(false, std::signbit(0.01F));
   TEST_ASSERT_EQUAL(true, std::signbit(-0.01F));

    // float tanRoll() const { return (w*x + y*z)/(0.5F - x*x - y*y); }
    const Quaternion q0 = Quaternion::fromEulerAnglesRadians(degrees89inRadians, 0.0F, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(89.0, q0.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.4999238F, q0.getW()*q0.getX() + q0.getY()*q0.getZ());
    TEST_ASSERT_FLOAT_WITHIN(0.0000003, 0.008726478F, 0.5F - q0.getX()*q0.getX() - q0.getY()*q0.getY());
    TEST_ASSERT_EQUAL(false, std::signbit(0.5F - q0.getX()*q0.getX() - q0.getY()*q0.getY()));
    TEST_ASSERT_EQUAL(sinf(degrees89inRadians), q0.sinRoll());
    TEST_ASSERT_EQUAL(sinf(degrees89inRadians), q0.sinRollClipped());

    const Quaternion q1 = Quaternion::fromEulerAnglesRadians(-degrees89inRadians, 0.0F, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(-89.0F, q1.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(-0.4999238F, q1.getW()*q1.getX() + q1.getY()*q1.getZ());
    TEST_ASSERT_FLOAT_WITHIN(0.0000003, 0.008726478F, 0.5F - q1.getX()*q1.getX() - q1.getY()*q1.getY());
    TEST_ASSERT_EQUAL(false, std::signbit(0.5F - q1.getX()*q1.getX() - q1.getY()*q1.getY()));
    TEST_ASSERT_EQUAL_FLOAT(sinf(-degrees89inRadians), q1.sinRoll());
    TEST_ASSERT_EQUAL_FLOAT(sinf(-degrees89inRadians), q1.sinRollClipped());

    const Quaternion q2 = Quaternion::fromEulerAnglesRadians(degrees91inRadians, 0.0F, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(91.0, q2.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.4999238F, q2.getW()*q2.getX() + q2.getY()*q2.getZ());
    TEST_ASSERT_FLOAT_WITHIN(0.0000003, -0.00872612F, 0.5F - q2.getX()*q2.getX() - q2.getY()*q2.getY());
    TEST_ASSERT_EQUAL(true, std::signbit(0.5F - q2.getX()*q2.getX() - q2.getY()*q2.getY()));
    TEST_ASSERT_EQUAL_FLOAT(sinf(degrees91inRadians), q2.sinRoll());
    TEST_ASSERT_EQUAL_FLOAT(1.0F, q2.sinRollClipped());

    const Quaternion q3 = Quaternion::fromEulerAnglesRadians(-degrees91inRadians, 0.0F, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(-91.0, q3.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(-0.4999238F, q3.getW()*q3.getX() + q3.getY()*q3.getZ());
    TEST_ASSERT_FLOAT_WITHIN(0.0000003, -0.00872612F, 0.5F - q3.getX()*q3.getX() - q3.getY()*q3.getY());
    TEST_ASSERT_EQUAL(true, std::signbit(0.5F - q3.getX()*q3.getX() - q3.getY()*q3.getY()));
    TEST_ASSERT_EQUAL_FLOAT(sinf(-degrees91inRadians), q3.sinRoll());
    TEST_ASSERT_EQUAL_FLOAT(-1.0F, q3.sinRollClipped());
}

void test_pitch_angle_clip()
{
    // float sinPitch() const { return 2.0F*(w*y - x*z); }

    const Quaternion q0 = Quaternion::fromEulerAnglesRadians(0.0F, degrees89inRadians, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, q0.magnitudeSquared());
    TEST_ASSERT_EQUAL_FLOAT(89.0, q0.calculatePitchDegrees());
    TEST_ASSERT_FLOAT_WITHIN(0.0000004, 0.01745278F, q0.getW()*q0.getW() - q0.getY()*q0.getY());
    TEST_ASSERT_EQUAL_FLOAT(0.7132504F, q0.getW());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, q0.getX());
    TEST_ASSERT_EQUAL_FLOAT(0.7009092, q0.getY());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, q0.getZ());
    TEST_ASSERT_EQUAL(false, std::signbit(q0.getW()*q0.getW() - q0.getY()*q0.getY()));
    TEST_ASSERT_EQUAL_FLOAT(sinf(degrees89inRadians), q0.sinPitch());
    TEST_ASSERT_EQUAL_FLOAT(sinf(degrees89inRadians), q0.sinPitchClipped());

    const Quaternion q2 = Quaternion::fromEulerAnglesRadians(0.0F, degrees91inRadians, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, q2.magnitudeSquared());
    TEST_ASSERT_EQUAL_FLOAT(91.0, q2.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(-0.01745245F, q2.getW()*q2.getW() - q2.getY()*q2.getY());
    TEST_ASSERT_EQUAL_FLOAT(0.4999238F, q2.getW()*q2.getY() - q2.getX()*q2.getZ());
    TEST_ASSERT_EQUAL_FLOAT(0.7009092F, q2.getW());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, q2.getX());
    TEST_ASSERT_EQUAL_FLOAT(0.7132504, q2.getY());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, q2.getZ());
    TEST_ASSERT_EQUAL(true, std::signbit(q2.getW()*q2.getW() - q2.getY()*q2.getY()));
    TEST_ASSERT_EQUAL_FLOAT(sinf(degrees91inRadians), q2.sinPitch());
    TEST_ASSERT_EQUAL_FLOAT(1.0F, q2.sinPitchClipped());

    const Quaternion q1 = Quaternion::fromEulerAnglesRadians(0.0F, -degrees89inRadians, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, q1.magnitudeSquared());
    TEST_ASSERT_EQUAL_FLOAT(-89.0, q1.calculatePitchDegrees());
    TEST_ASSERT_FLOAT_WITHIN(0.0000004, 0.01745278F, q1.getW()*q1.getW() - q1.getY()*q1.getY());
    TEST_ASSERT_EQUAL_FLOAT(0.7132504F, q1.getW());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, q1.getX());
    TEST_ASSERT_EQUAL_FLOAT(-0.7009092, q1.getY());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, q1.getZ());
    TEST_ASSERT_EQUAL(false, std::signbit(q1.getW()*q1.getW() - q1.getY()*q1.getY()));
    TEST_ASSERT_EQUAL_FLOAT(sinf(-degrees89inRadians), q1.sinPitch());
    TEST_ASSERT_EQUAL_FLOAT(sinf(-degrees89inRadians), q1.sinPitchClipped());

    const Quaternion q3 = Quaternion::fromEulerAnglesRadians(0.0F, -degrees91inRadians, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, q3.magnitudeSquared());
    TEST_ASSERT_EQUAL_FLOAT(-91.0, q3.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(-0.01745245F, q3.getW()*q3.getW() - q3.getY()*q3.getY());
    TEST_ASSERT_EQUAL_FLOAT(0.4999238F, q2.getW()*q2.getY() - q2.getX()*q2.getZ());
    TEST_ASSERT_EQUAL_FLOAT(0.7009092, q3.getW());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, q3.getX());
    TEST_ASSERT_EQUAL_FLOAT(-0.7132504F, q3.getY());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, q3.getZ());
    TEST_ASSERT_EQUAL(true, std::signbit(q3.getW()*q3.getW() - q3.getY()*q3.getY()));
    TEST_ASSERT_EQUAL_FLOAT(sinf(-degrees91inRadians), q3.sinPitch());
    TEST_ASSERT_EQUAL_FLOAT(-1.0F, q3.sinPitchClipped());

    //TEST_ASSERT_EQUAL_FLOAT(91.0, q2.calculatePitchDegrees()); // was 88.99927

    const Quaternion q4 = Quaternion::fromEulerAnglesRadians(0.0F, -degrees95inRadians, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(-95.0F, q4.calculatePitchDegrees());
}

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_quaternion_operators);
    RUN_TEST(test_quaternion_functions);
    RUN_TEST(test_quaternion_angles);
    RUN_TEST(test_quaternion_angles_sin_cos_tan);
    RUN_TEST(test_quaternion_rotation);
    RUN_TEST(test_quaternion_rotate);
    RUN_TEST(test_quaternion_rotate_x);
    RUN_TEST(test_quaternion_rotate_y);
    RUN_TEST(test_quaternion_rotate_z);
    RUN_TEST(test_quaternion_rotate_enu_to_ned);

    RUN_TEST(test_roll_pitch);
    RUN_TEST(test_roll_angle_clip);
    RUN_TEST(test_pitch_angle_clip);

    UNITY_END();
}
