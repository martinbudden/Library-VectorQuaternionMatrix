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
    TEST_ASSERT_EQUAL_FLOAT(1.0F, aN.magnitude());
    
    TEST_ASSERT_EQUAL_FLOAT(aNE.getW(), aN.getW());
    TEST_ASSERT_EQUAL_FLOAT(aNE.getX(), aN.getX());
    TEST_ASSERT_EQUAL_FLOAT(aNE.getY(), aN.getY());
    TEST_ASSERT_EQUAL_FLOAT(aNE.getZ(), aN.getZ());

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
constexpr float degrees90inRadians = 90.0F * Quaternion::degreesToRadians;

void test_quaternion_angles()
{
    const Quaternion q0 = Quaternion::fromEulerAnglesRadians(degrees19inRadians, degrees43inRadians, degrees67inRadians);
    TEST_ASSERT_EQUAL_FLOAT(19.0, q0.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(43.0, q0.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(67.0, q0.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q0.magnitudeSquared());

    const Quaternion q1 = Quaternion::fromEulerAnglesRadians(degrees43inRadians, -degrees19inRadians, degrees67inRadians);
    TEST_ASSERT_EQUAL_FLOAT(43.0, q1.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(-19.0, q1.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(67.0, q1.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q1.magnitudeSquared());
}

void test_quaternion_rotation()
{
    const Quaternion q2 = Quaternion::fromEulerAnglesRadians(degrees43inRadians, 0, 0);
    TEST_ASSERT_EQUAL_FLOAT(43.0, q2.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q2.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q2.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q2.magnitudeSquared());

    const Quaternion q3 = Quaternion::fromEulerAnglesRadians(-degrees19inRadians, 0, 0);
    TEST_ASSERT_EQUAL_FLOAT(-19.0, q3.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q3.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q3.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q3.magnitudeSquared());

    const Quaternion q2q3 = q2 * q3;
    TEST_ASSERT_EQUAL_FLOAT(1.0, q2q3.magnitudeSquared());
    TEST_ASSERT_EQUAL_FLOAT(24.0, q2q3.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q2q3.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q2q3.calculateYawDegrees());

    const Quaternion q4 = Quaternion::fromEulerAnglesRadians(0, -degrees67inRadians, 0);
    TEST_ASSERT_EQUAL_FLOAT(0.0, q4.calculateRollDegrees());
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
    TEST_ASSERT_TRUE(qDExpected == q19);
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
    TEST_ASSERT_TRUE(qDExpected == q19);
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
    TEST_ASSERT_TRUE(qDExpected == q19);
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_quaternion_operators);
    RUN_TEST(test_quaternion_functions);
    RUN_TEST(test_quaternion_angles);
    RUN_TEST(test_quaternion_rotation);
    RUN_TEST(test_quaternion_rotate);
    RUN_TEST(test_quaternion_rotate_x);
    RUN_TEST(test_quaternion_rotate_y);
    RUN_TEST(test_quaternion_rotate_z);

    UNITY_END();
}
