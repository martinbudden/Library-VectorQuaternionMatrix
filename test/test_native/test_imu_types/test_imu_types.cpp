#include "Quaternion.h"
#include "xyz_type.h"
#include <unity.h>

void setUp() {
}

void tearDown() {
}

void test_xyz_t()
{
    const xyz_t a{2, 3, 5};
    TEST_ASSERT_TRUE(a == +a);
    const xyz_t minusA{-2, -3, -5};
    TEST_ASSERT_TRUE(minusA == -a);

    const xyz_t b{7, 11, 17};
    TEST_ASSERT_TRUE(a != b);
    TEST_ASSERT_FALSE(a == b);

    const xyz_t a2{4, 6, 10};
    TEST_ASSERT_TRUE(a2 == a*2);
    TEST_ASSERT_TRUE(a2 == 2*a);

    xyz_t a2dividedBy2 = a2;
    a2dividedBy2 /= 2;
    TEST_ASSERT_EQUAL_FLOAT(a.x, a2dividedBy2.x);
    TEST_ASSERT_EQUAL_FLOAT(a.y, a2dividedBy2.y);
    TEST_ASSERT_EQUAL_FLOAT(a.z, a2dividedBy2.z);

    TEST_ASSERT_TRUE(a == a2/2);
    TEST_ASSERT_TRUE(a == a2dividedBy2);

    const xyz_t b2{14, 22, 34};
    TEST_ASSERT_TRUE(b2 == b*2);

    xyz_t c = b;
    c *= 2;
    TEST_ASSERT_TRUE(c == b*2);

    const xyz_t b2dividedBy2 = b2 / 2;
    TEST_ASSERT_EQUAL_FLOAT(b.x, b2dividedBy2.x);
    TEST_ASSERT_EQUAL_FLOAT(b.y, b2dividedBy2.y);
    TEST_ASSERT_EQUAL_FLOAT(b.z, b2dividedBy2.z);

    TEST_ASSERT_TRUE(b == b2/2);
    TEST_ASSERT_TRUE(b == b2dividedBy2);


    const xyz_t a_plus_b =  a + b;

    TEST_ASSERT_EQUAL(9, a_plus_b.x);
    TEST_ASSERT_EQUAL(14, a_plus_b.y);
    TEST_ASSERT_EQUAL(22, a_plus_b.z);

    TEST_ASSERT_TRUE(a_plus_b == a + b);
    TEST_ASSERT_TRUE(a_plus_b == b + a);

    const xyz_t a_minus_b =  a - b;

    TEST_ASSERT_EQUAL(-5, a_minus_b.x);
    TEST_ASSERT_EQUAL(-8, a_minus_b.y);
    TEST_ASSERT_EQUAL(-12, a_minus_b.z);
    TEST_ASSERT_TRUE(a_minus_b == a - b);
    TEST_ASSERT_TRUE(-a_minus_b == b - a);

    TEST_ASSERT_TRUE(a2 == a_plus_b + a_minus_b);
    TEST_ASSERT_TRUE(b2 == a_plus_b - a_minus_b);

    const float a_dot_b =  14 + 33 + 85;
    TEST_ASSERT_TRUE(a_dot_b == a.dot_product(b));
    TEST_ASSERT_TRUE(a.magnitude_squared() == a.dot_product(a));
    TEST_ASSERT_TRUE(b.magnitude_squared() == b.dot_product(b));

    const xyz_t a_cross_b = a.cross_product(b);
    TEST_ASSERT_EQUAL(3*17 - 5*11, a_cross_b.x);
    TEST_ASSERT_EQUAL(-2*17 + 5*7, a_cross_b.y);
    TEST_ASSERT_EQUAL(2*11 - 3*7, a_cross_b.z);

    TEST_ASSERT_TRUE(a_cross_b != b.cross_product(a));

    const xyz_t d{-100, 200, 50};
    const xyz_t d_clipped = xyz_t{-50, 50, 50};
    TEST_ASSERT_TRUE(d.clip(-50, 50) == d_clipped);

    const xyz_t e{-100, 200, 50};
    const xyz_t e_clipped = xyz_t{-20, 20, 20};
    TEST_ASSERT_TRUE(e.clip(-20, 20) == e_clipped);

    xyz_t f{-100, 200, 50};
    f = f.clip(-20, 20);
    const xyz_t f_clipped = xyz_t{-20, 20, 20};
    TEST_ASSERT_TRUE(f == f_clipped);

}

void test_quaternion()
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

    TEST_ASSERT_TRUE(a.magnitude_squared()*a.magnitude_squared() == (a*a).magnitude_squared());
    TEST_ASSERT_TRUE(a.magnitude_squared()*a.magnitude_squared() == (a*a.conjugate()).magnitude_squared());

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
    TEST_ASSERT_EQUAL_FLOAT(1.0, q0.magnitude_squared());

    const Quaternion q1 = Quaternion::fromEulerAnglesRadians(degrees43inRadians, -degrees19inRadians, degrees67inRadians);
    TEST_ASSERT_EQUAL_FLOAT(43.0, q1.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(-19.0, q1.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(67.0, q1.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q1.magnitude_squared());
}

void test_quaternion_rotation()
{
    const Quaternion q2 = Quaternion::fromEulerAnglesRadians(degrees43inRadians, 0, 0);
    TEST_ASSERT_EQUAL_FLOAT(43.0, q2.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q2.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q2.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q2.magnitude_squared());

    const Quaternion q3 = Quaternion::fromEulerAnglesRadians(-degrees19inRadians, 0, 0);
    TEST_ASSERT_EQUAL_FLOAT(-19.0, q3.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q3.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q3.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q3.magnitude_squared());

    const Quaternion q2q3 = q2 * q3;
    TEST_ASSERT_EQUAL_FLOAT(1.0, q2q3.magnitude_squared());
    TEST_ASSERT_EQUAL_FLOAT(24.0, q2q3.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q2q3.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q2q3.calculateYawDegrees());

    const Quaternion q4 = Quaternion::fromEulerAnglesRadians(0, -degrees67inRadians, 0);
    TEST_ASSERT_EQUAL_FLOAT(0.0, q4.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(-67.0, q4.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q4.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q4.magnitude_squared());

    const Quaternion q5 = Quaternion::fromEulerAnglesRadians(0, degrees43inRadians, 0);
    TEST_ASSERT_EQUAL_FLOAT(0.0, q5.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(43.0, q5.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q5.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q5.magnitude_squared());

    const Quaternion q4q5 = q4 * q5;
    TEST_ASSERT_EQUAL_FLOAT(1.0, q4q5.magnitude_squared());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q4q5.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(-24.0, q4q5.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q4q5.calculateYawDegrees());

    const Quaternion q6 = Quaternion::fromEulerAnglesRadians(0, 0, -degrees19inRadians);
    TEST_ASSERT_EQUAL_FLOAT(0.0, q6.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q6.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(-19.0, q6.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q6.magnitude_squared());

    const Quaternion q7 = Quaternion::fromEulerAnglesRadians(0, 0, degrees43inRadians);
    TEST_ASSERT_EQUAL_FLOAT(0.0, q7.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q7.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(43.0, q7.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q7.magnitude_squared());

    const Quaternion q6q7 = q6 * q7;
    TEST_ASSERT_EQUAL_FLOAT(1.0, q6q7.magnitude_squared());
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

void test_quaternion_apply_delta()
{
    const Quaternion qI(1.0F, 0.0F, 0.0F, 0.0F);
    Quaternion qD = qI;
    const float delta = degrees19inRadians;
    const Quaternion qDExpected = qI * Quaternion(cosf(delta/2.0F), 0, 0, sinf(delta/2.0F));
    qD.applyDelta(delta);
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getW(), qD.getW());
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getX(), qD.getX());
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getY(), qD.getY());
    TEST_ASSERT_EQUAL_FLOAT(qDExpected.getZ(), qD.getZ());
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_xyz_t);
    RUN_TEST(test_quaternion);
    RUN_TEST(test_quaternion_angles);
    RUN_TEST(test_quaternion_rotation);
    RUN_TEST(test_quaternion_rotate);
    RUN_TEST(test_quaternion_apply_delta);

    UNITY_END();
}
