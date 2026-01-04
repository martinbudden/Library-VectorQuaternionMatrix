#include "xyz_type.h"
#include <unity.h>

void setUp() {
}

void tearDown() {
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)
void test_xyz_t_assignment()
{
    xyz_t a{2.0F, 3.0F, 5.0F};
    TEST_ASSERT_EQUAL(10.0F, a.sum());
    TEST_ASSERT_EQUAL(10.F/3.0F, a.mean());
    TEST_ASSERT_EQUAL(30.0F, a.prod());
    const xyz_t b{7.0F, 11.0F, 13.0F};

    a = b;
    TEST_ASSERT_EQUAL(b.x, a.x);
    TEST_ASSERT_EQUAL(b.y, a.y);
    TEST_ASSERT_EQUAL(b.z, a.z);

    a = 17.0F;
    TEST_ASSERT_EQUAL(17.0F, a.x);
    TEST_ASSERT_EQUAL(17.0F, a.y);
    TEST_ASSERT_EQUAL(17.0F, a.z);

    a = 0.0F;
    TEST_ASSERT_EQUAL(0.0F, a.x);
    TEST_ASSERT_EQUAL(0.0F, a.y);
    TEST_ASSERT_EQUAL(0.0F, a.z);

    a.setOnes();
    TEST_ASSERT_EQUAL(1.0F, a.x);
    TEST_ASSERT_EQUAL(1.0F, a.y);
    TEST_ASSERT_EQUAL(1.0F, a.z);

    a.setConstant(-0.7F);
    TEST_ASSERT_EQUAL(-0.7F, a.x);
    TEST_ASSERT_EQUAL(-0.7F, a.y);
    TEST_ASSERT_EQUAL(-0.7F, a.z);

    a.setZero();
    TEST_ASSERT_EQUAL(0.0F, a.x);
    TEST_ASSERT_EQUAL(0.0F, a.y);
    TEST_ASSERT_EQUAL(0.0F, a.z);
}

void test_xyz_t_operators()
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
}

void test_xyz_t_functions()
{
    const xyz_t a{2, 3, 5};
    const xyz_t b{7, 11, 17};

    TEST_ASSERT_EQUAL_FLOAT(38, a.magnitudeSquared());
    TEST_ASSERT_EQUAL_FLOAT(sqrtf(38.0F), a.magnitude());

    const xyz_t aNE{2/sqrt(38.0F), 3.0F/sqrt(38.0F), 5.0F/sqrt(38.0F)};
    const xyz_t aN = a.normalized();
    TEST_ASSERT_EQUAL_FLOAT(1.0F, aN.magnitude());
    TEST_ASSERT_EQUAL_FLOAT(aNE.x, aN.x);
    TEST_ASSERT_EQUAL_FLOAT(aNE.y, aN.y);
    TEST_ASSERT_EQUAL_FLOAT(aNE.z, aN.z);

    xyz_t aN2 = a;
    aN2.normalizeInPlace();
    TEST_ASSERT_EQUAL_FLOAT(1.0F, aN2.magnitude());
    TEST_ASSERT_EQUAL_FLOAT(aNE.x, aN2.x);
    TEST_ASSERT_EQUAL_FLOAT(aNE.y, aN2.y);
    TEST_ASSERT_EQUAL_FLOAT(aNE.z, aN2.z);

    const float a_dot_b =  14 + 33 + 85;
    TEST_ASSERT_TRUE(a_dot_b == a.dot(b));
    TEST_ASSERT_TRUE(a.magnitudeSquared() == a.dot(a));
    TEST_ASSERT_TRUE(b.magnitudeSquared() == b.dot(b));

    const xyz_t a_cross_b = a.cross(b);
    TEST_ASSERT_EQUAL(3*17 - 5*11, a_cross_b.x);
    TEST_ASSERT_EQUAL(-2*17 + 5*7, a_cross_b.y);
    TEST_ASSERT_EQUAL(2*11 - 3*7, a_cross_b.z);

    TEST_ASSERT_TRUE(a_cross_b != b.cross(a));

    const xyz_t d{-100, 200, 50};
    const xyz_t d_clamped = xyz_t{-50, 50, 50};
    TEST_ASSERT_TRUE(xyz_t::clamp(d, -50, 50) == d_clamped);

    const xyz_t e{-100, 200, 50};
    const xyz_t e_clamped = xyz_t{-20, 20, 20};
    TEST_ASSERT_TRUE(xyz_t::clamp(e, -20, 20) == e_clamped);

    xyz_t f{-100, 200, 50};
    f.clampInPlace(-20, 20);
    const xyz_t f_clamped = xyz_t{-20, 20, 20};
    TEST_ASSERT_TRUE(f == f_clamped);
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();

    RUN_TEST(test_xyz_t_assignment);
    RUN_TEST(test_xyz_t_operators);
    RUN_TEST(test_xyz_t_functions);

    UNITY_END();
}
