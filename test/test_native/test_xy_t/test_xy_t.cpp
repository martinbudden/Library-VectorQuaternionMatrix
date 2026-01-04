#include "xy_type.h"
#include <unity.h>

void setUp() {
}

void tearDown() {
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)
void test_xy_t_assignment()
{
    xy_t a{2.0F, 3.0F};
    TEST_ASSERT_EQUAL(5.0F, a.sum());
    TEST_ASSERT_EQUAL(2.5F, a.mean());
    TEST_ASSERT_EQUAL(6.0F, a.prod());
    const xy_t b{7.0F, 11.0F};

    a = b;
    TEST_ASSERT_EQUAL(b.x, a.x);
    TEST_ASSERT_EQUAL(b.y, a.y);

    a = 17.0F;
    TEST_ASSERT_EQUAL(17.0F, a.x);
    TEST_ASSERT_EQUAL(17.0F, a.y);

    a = 0.0F;
    TEST_ASSERT_EQUAL(0.0F, a.x);
    TEST_ASSERT_EQUAL(0.0F, a.y);

    a.setOnes();
    TEST_ASSERT_EQUAL(1.0F, a.x);
    TEST_ASSERT_EQUAL(1.0F, a.y);

    a.setConstant(-0.7F);
    TEST_ASSERT_EQUAL(-0.7F, a.x);
    TEST_ASSERT_EQUAL(-0.7F, a.y);

    a.setZero();
    TEST_ASSERT_EQUAL(0.0F, a.x);
    TEST_ASSERT_EQUAL(0.0F, a.y);
}

void test_xy_t_operators()
{
    const xy_t a{2, 3};
    TEST_ASSERT_TRUE(a == +a);
    const xy_t minusA{-2, -3};
    TEST_ASSERT_TRUE(minusA == -a);

    const xy_t b{7, 11};
    TEST_ASSERT_TRUE(a != b);
    TEST_ASSERT_FALSE(a == b);

    const xy_t a2{4, 6};
    TEST_ASSERT_TRUE(a2 == a*2);
    TEST_ASSERT_TRUE(a2 == 2*a);

    xy_t a2dividedBy2 = a2;
    a2dividedBy2 /= 2;
    TEST_ASSERT_EQUAL_FLOAT(a.x, a2dividedBy2.x);
    TEST_ASSERT_EQUAL_FLOAT(a.y, a2dividedBy2.y);

    TEST_ASSERT_TRUE(a == a2/2);
    TEST_ASSERT_TRUE(a == a2dividedBy2);

    const xy_t b2{14, 22};
    TEST_ASSERT_TRUE(b2 == b*2);

    xy_t c = b;
    c *= 2;
    TEST_ASSERT_TRUE(c == b*2);

    const xy_t b2dividedBy2 = b2 / 2;
    TEST_ASSERT_EQUAL_FLOAT(b.x, b2dividedBy2.x);
    TEST_ASSERT_EQUAL_FLOAT(b.y, b2dividedBy2.y);

    TEST_ASSERT_TRUE(b == b2/2);
    TEST_ASSERT_TRUE(b == b2dividedBy2);


    const xy_t a_plus_b =  a + b;

    TEST_ASSERT_EQUAL(9, a_plus_b.x);
    TEST_ASSERT_EQUAL(14, a_plus_b.y);

    TEST_ASSERT_TRUE(a_plus_b == a + b);
    TEST_ASSERT_TRUE(a_plus_b == b + a);

    const xy_t a_minus_b =  a - b;

    TEST_ASSERT_EQUAL(-5, a_minus_b.x);
    TEST_ASSERT_EQUAL(-8, a_minus_b.y);
    TEST_ASSERT_TRUE(a_minus_b == a - b);
    TEST_ASSERT_TRUE(-a_minus_b == b - a);

    TEST_ASSERT_TRUE(a2 == a_plus_b + a_minus_b);
    TEST_ASSERT_TRUE(b2 == a_plus_b - a_minus_b);
}

void test_xy_t_functions()
{
    const xy_t a{2, 3};
    const xy_t b{7, 11};

    TEST_ASSERT_EQUAL_FLOAT(13, a.magnitudeSquared());
    TEST_ASSERT_EQUAL_FLOAT(sqrtf(13.0F), a.magnitude());

    const xy_t aNE{2/sqrt(13.0F), 3.0F/sqrt(13.0F)};
    const xy_t aN = a.normalized();
    TEST_ASSERT_EQUAL_FLOAT(1.0F, aN.magnitude());
    TEST_ASSERT_EQUAL_FLOAT(aNE.x, aN.x);
    TEST_ASSERT_EQUAL_FLOAT(aNE.y, aN.y);

    xy_t aN2 = a;
    aN2.normalizeInPlace();
    TEST_ASSERT_EQUAL_FLOAT(1.0F, aN2.magnitude());
    TEST_ASSERT_EQUAL_FLOAT(aNE.x, aN2.x);
    TEST_ASSERT_EQUAL_FLOAT(aNE.y, aN2.y);

    const float a_dot_b =  14 + 33;
    TEST_ASSERT_TRUE(a_dot_b == a.dot(b));
    TEST_ASSERT_TRUE(a.magnitudeSquared() == a.dot(a));
    TEST_ASSERT_TRUE(b.magnitudeSquared() == b.dot(b));

    const xy_t d{-100, 200};
    const xy_t d_clamped = xy_t{-50, 50};
    TEST_ASSERT_TRUE(xy_t::clamp(d, -50, 50) == d_clamped);

    const xy_t e{-100, 200};
    const xy_t e_clamped = xy_t{-20, 20};
    TEST_ASSERT_TRUE(xy_t::clamp(e, -20, 20) == e_clamped);

    xy_t f{-100, 200};
    f.clampInPlace(-20, 20);
    const xy_t f_clamped = xy_t{-20, 20};
    TEST_ASSERT_TRUE(f == f_clamped);
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();

    RUN_TEST(test_xy_t_assignment);
    RUN_TEST(test_xy_t_operators);
    RUN_TEST(test_xy_t_functions);

    UNITY_END();
}
