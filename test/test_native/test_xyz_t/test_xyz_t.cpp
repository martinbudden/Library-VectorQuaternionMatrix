#include "Quaternion.h"
#include "xyz_type.h"
#include <unity.h>

void setUp() {
}

void tearDown() {
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)
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
    TEST_ASSERT_TRUE(a.magnitudeSquared() == a.dot_product(a));
    TEST_ASSERT_TRUE(b.magnitudeSquared() == b.dot_product(b));

    const xyz_t a_cross_b = a.cross_product(b);
    TEST_ASSERT_EQUAL(3*17 - 5*11, a_cross_b.x);
    TEST_ASSERT_EQUAL(-2*17 + 5*7, a_cross_b.y);
    TEST_ASSERT_EQUAL(2*11 - 3*7, a_cross_b.z);

    TEST_ASSERT_TRUE(a_cross_b != b.cross_product(a));

    const xyz_t d{-100, 200, 50};
    const xyz_t d_clipped = xyz_t{-50, 50, 50};
    TEST_ASSERT_TRUE(xyz_t::clip(d, -50, 50) == d_clipped);

    const xyz_t e{-100, 200, 50};
    const xyz_t e_clipped = xyz_t{-20, 20, 20};
    TEST_ASSERT_TRUE(xyz_t::clip(e, -20, 20) == e_clipped);

    xyz_t f{-100, 200, 50};
    f.clip(-20, 20);
    const xyz_t f_clipped = xyz_t{-20, 20, 20};
    TEST_ASSERT_TRUE(f == f_clipped);
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_xyz_t);

    UNITY_END();
}
