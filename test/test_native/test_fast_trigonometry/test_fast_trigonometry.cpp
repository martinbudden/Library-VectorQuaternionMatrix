#include <FastTrigonometry.h>
#include <cmath>

#include <unity.h>

void setUp() {
}

void tearDown() {
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)
void test_sin()
{
    constexpr float degreesToRadians { static_cast<float>(M_PI) / 180.0F };

    TEST_ASSERT_EQUAL_FLOAT(0.0F, FastTrigonometry::sin(0.0F));
    TEST_ASSERT_EQUAL_FLOAT(sinf(10.0F*degreesToRadians), FastTrigonometry::sin(10.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(sinf(20.0F*degreesToRadians), FastTrigonometry::sin(20.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(sinf(30.0F*degreesToRadians), FastTrigonometry::sin(30.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(sinf(40.0F*degreesToRadians), FastTrigonometry::sin(40.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(sinf(50.0F*degreesToRadians), FastTrigonometry::sin(50.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(sinf(60.0F*degreesToRadians), FastTrigonometry::sin(60.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(sinf(70.0F*degreesToRadians), FastTrigonometry::sin(70.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(sinf(80.0F*degreesToRadians), FastTrigonometry::sin(80.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(1.0F, FastTrigonometry::sin(90.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(sinf(100.0F*degreesToRadians), FastTrigonometry::sin(100.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(sinf(110.0F*degreesToRadians), FastTrigonometry::sin(110.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(sinf(120.0F*degreesToRadians), FastTrigonometry::sin(120.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(sinf(130.0F*degreesToRadians), FastTrigonometry::sin(130.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(sinf(140.0F*degreesToRadians), FastTrigonometry::sin(140.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(sinf(150.0F*degreesToRadians), FastTrigonometry::sin(150.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(sinf(160.0F*degreesToRadians), FastTrigonometry::sin(160.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(sinf(170.0F*degreesToRadians), FastTrigonometry::sin(170.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(0.0F, FastTrigonometry::sin(180.0F*degreesToRadians));

    TEST_ASSERT_EQUAL_FLOAT(sinf(-10.0F*degreesToRadians), FastTrigonometry::sin(-10.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(sinf(-90.0F*degreesToRadians), FastTrigonometry::sin(-90.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(sinf(-100.0F*degreesToRadians), FastTrigonometry::sin(-100.0F*degreesToRadians));
}

void test_cos()
{
    constexpr float degreesToRadians { static_cast<float>(M_PI) / 180.0F };

    TEST_ASSERT_EQUAL_FLOAT(1.0F, FastTrigonometry::cos(0.0F));
    TEST_ASSERT_EQUAL_FLOAT(cosf(10.0F*degreesToRadians), FastTrigonometry::cos(10.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(cosf(20.0F*degreesToRadians), FastTrigonometry::cos(20.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(cosf(30.0F*degreesToRadians), FastTrigonometry::cos(30.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(cosf(40.0F*degreesToRadians), FastTrigonometry::cos(40.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(cosf(50.0F*degreesToRadians), FastTrigonometry::cos(50.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(cosf(60.0F*degreesToRadians), FastTrigonometry::cos(60.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(cosf(70.0F*degreesToRadians), FastTrigonometry::cos(70.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(cosf(80.0F*degreesToRadians), FastTrigonometry::cos(80.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(0.0F,FastTrigonometry::cos(90.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(cosf(100.0F*degreesToRadians), FastTrigonometry::cos(100.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(cosf(110.0F*degreesToRadians), FastTrigonometry::cos(110.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(cosf(120.0F*degreesToRadians), FastTrigonometry::cos(120.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(cosf(130.0F*degreesToRadians), FastTrigonometry::cos(130.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(cosf(140.0F*degreesToRadians), FastTrigonometry::cos(140.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(cosf(150.0F*degreesToRadians), FastTrigonometry::cos(150.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(cosf(160.0F*degreesToRadians), FastTrigonometry::cos(160.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(cosf(170.0F*degreesToRadians), FastTrigonometry::cos(170.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(-1.0F, FastTrigonometry::cos(180.0F*degreesToRadians));

    TEST_ASSERT_EQUAL_FLOAT(cosf(-10.0F*degreesToRadians), FastTrigonometry::cos(-10.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(0.0F,FastTrigonometry::cos(-90.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(cosf(-100.0F*degreesToRadians), FastTrigonometry::cos(-100.0F*degreesToRadians));
}

void test_sincos()
{
    constexpr float degreesToRadians { static_cast<float>(M_PI) / 180.0F };

    float sin = 0.0F;
    float cos = 0.0F;

    FastTrigonometry::sincos(0.0F, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, sin);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, cos);

    FastTrigonometry::sincos(370.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(370.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(370.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(-370.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-370.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-370.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(10.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(10.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(10.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(20.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(20.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(20.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(30.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(30.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(30.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(40.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(40.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(40.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(50.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(50.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(50.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(60.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(60.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(60.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(70.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(70.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(70.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(80.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(80.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(80.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(90.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(90.0F*degreesToRadians), sin);
    TEST_ASSERT_FLOAT_WITHIN(4.4E-08, cosf(90.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(100.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(100.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(100.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(110.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(110.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(110.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(120.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(120.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(120.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(130.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(130.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(130.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(140.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(140.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(140.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(150.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(150.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(150.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(160.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(160.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(160.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(170.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(170.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(170.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(180.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, sin);
    TEST_ASSERT_EQUAL_FLOAT(-1.0F, cos);

    FastTrigonometry::sincos(-10.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-10.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-10.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(-20.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-20.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-20.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(-30.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-30.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-30.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(-40.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-40.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-40.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(-50.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-50.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-50.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(-60.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-60.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-60.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(-70.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-70.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-70.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(-80.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-80.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-80.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(-90.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-90.0F*degreesToRadians), sin);
    TEST_ASSERT_FLOAT_WITHIN(4.4E-08, cosf(-90.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(-100.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-100.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-100.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(-110.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-110.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-110.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(-120.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-120.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-120.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(-130.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-130.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-130.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(-140.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-140.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-140.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(-150.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-150.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-150.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(-160.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-160.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-160.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(-170.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-170.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-170.0F*degreesToRadians), cos);

    FastTrigonometry::sincos(180.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, sin);
    TEST_ASSERT_EQUAL_FLOAT(-1.0F, cos);
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_sin);
    RUN_TEST(test_cos);
    RUN_TEST(test_sincos);

    UNITY_END();
}
