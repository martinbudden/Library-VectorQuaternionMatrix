#include "Matrix3x3.h"
#include <unity.h>

void setUp() {
}

void tearDown() {
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)
void test_matrix3x3_unary()
{
    // 0 1 2
    // 3 4 5
    // 6 7 8

    std::array<float, 9> z;
    z.fill(0.0F);
    const Matrix3x3 Z(z);
    const float e[9] = {  2,  3,  5,  7, 11, 13, 17, 19, 23 }; // NOLINT(cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)
    const Matrix3x3 E(e); // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay,hicpp-no-array-decay)
    TEST_ASSERT_EQUAL_FLOAT(2.0F, E[0]);
    TEST_ASSERT_EQUAL_FLOAT(3.0F, E[1]);
    TEST_ASSERT_EQUAL_FLOAT(23.0F, E[8]);


    const Matrix3x3 I(1.0F);
    TEST_ASSERT_TRUE(Z == Z * I);

    TEST_ASSERT_TRUE(I == I); // NOLINT(misc-redundant-expression)
    TEST_ASSERT_FALSE(I != I); // NOLINT(misc-redundant-expression)
    TEST_ASSERT_TRUE(I == +I);
    TEST_ASSERT_FALSE(I == -I);

    TEST_ASSERT_EQUAL_FLOAT(1.0F, I[0]);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, I[4]);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, I[8]);

    TEST_ASSERT_EQUAL_FLOAT(0.0F, I[1]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, I[2]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, I[3]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, I[5]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, I[6]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, I[7]);

    Matrix3x3 IZ = Z;
    IZ.setToIdentity();
    TEST_ASSERT_TRUE(I == IZ);

    const Matrix3x3 D2(2.0F);
    const Matrix3x3 I2 = I * 2.0F;
    TEST_ASSERT_TRUE(D2 == I2);

    const Matrix3x3 I_NEG(-1.0F);
    TEST_ASSERT_TRUE(I_NEG == -I);

    //  2  3  5
    //  7 11 13
    // 17 19 23
    const Matrix3x3 A  ( 2,  3,  5,  7, 11, 13, 17, 19, 23);
    const Matrix3x3 ATE( 2,  7, 17,  3, 11, 19,  5, 13, 23);

    Matrix3x3 A_plus_A = A;
    A_plus_A += A;
    TEST_ASSERT_TRUE(A_plus_A == A*2.0F);
    TEST_ASSERT_TRUE(A_plus_A == A + A);

    TEST_ASSERT_TRUE(A == A * I);
    TEST_ASSERT_TRUE(A == I * A);

    const Matrix3x3 AT = A.transpose();
    TEST_ASSERT_TRUE(ATE == AT);
    const Matrix3x3 ATT = AT.transpose();
    TEST_ASSERT_TRUE(ATT == A);
    Matrix3x3 A2 = A;
    A2.transposeInPlace();
    TEST_ASSERT_TRUE(ATE == A2);

    const Matrix3x3 B(29, 31, 37, 41, 43, 47, 53, 59, 61);
    Matrix3x3 A_plus_B = A;
    A_plus_B += B;
    Matrix3x3 B_plus_A = B;
    B_plus_A += A;
    TEST_ASSERT_TRUE(A_plus_B == B_plus_A);
    TEST_ASSERT_TRUE(A_plus_B == A + B);
    TEST_ASSERT_TRUE(A_plus_B == B + A);

    Matrix3x3 A_inv = A;
    A_inv.invertInPlace();
    const Matrix3x3 A_times_A_inv = A * A_inv;
    TEST_ASSERT_EQUAL_FLOAT(1.0F, A_times_A_inv[0]);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, A_times_A_inv[4]);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, A_times_A_inv[8]);

    TEST_ASSERT_FLOAT_WITHIN(1e-07, 0.0F, A_times_A_inv[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-07, 0.0F, A_times_A_inv[2]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, A_times_A_inv[3]);
    TEST_ASSERT_FLOAT_WITHIN(1e-07, 0.0F, A_times_A_inv[5]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, A_times_A_inv[6]);
    TEST_ASSERT_FLOAT_WITHIN(5e-07, 0.0F, A_times_A_inv[7]);

    const Matrix3x3 A_inv2 = A.inverse();
    TEST_ASSERT_TRUE(A_inv == A_inv2);


    TEST_ASSERT_TRUE(A * B != B * A);

    Matrix3x3 A_times_B = A;
    A_times_B *= B;
    Matrix3x3 B_times_A = B;
    B_times_A *= A;
    TEST_ASSERT_TRUE(A_times_B == A * B);
    TEST_ASSERT_TRUE(B_times_A == B * A);

    // 0 1 2
    // 3 4 5
    // 6 7 8
    const float d = A.determinant();
    const float dE =  A[0]*(A[4]*A[8] - A[5]*A[7]) + A[1]*(A[5]*A[6] - A[3]*A[8]) + A[2]*(A[3]*A[7] - A[4]*A[6]);
    TEST_ASSERT_EQUAL_FLOAT(dE, d);
}

void test_matrix3x3_binary()
{
    const Matrix3x3 A(  2,  3,  5,
                        7, 11, 13,
                       17, 19, 23);
    const Matrix3x3 B(29, 31, 37,
                      41, 43, 47,
                      53, 59, 61);
    const Matrix3x3 A_times_B(
         2*29 +  3*41 +  5*53,  2*31 +  3*43 +  5*59,  2*37 +  3*47 +  5*61,
         7*29 + 11*41 + 13*53,  7*31 + 11*43 + 13*59,  7*37 + 11*47 + 13*61,
        17*29 + 19*41 + 23*53, 17*31 + 19*43 + 23*59, 17*37 + 19*47 + 23*61);
    TEST_ASSERT_TRUE(A_times_B == A * B);

    const Matrix3x3 A_plus_B (2+29, 3+31, 5+37, 7+41, 11+43, 13+47, 17+53, 19+59, 23+61);
    const Matrix3x3 A_minus_B(2-29, 3-31, 5-37, 7-41, 11-43, 13-47, 17-53, 19-59, 23-61);

    TEST_ASSERT_TRUE(A_plus_B == A + B);
    TEST_ASSERT_TRUE(A_plus_B == B + A);
    TEST_ASSERT_TRUE(A_minus_B == A - B);
    TEST_ASSERT_TRUE(-A_minus_B == B - A);

    const float k = 4.0F;
    const Matrix3x3 Ak(2*k, 3*k, 5*k, 7*k, 11*k, 13*k, 17*k, 19*k, 23*k);
    TEST_ASSERT_TRUE(Ak == A*k);
    TEST_ASSERT_TRUE(Ak == k*A);
    TEST_ASSERT_TRUE(A == Ak/k);
    Matrix3x3 Ake = A;
    Ake *= k;
    TEST_ASSERT_TRUE(Ak == Ake);
    Ake /= k;
    TEST_ASSERT_TRUE(A == Ake);

    const xyz_t v  = {29, 31, 37};
    const xyz_t Av = {2*29 + 3*31 +5*37, 7*29 + 11*31 +13*37, 17*29 + 19*31 + 23*37};
    TEST_ASSERT_TRUE(Av == A*v);
}

constexpr float degrees19inRadians = 19.0F * Quaternion::degreesToRadians;
constexpr float degrees43inRadians = 43.0F * Quaternion::degreesToRadians;
constexpr float degrees45inRadians = 45.0F * Quaternion::degreesToRadians;
constexpr float degrees67inRadians = 67.0F * Quaternion::degreesToRadians;

void test_matrix3x3_quaternion()
{
    const Quaternion q0 = Quaternion::fromEulerAnglesRadians(degrees19inRadians, degrees43inRadians, degrees67inRadians);
    const Matrix3x3 m0 = Matrix3x3(q0);
    const Quaternion m0q = m0.quaternion();
    TEST_ASSERT_EQUAL_FLOAT(q0.getW(), m0q.getW());
    TEST_ASSERT_EQUAL_FLOAT(q0.getX(), m0q.getX());
    TEST_ASSERT_EQUAL_FLOAT(q0.getY(), m0q.getY());
    TEST_ASSERT_EQUAL_FLOAT(q0.getZ(), m0q.getZ());
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_matrix3x3_unary);
    RUN_TEST(test_matrix3x3_binary);
    RUN_TEST(test_matrix3x3_quaternion);

    UNITY_END();
}
