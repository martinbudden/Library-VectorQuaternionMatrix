#include "Matrix3x3.h"
#include <unity.h>

void setUp() {
}

void tearDown() {
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)
void test_readme()
{
    // test to check code example in readme.md compiles


    const xyz_t a{1.0F, 2.0F, 3.0F};
    const xyz_t b{5.0F, 7.0F, 11.0F};

    // vector arithmetic
    const xyz_t c = a + b;
    xyz_t d = (a - b)*2.0F;
    d += a;
    d = c - d;

    // vector dot and cross product
    const float dotProduct = a.dot(b);
    const xyz_t crossProduct = a.cross(b);

    const Matrix3x3 M( 2,  3,  5,
                       7, 11, 13,
                      17, 19, 23);
    const Matrix3x3 N(29, 31, 37,
                      41, 43, 47,
                      53, 59, 61);

    // Matrix arithmetic
    Matrix3x3 P = M*N;
    P += M;
    P *= 2;
    P = P + N*M;

    // Multiplication of a vector by a matrix
    const xyz_t v = P*a;

    const Quaternion q{2, 3, 5, 7};
    const Quaternion r{11, 13, 17, 23};

    // quaternion arithmetic
    const Quaternion s = q + r;
    Quaternion t = (s - q)*2.0F;
    t += s;
    t = s - t;
    t = s * t;


    // dummy tests to avoid -Werror=unused-but-set-variable
    const xyz_t z = {0.0F, 0.0F, 0.0F};
    const Matrix3x3 Z(0.0F);
    TEST_ASSERT_FALSE(d == z);
    TEST_ASSERT_FALSE(dotProduct == 0);
    TEST_ASSERT_FALSE(crossProduct == z);
    TEST_ASSERT_FALSE(P == Z);
    TEST_ASSERT_FALSE(v == z);
    TEST_ASSERT_TRUE(t != s);
}

void test_matrix3x3_unary()
{
    // 0 1 2
    // 3 4 5
    // 6 7 8

    std::array<float, 9> z;
    z.fill(0.0F);
    const Matrix3x3 Z(z);
    const float f[9] = {  2,  3,  5,  7, 11, 13, 17, 19, 23 }; // NOLINT(cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)
    const Matrix3x3 F(f); // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay,hicpp-no-array-decay)
    TEST_ASSERT_EQUAL_FLOAT(2.0F, F[0]);
    TEST_ASSERT_EQUAL_FLOAT(3.0F, F[1]);
    TEST_ASSERT_EQUAL_FLOAT(23.0F, F[8]);


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

    const Matrix3x3 D(29, 0, 0, 0, 31, 0, 0, 0, 37);
    Matrix3x3 D_inv = D;
    D_inv.invertInPlaceAssumingDiagonal();
    TEST_ASSERT_EQUAL_FLOAT(1.0F / 29.0F, D_inv[0]);
    TEST_ASSERT_EQUAL_FLOAT(1.0F / 31.0F, D_inv[4]);
    TEST_ASSERT_EQUAL_FLOAT(1.0F / 37.0F, D_inv[8]);

    const Matrix3x3 D_times_D_inv = D * D_inv;
    TEST_ASSERT_EQUAL_FLOAT(1.0F, D_times_D_inv[0]);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, D_times_D_inv[4]);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, D_times_D_inv[8]);

    TEST_ASSERT_EQUAL_FLOAT(0.0F, D_times_D_inv[1]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, D_times_D_inv[2]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, D_times_D_inv[3]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, D_times_D_inv[5]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, D_times_D_inv[6]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, D_times_D_inv[7]);

    const Matrix3x3 D_inv2 = D.inverseAssumingDiagonal();
    TEST_ASSERT_EQUAL_FLOAT(1.0F / 29.0F, D_inv2[0]);
    TEST_ASSERT_EQUAL_FLOAT(1.0F / 31.0F, D_inv2[4]);
    TEST_ASSERT_EQUAL_FLOAT(1.0F / 37.0F, D_inv2[8]);
    TEST_ASSERT_TRUE(D_inv == D_inv2);

    const xyz_t v{2, 3, 5};
    const Matrix3x3 DplusV = D.addToDiagonal(v);
    TEST_ASSERT_EQUAL_FLOAT(31, DplusV[0]);
    TEST_ASSERT_EQUAL_FLOAT(34, DplusV[4]);
    TEST_ASSERT_EQUAL_FLOAT(42, DplusV[8]);
    Matrix3x3 DplusVinPlace = D; // NOLINT(misc-const-correctness) false positive
    DplusVinPlace.addToDiagonalInPlace(v);
    TEST_ASSERT_TRUE(DplusV == DplusVinPlace);

    const Matrix3x3 DminusV = D.subtractFromDiagonal(v);
    TEST_ASSERT_EQUAL_FLOAT(27, DminusV[0]);
    TEST_ASSERT_EQUAL_FLOAT(28, DminusV[4]);
    TEST_ASSERT_EQUAL_FLOAT(32, DminusV[8]);
    Matrix3x3 DminusVinPlace = D; // NOLINT(misc-const-correctness) false positive
    DminusVinPlace.subtractFromDiagonalInPlace(v);
    TEST_ASSERT_TRUE(DminusV == DminusVinPlace);

    const Matrix3x3 DminusVplusV = DminusV.addToDiagonal(v);
    TEST_ASSERT_TRUE(D == DminusVplusV);

    const Matrix3x3 E(2, 0, 0, 0, 3, 0, 0, 0, 5);
    const Matrix3x3 DtimesE(29*2, 0, 0, 0, 31*3, 0, 0, 0, 37*5);
    TEST_ASSERT_TRUE(DtimesE == D.multiplyAssumingDiagonal(E));

    Matrix3x3 DtimesEinPlace = D;
    DtimesEinPlace.multiplyAssumingDiagonalInPlace(E);
    TEST_ASSERT_TRUE(DtimesE == DtimesEinPlace);
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

    const Matrix3x3 D(67, 0, 0, 0, 71, 0, 0, 0, 73);
    const Matrix3x3 DA = D * A;
    TEST_ASSERT_EQUAL_FLOAT(134.0F , DA[0]);
    TEST_ASSERT_EQUAL_FLOAT(201.0F , DA[1]);
    TEST_ASSERT_EQUAL_FLOAT(335.0F , DA[2]);
    TEST_ASSERT_EQUAL_FLOAT(497.0F , DA[3]);
    TEST_ASSERT_EQUAL_FLOAT(781.0F , DA[4]);
    TEST_ASSERT_EQUAL_FLOAT(923.0F , DA[5]);
    TEST_ASSERT_EQUAL_FLOAT(1241.0F , DA[6]);
    TEST_ASSERT_EQUAL_FLOAT(1387.0F , DA[7]);
    TEST_ASSERT_EQUAL_FLOAT(1679.0F , DA[8]);
}

constexpr float degrees19inRadians = 19.0F * Quaternion::degreesToRadians;
constexpr float degrees43inRadians = 43.0F * Quaternion::degreesToRadians;
constexpr float degrees45inRadians = 45.0F * Quaternion::degreesToRadians;
constexpr float degrees67inRadians = 67.0F * Quaternion::degreesToRadians;

void test_matrix3x3_quaternion()
{
    const Quaternion q0 = Quaternion::fromEulerAnglesRadians(degrees19inRadians, degrees43inRadians, degrees67inRadians);
    TEST_ASSERT_EQUAL_FLOAT( 0.7986084F, q0.getW());
    TEST_ASSERT_EQUAL_FLOAT(-0.07145718F, q0.getX());
    TEST_ASSERT_EQUAL_FLOAT( 0.386186F, q0.getY());
    TEST_ASSERT_EQUAL_FLOAT( 0.4560472F, q0.getZ());
    const Matrix3x3 m0 = Matrix3x3(q0);
    // test uses w-form
    TEST_ASSERT_FALSE(m0[8] < 0.0F);
    TEST_ASSERT_FALSE(m0[0] < -m0[4]);
    const Quaternion m0q = m0.quaternion();
#if defined(LIBRARY_VECTOR_QUATERNION_MATRIX_USE_FAST_RECIPROCAL_SQUARE_ROOT)
    TEST_ASSERT_FLOAT_WITHIN(0.00047, q0.getW(), m0q.getW());
    TEST_ASSERT_FLOAT_WITHIN(0.00021, q0.getX(), m0q.getX());
#else
    TEST_ASSERT_EQUAL_FLOAT(q0.getW(), m0q.getW());
    TEST_ASSERT_EQUAL_FLOAT(q0.getX(), m0q.getX());
    TEST_ASSERT_EQUAL_FLOAT(q0.getY(), m0q.getY());
    TEST_ASSERT_EQUAL_FLOAT(q0.getZ(), m0q.getZ());
#endif

    const Quaternion qX(0.25F, sqrtf(50.0F/64.0F), 0.125F, 0.375F);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, qX.magnitude());
    const Matrix3x3 mX = Matrix3x3(qX);
    // check conversion uses x-form
    TEST_ASSERT_EQUAL_FLOAT(-0.59375F, mX[8]);
    TEST_ASSERT_TRUE(mX[8] < 0.0F);
    TEST_ASSERT_TRUE(mX[0] > mX[4]);
    const Quaternion mXq = mX.quaternion();
    TEST_ASSERT_EQUAL_FLOAT(qX.getW(), mXq.getW());
    TEST_ASSERT_EQUAL_FLOAT(qX.getX(), mXq.getX());
    TEST_ASSERT_EQUAL_FLOAT(qX.getY(), mXq.getY());
    TEST_ASSERT_EQUAL_FLOAT(qX.getZ(), mXq.getZ());

    const Quaternion qY(0.25F, 0.125F, sqrtf(50.0F/64.0F), 0.375F);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, qY.magnitude());
    const Matrix3x3 mY = Matrix3x3(qY);
    // check conversion uses y-form
    TEST_ASSERT_EQUAL_FLOAT(-0.59375F, mY[8]);
    TEST_ASSERT_TRUE(mY[8] < 0.0F);
    TEST_ASSERT_FALSE(mY[0] > mY[4]);
    const Quaternion mYq = mY.quaternion();
    TEST_ASSERT_EQUAL_FLOAT(qY.getW(), mYq.getW());
    TEST_ASSERT_EQUAL_FLOAT(qY.getX(), mYq.getX());
    TEST_ASSERT_EQUAL_FLOAT(qY.getY(), mYq.getY());
    TEST_ASSERT_EQUAL_FLOAT(qY.getZ(), mYq.getZ());

    const Quaternion qZ(0.25F, 0.125F, 0.375F, sqrtf(50.0F/64.0F));
    TEST_ASSERT_EQUAL_FLOAT(1.0F, qZ.magnitude());
    const Matrix3x3 mZ = Matrix3x3(qZ);
    // check conversion uses z-form
    TEST_ASSERT_EQUAL_FLOAT(0.6875F, mZ[8]);
    TEST_ASSERT_FALSE(mZ[8] < 0.0F);
    TEST_ASSERT_TRUE(mZ[0] < -mZ[4]);
    const Quaternion mZq = mZ.quaternion();
    TEST_ASSERT_EQUAL_FLOAT(qZ.getW(), mZq.getW());
    TEST_ASSERT_EQUAL_FLOAT(qZ.getX(), mZq.getX());
    TEST_ASSERT_EQUAL_FLOAT(qZ.getY(), mZq.getY());
    TEST_ASSERT_EQUAL_FLOAT(qZ.getZ(), mZq.getZ());

    const Quaternion qW(sqrtf(50.0F/64.0F), 0.25F, 0.125F, 0.375F);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, qW.magnitude());
    const Matrix3x3 mW = Matrix3x3(qW);
    // check conversion uses w-form
    TEST_ASSERT_EQUAL_FLOAT(0.84375F, mW[8]);
    TEST_ASSERT_FALSE(mW[8] < 0.0F);
    TEST_ASSERT_FALSE(mW[0] < -mW[4]);
    const Quaternion mWq = mW.quaternion();
    TEST_ASSERT_EQUAL_FLOAT(qW.getW(), mWq.getW());
    TEST_ASSERT_EQUAL_FLOAT(qW.getX(), mWq.getX());
    TEST_ASSERT_EQUAL_FLOAT(qW.getY(), mWq.getY());
    TEST_ASSERT_EQUAL_FLOAT(qW.getZ(), mWq.getZ());
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_readme);
    RUN_TEST(test_matrix3x3_unary);
    RUN_TEST(test_matrix3x3_binary);
    RUN_TEST(test_matrix3x3_quaternion);

    UNITY_END();
}
