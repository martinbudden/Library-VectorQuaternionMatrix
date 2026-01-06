#include "Matrix2x2.h"
#include <unity.h>

void setUp() {
}

void tearDown() {
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)
void test_readme()
{
    // test to check code example in readme.md compiles

    const xy_t a{1.0F, 2.0F};
    const xy_t b{5.0F, 7.0F};

    // vector arithmetic
    const xy_t c = a + b;
    xy_t d = (a - b)*2.0F;
    d += a;
    d = c - d;

    // vector dot and cross product
    const float dotProduct = a.dot(b);

    const Matrix2x2 M( 2,  3,
                       7, 11);
    const Matrix2x2 N(29, 31,
                      41, 43);

    // Matrix arithmetic
    Matrix2x2 P = M*N;
    P += M;
    P *= 2;
    P = P + N*M;

    // Multiplication of a vector by a matrix
    const xy_t v = P*a;

    // dummy tests to avoid -Werror=unused-but-set-variable
    const xy_t z = {0.0F, 0.0F};
    const Matrix2x2 Z(0.0F);
    TEST_ASSERT_FALSE(d == z);
    TEST_ASSERT_FALSE(dotProduct == 0);
    TEST_ASSERT_FALSE(P == Z);
    TEST_ASSERT_FALSE(v == z);
}

void test_matrix2x2_constructors()
{
    const Matrix2x2 A = {  2,  3,  5,  7 };
    const Matrix2x2 B(  2,  3,  5,  7 );
    const std::array<float, 4> a = {  2,  3,  5,  7 };
    const Matrix2x2 C = Matrix2x2(a);
    const Matrix2x2 D(std::array<float, 4>({  2,  3,  5,  7 }));
    const float b[4] = { 2, 3, 5, 7 }; // NOLINT(cppcoreguidelines-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)
    const Matrix2x2 E(&b[0]);
    const Matrix2x2 F = A;
    const Matrix2x2 G(A);
    const Matrix2x2 H(xy_t{2,3}, xy_t{5,7});
    Matrix2x2 I;
    I.setRow(0, xy_t{2,3});
    I.setRow(1, xy_t{5,7});
    TEST_ASSERT_TRUE((xy_t{2,3} == I.getRow(0)));
    TEST_ASSERT_TRUE((xy_t{5,7} == I.getRow(1)));
    TEST_ASSERT_TRUE((xy_t{2,5} == I.getColumn(0)));
    TEST_ASSERT_TRUE((xy_t{3,7} == I.getColumn(1)));
    Matrix2x2 J;
    J.setColumn(0, xy_t{2,5});
    J.setColumn(1, xy_t{3,7});
    TEST_ASSERT_TRUE((xy_t{2,3} == J.getRow(0)));
    TEST_ASSERT_TRUE((xy_t{5,7} == J.getRow(1)));
    TEST_ASSERT_TRUE((xy_t{2,5} == J.getColumn(0)));
    TEST_ASSERT_TRUE((xy_t{3,7} == J.getColumn(1)));


    TEST_ASSERT_TRUE(A == A); // NOLINT(misc-redundant-expression)
    TEST_ASSERT_TRUE(A == B);
    TEST_ASSERT_TRUE(A == C);
    TEST_ASSERT_TRUE(A == D);
    TEST_ASSERT_TRUE(A == E);
    TEST_ASSERT_TRUE(A == F);
    TEST_ASSERT_TRUE(A == G);
    TEST_ASSERT_TRUE(A == H);
    TEST_ASSERT_TRUE(A == I);
    TEST_ASSERT_TRUE(A == J);

    Matrix2x2 M {};
    TEST_ASSERT_EQUAL(0.0F, M[0]);
    TEST_ASSERT_EQUAL(0.0F, M[1]);
    TEST_ASSERT_EQUAL(0.0F, M[2]);
    TEST_ASSERT_EQUAL(0.0F, M[3]);

    M.setOnes();
    TEST_ASSERT_EQUAL(1.0F, M[0]);
    TEST_ASSERT_EQUAL(1.0F, M[1]);
    TEST_ASSERT_EQUAL(1.0F, M[2]);
    TEST_ASSERT_EQUAL(1.0F, M[3]);

    M.setConstant(-0.7F);
    TEST_ASSERT_EQUAL(-0.7F, M[0]);
    TEST_ASSERT_EQUAL(-0.7F, M[1]);
    TEST_ASSERT_EQUAL(-0.7F, M[2]);
    TEST_ASSERT_EQUAL(-0.7F, M[3]);
}

void test_Matrix2x2_unary()
{
    // 0 1
    // 2 3

    std::array<float, 4> z;
    z.fill(0.0F);
    const Matrix2x2 Z(z);
    const float f[4] = {  2,  3,  5,  7 }; // NOLINT(cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)
    const Matrix2x2 F(f); // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay,hicpp-no-array-decay)
    TEST_ASSERT_EQUAL_FLOAT(2.0F, F[0]);
    TEST_ASSERT_EQUAL_FLOAT(3.0F, F[1]);
    TEST_ASSERT_EQUAL_FLOAT(7.0F, F[3]);


    const Matrix2x2 I(1.0F);
    TEST_ASSERT_TRUE(Z == Z * I);

    TEST_ASSERT_TRUE(I == I); // NOLINT(misc-redundant-expression)
    TEST_ASSERT_FALSE(I != I); // NOLINT(misc-redundant-expression)
    TEST_ASSERT_TRUE(I == +I);
    TEST_ASSERT_FALSE(I == -I);

    TEST_ASSERT_EQUAL_FLOAT(1.0F, I[0]);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, I[3]);

    TEST_ASSERT_EQUAL_FLOAT(0.0F, I[1]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, I[2]);

    Matrix2x2 IZ = Z;
    IZ.setToIdentity();
    TEST_ASSERT_TRUE(I == IZ);

    const Matrix2x2 D2(2.0F);
    const Matrix2x2 I2 = I * 2.0F;
    TEST_ASSERT_TRUE(D2 == I2);

    const Matrix2x2 I_NEG(-1.0F);
    TEST_ASSERT_TRUE(I_NEG == -I);

    //  2  3
    //  5  7
    const Matrix2x2 A  (2, 3, 5, 7);
    TEST_ASSERT_EQUAL_FLOAT(-1.0F, A.determinant());
    TEST_ASSERT_EQUAL_FLOAT(17.0F, A.sum());
    TEST_ASSERT_EQUAL_FLOAT(4.25F, A.mean());
    TEST_ASSERT_EQUAL_FLOAT(210.0F, A.prod());
    TEST_ASSERT_EQUAL_FLOAT(9.0F, A.trace());
    const Matrix2x2 ATE(2, 5, 3, 7);

    Matrix2x2 A_plus_A = A;
    A_plus_A += A;
    TEST_ASSERT_TRUE(A_plus_A == A*2.0F);
    TEST_ASSERT_TRUE(A_plus_A == A + A);

    TEST_ASSERT_TRUE(A == A * I);
    TEST_ASSERT_TRUE(A == I * A);

    const Matrix2x2 AT = A.transpose();
    TEST_ASSERT_TRUE(ATE == AT);
    const Matrix2x2 ATT = AT.transpose();
    TEST_ASSERT_TRUE(ATT == A);
    Matrix2x2 A2 = A;
    A2.transposeInPlace();
    TEST_ASSERT_TRUE(ATE == A2);

    const Matrix2x2 B(29, 31, 37, 41);
    Matrix2x2 A_plus_B = A;
    A_plus_B += B;
    Matrix2x2 B_plus_A = B;
    B_plus_A += A;
    TEST_ASSERT_TRUE(A_plus_B == B_plus_A);
    TEST_ASSERT_TRUE(A_plus_B == A + B);
    TEST_ASSERT_TRUE(A_plus_B == B + A);

    const Matrix2x2 C(29, -131, 537, -41);
    TEST_ASSERT_EQUAL(69158.0F, C.determinant());
    Matrix2x2 C_inv = C;

    C_inv.invertInPlace();
    TEST_ASSERT_EQUAL_FLOAT(-41.0F/69158.0F, C_inv[0]);
    TEST_ASSERT_EQUAL_FLOAT(131.0F/69158.0F, C_inv[1]);
    TEST_ASSERT_EQUAL_FLOAT(-537.0F/69158.0F, C_inv[2]);
    TEST_ASSERT_EQUAL_FLOAT(29.0F/69158.0F, C_inv[3]);
    const Matrix2x2 C_times_C_inv = C * C_inv;
    TEST_ASSERT_EQUAL_FLOAT(1.0F, C_times_C_inv[0]);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, C_times_C_inv[3]);

    TEST_ASSERT_FLOAT_WITHIN(1e-07, 0.0F, C_times_C_inv[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-07, 0.0F, C_times_C_inv[2]);

    const Matrix2x2 C_inv2 = C.inverse();
    TEST_ASSERT_TRUE(C_inv == C_inv2);

    Matrix2x2 AJ(2, 3, 4, 5);
    const Matrix2x2 AJ_adjoint(5, -3, -4, 2);
    TEST_ASSERT_TRUE(AJ_adjoint == AJ.adjoint());
    AJ.adjointInPlace();
    TEST_ASSERT_TRUE(AJ_adjoint == AJ);

    TEST_ASSERT_TRUE(A * B != B * A);

    Matrix2x2 A_times_B = A;
    A_times_B *= B;
    Matrix2x2 B_times_A = B;
    B_times_A *= A;
    TEST_ASSERT_TRUE(A_times_B == A * B);
    TEST_ASSERT_TRUE(B_times_A == B * A);

    const float d = A.determinant();
    const float dE =  A[0]*A[3] - A[1]*A[2];
    TEST_ASSERT_EQUAL_FLOAT(dE, d);

    const Matrix2x2 D(29, 0, 0, 31);
    Matrix2x2 D_inv = D;
    D_inv.invertInPlaceAssumingDiagonal();
    TEST_ASSERT_EQUAL_FLOAT(1.0F / 29.0F, D_inv[0]);
    TEST_ASSERT_EQUAL_FLOAT(1.0F / 31.0F, D_inv[3]);

    const Matrix2x2 D_times_D_inv = D * D_inv;
    TEST_ASSERT_EQUAL_FLOAT(1.0F, D_times_D_inv[0]);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, D_times_D_inv[3]);

    TEST_ASSERT_EQUAL_FLOAT(0.0F, D_times_D_inv[1]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, D_times_D_inv[2]);

    const Matrix2x2 D_inv2 = D.inverseAssumingDiagonal();
    TEST_ASSERT_EQUAL_FLOAT(1.0F / 29.0F, D_inv2[0]);
    TEST_ASSERT_EQUAL_FLOAT(1.0F / 31.0F, D_inv2[3]);
    TEST_ASSERT_TRUE(D_inv == D_inv2);

    const xy_t v{2, 3};
    const Matrix2x2 DplusV = D.addToDiagonal(v);
    TEST_ASSERT_EQUAL_FLOAT(31, DplusV[0]);
    TEST_ASSERT_EQUAL_FLOAT(34, DplusV[3]);
    Matrix2x2 DplusVinPlace = D; // NOLINT(misc-const-correctness) false positive
    DplusVinPlace.addToDiagonalInPlace(v);
    TEST_ASSERT_TRUE(DplusV == DplusVinPlace);

    const Matrix2x2 DminusV = D.subtractFromDiagonal(v);
    TEST_ASSERT_EQUAL_FLOAT(27, DminusV[0]);
    TEST_ASSERT_EQUAL_FLOAT(28, DminusV[3]);
    Matrix2x2 DminusVinPlace = D; // NOLINT(misc-const-correctness) false positive
    DminusVinPlace.subtractFromDiagonalInPlace(v);
    TEST_ASSERT_TRUE(DminusV == DminusVinPlace);

    const Matrix2x2 DminusVplusV = DminusV.addToDiagonal(v);
    TEST_ASSERT_TRUE(D == DminusVplusV);

    const Matrix2x2 E(2, 0, 0, 3);
    const Matrix2x2 DtimesE(29*2, 0, 0, 31*3);
    TEST_ASSERT_TRUE(DtimesE == D.multiplyAssumingDiagonal(E));

    Matrix2x2 DtimesEinPlace = D;
    DtimesEinPlace.multiplyAssumingDiagonalInPlace(E);
    TEST_ASSERT_TRUE(DtimesE == DtimesEinPlace);
}

void test_Matrix2x2_binary()
{
    const Matrix2x2 A(  2,  3,
                        5,  7);
    const Matrix2x2 B(29, 31,
                      37, 41);
    const Matrix2x2 A_times_B(
        2*29 +  3*37,   2*31 +  3*41,
        5*29 +  7*37, + 5*31 +  7*41);
    TEST_ASSERT_TRUE(A_times_B == A * B);

    const Matrix2x2 A_plus_B (2+29, 3+31, 5+37, 7+41);
    const Matrix2x2 A_minus_B(2-29, 3-31, 5-37, 7-41);

    TEST_ASSERT_TRUE(A_plus_B == A + B);
    TEST_ASSERT_TRUE(A_plus_B == B + A);
    TEST_ASSERT_TRUE(A_minus_B == A - B);
    TEST_ASSERT_TRUE(-A_minus_B == B - A);

    const float k = 4.0F;
    const Matrix2x2 Ak(2*k, 3*k, 5*k, 7*k);
    TEST_ASSERT_TRUE(Ak == A*k);
    TEST_ASSERT_TRUE(Ak == k*A);
    TEST_ASSERT_TRUE(A == Ak/k);
    Matrix2x2 Ake = A;
    Ake *= k;
    TEST_ASSERT_TRUE(Ak == Ake);
    Ake /= k;
    TEST_ASSERT_TRUE(A == Ake);

    const xy_t v  = {29, 31};
    const xy_t Av = {2*29 + 3*31, 5*29 + 7*31};
    TEST_ASSERT_TRUE(Av == A*v);

    //const Matrix2x2 A(  2,  3,
    //                    5,  7);
    const Matrix2x2 D(67,  0,
                       0, 71);
    const Matrix2x2 DA = D * A;
    TEST_ASSERT_EQUAL_FLOAT(2 * 67, DA[0]);
    TEST_ASSERT_EQUAL_FLOAT(3 * 67, DA[1]);
    TEST_ASSERT_EQUAL_FLOAT(5 * 71, DA[2]);
    TEST_ASSERT_EQUAL_FLOAT(7 * 71, DA[3]);
}

void test_Matrix2x2_Eigen_interworking()
{
    typedef Matrix2x2 Matrix2f;
    const Matrix2f A(  2,  3,
                        5,  7);
    const Matrix2f B(29, 31,
                      37, 41);
    const Matrix2f A_times_B(
        2*29 +  3*37,   2*31 +  3*41,
        5*29 +  7*37, + 5*31 +  7*41);
    TEST_ASSERT_TRUE(A_times_B == A * B);
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();

    RUN_TEST(test_readme);
    RUN_TEST(test_matrix2x2_constructors);
    RUN_TEST(test_Matrix2x2_unary);
    RUN_TEST(test_Matrix2x2_binary);
    RUN_TEST(test_Matrix2x2_Eigen_interworking);

    UNITY_END();
}
