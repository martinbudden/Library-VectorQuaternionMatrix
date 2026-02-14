#include "xyz_type.h"
#include "Matrix3x3.h"
#include <unity.h>

void setUp() {
}

void tearDown() {
}
/*
xy_t calculateEigenvalues() const {
    const float d = discriminant();
    const float t = trace();
    return (d >= 0) ? xy_t{t + std::sqrt(d), t - std::sqrt(d)} : xy_t{std::nanf(""), std::nanf("")};
}
*/

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)
bool trilaterate(
    const xyz_t &p1, float r1,
    const xyz_t &p2, float r2,
    const xyz_t &p3, float r3,
    xyz_t &sol1, xyz_t &sol2)
{
    // Step 1: Create unit vectors in the trilateration coordinate system
    const xyz_t ex = (p2 - p1).normalized();
    const float i = ex.dot(p3 - p1);
    const xyz_t temp = p3 - p1 - i * ex;
    const xyz_t ey = temp.normalized();
    const xyz_t ez = ex.cross(ey);

    // Step 2: Distances between points
    const float d = (p2 - p1).norm();
    const float j = ey.dot(p3 - p1);

    // Step 3: Solve for x, y, z using least squares (handles small errors)
    const float x = (r1*r1 - r2*r2 + d*d) / (2*d);
    const float y = (r1*r1 - r3*r3 + i*i + j*j - 2*i*x) / (2*j);

    float z_sq = r1*r1 - x*x - y*y;
    if (z_sq < 0.0F) {
        // Due to measurement errors, z_sq might be slightly negative
        if (z_sq > -1e-8F) {
            z_sq = 0.0F; // Clamp small negatives to zero
        } else {
            return false; // No real intersection
        }
    }
    const float z = std::sqrt(z_sq);

    // Step 4: Convert back to original coordinates
    sol1 = p1 + x * ex + y * ey + z * ez;
    sol2 = p1 + x * ex + y * ey - z * ez;

    return true;
}

void test_trilaterate3()
{
    const xyz_t p1{0.0F, 0.0F, 0.0F};
    const xyz_t p2{4.0F, 0.0F, 0.0F};
    const xyz_t p3{0.0F, 4.0F, 0.0F};

    // Measured distances (with small errors)
    const float r1 = 3.0F;
    const float r2 = 3.1623F; // ~sqrt(10)
    const float r3 = 3.1623F; // ~sqrt(10)

    xyz_t sol1 {};
    xyz_t sol2 {};
    trilaterate(p1, r1, p2, r2, p3, r3, sol1, sol2);
    TEST_ASSERT_EQUAL_FLOAT(1.874982F, sol1.x);
    TEST_ASSERT_EQUAL_FLOAT(1.874982F, sol1.y);
    TEST_ASSERT_EQUAL_FLOAT(1.403169F, sol1.z);
    TEST_ASSERT_EQUAL_FLOAT(1.874982F, sol2.x);
    TEST_ASSERT_EQUAL_FLOAT(1.874982F, sol2.y);
    TEST_ASSERT_EQUAL_FLOAT(-1.403169F, sol2.z);
}
void test_trilaterate3b()
{
    const xyz_t p1{1.2F, -0.5F, 2.8F};
    const xyz_t p2{4.7F, 3.1F, -1.4F};
    const xyz_t p3{-2.3F, 2.9F, 4.5F};

    // Measured distances (with small errors)
    const float r1 = 5.4321F;
    const float r2 = 4.8765F;
    const float r3 = 6.1234F;

    xyz_t sol1 {};
    xyz_t sol2 {};
    trilaterate(p1, r1, p2, r2, p3, r3, sol1, sol2);
    const xyz_t e1{3.470161073409595F, 4.428932335099278F, 3.128747191390586F};
    //const xyz_t e2{-0.166832146567832F, 2.859067664900722F, -1.238747191390586F};
    TEST_ASSERT_EQUAL_FLOAT(e1.x , sol1.x);
    TEST_ASSERT_EQUAL_FLOAT(4.423984F , sol1.y);
    TEST_ASSERT_EQUAL_FLOAT(3.129328 , sol1.z);
    TEST_ASSERT_EQUAL_FLOAT(-0.1681367 , sol2.x);
    TEST_ASSERT_EQUAL_FLOAT(2.86344 , sol2.y);
    TEST_ASSERT_EQUAL_FLOAT(-1.240196 , sol2.z);
}

void test_trilaterate()
{
    const xyz_t P0 {0.0F, 0.0F, 0.0F};
    const float r0 = 7.1F;

    const xyz_t P1 {8.0F, 2.0F, 1.0F};
    const float r1 = 6.5F;

    const xyz_t P2 {1.0F, 7.0F, 3.0F};
    const float r2 = 5.9F;

    const xyz_t P3 {2.0F, 3.0F, 9.0F};
    const float r3 = 8.2F;

    const Matrix3x3 A(2*(P1 - P0), 2*(P2- P0), 2*(P3 -P0));
    // Matrix A:
    // Row 1 (P2): [16,  4,  2]
    // Row 2 (P3): [ 2, 14,  6]
    // Row 3 (P4): [ 4,  6, 18]

    TEST_ASSERT_EQUAL_FLOAT(16.0F, A[0]);
    TEST_ASSERT_EQUAL_FLOAT(4.0F, A[1]);
    TEST_ASSERT_EQUAL_FLOAT(2.0F, A[2]);
    TEST_ASSERT_EQUAL_FLOAT(2.0F, A[3]);
    TEST_ASSERT_EQUAL_FLOAT(14.0F, A[4]);
    TEST_ASSERT_EQUAL_FLOAT(6.0F, A[5]);
    TEST_ASSERT_EQUAL_FLOAT(4.0F, A[6]);
    TEST_ASSERT_EQUAL_FLOAT(6.0F, A[7]);
    TEST_ASSERT_EQUAL_FLOAT(18.0F, A[8]);
    // b = [ 77.16 ] [ 74.60 ] [ 77.17 ]
    // b[i-1] = (r₁² - rᵢ²) - (x₁² - xᵢ²) - (y₁² - yᵢ²) - (z₁² - zᵢ²)
    const xyz_t b {
        r0*r0 - r1*r1 - (P0.x*P0.x - P1.x*P1.x) - (P0.y*P0.y - P1.y*P1.y) - (P0.z*P0.z - P1.z*P1.z),
        r0*r0 - r2*r2 - (P0.x*P0.x - P2.x*P2.x) - (P0.y*P0.y - P2.y*P2.y) - (P0.z*P0.z - P2.z*P2.z),
        r0*r0 - r3*r3 - (P0.x*P0.x - P3.x*P3.x) - (P0.y*P0.y - P3.y*P3.y) - (P0.z*P0.z - P3.z*P3.z),
    };
    TEST_ASSERT_EQUAL_FLOAT(77.16F, b.x);
    TEST_ASSERT_EQUAL_FLOAT(74.60F, b.y);
    TEST_ASSERT_EQUAL_FLOAT(77.17F, b.z);

    // AᵀA =
    // [ 276  116  116 ]
    // [ 116  248  200 ]
    // [ 116  200  364 ]
    const Matrix3x3 AtA = A.transpose()*A;
    TEST_ASSERT_EQUAL_FLOAT(276.0F, AtA[0]);
    TEST_ASSERT_EQUAL_FLOAT(116.0F, AtA[1]);
    TEST_ASSERT_EQUAL_FLOAT(116.0F, AtA[2]);
    TEST_ASSERT_EQUAL_FLOAT(116.0F, AtA[3]);
    TEST_ASSERT_EQUAL_FLOAT(248.0F, AtA[4]);
    TEST_ASSERT_EQUAL_FLOAT(200.0F, AtA[5]);
    TEST_ASSERT_EQUAL_FLOAT(116.0F, AtA[6]);
    TEST_ASSERT_EQUAL_FLOAT(200.0F, AtA[7]);
    TEST_ASSERT_EQUAL_FLOAT(364.0F, AtA[8]);

    // Matrix V (orthonormal eigenvectors of AᵀA):
    /*const Matrix3x3 V {
        0.54F,   0.60F,   0.59F,
        0.50F,   0.50F,  -0.71F,
        0.68F,  -0.62F,   0.39F
    };*/
    const Matrix3x3 V {
        0.539163866017192F,   0.601500955007545F,   0.589548612479876F,
        0.500222991027446F,   0.500222991027446F,  -0.707106781186547F,
        0.677872379632464F,  -0.623489802531646F,   0.387907304067314F,
    };

    //Singular values σ: [ 22.3607, 15.4919, 12.1655 ]
    // σ₁ = 22.360679774997898
    // σ₂ = 15.491933384829668
    // σ₃ = 12.165525060596439
    const Matrix3x3 Sinverse {
        0.044721359549995794F,  0.0F,                   0.0F,
        0.0F,                   0.06453770853720169F,   0.0F,
        0.0F,                   0.0F,                   0.08218726670430199F
    };


    // A × V =
    // [ 11.983258579649784   10.377927639167212   7.380165283066456 ]
    // [ 12.148683884213412    4.465184969209458  -6.392954043510022 ]
    // [ 17.359696243617796   -5.815474679374772   5.097885236011874 ]
    const Matrix3x3 AV = A*V;
    TEST_ASSERT_EQUAL_FLOAT(11.983258579649784F, AV[0]);
    TEST_ASSERT_EQUAL_FLOAT(10.377927639167212F, AV[1]);
    TEST_ASSERT_EQUAL_FLOAT( 7.380165283066456F, AV[2]);
    TEST_ASSERT_EQUAL_FLOAT(12.148683884213412F, AV[3]);
    TEST_ASSERT_EQUAL_FLOAT( 4.465184969209458F, AV[4]);
    TEST_ASSERT_EQUAL_FLOAT(-6.392954043510022F, AV[5]);
    TEST_ASSERT_EQUAL_FLOAT(17.359696243617796F, AV[6]);
    TEST_ASSERT_EQUAL_FLOAT(-5.815474679374772F, AV[7]);
    TEST_ASSERT_EQUAL_FLOAT( 5.097885236011874F, AV[8]);

    //Step 7 — Compute U
    // U = A × V × S⁻¹
    const Matrix3x3 U = AV*Sinverse;
    // U =
    // [ 0.5360000000000000   0.6700000000000000   0.6050000000000000 ]
    // [ 0.5440000000000000   0.2880000000000000  -0.5260000000000000 ]
    //[ 0.7760000000000000  -0.3740000000000000   0.4180000000000000 ]
    TEST_ASSERT_EQUAL_FLOAT(11.983258579649784F * 0.044721359549995794F, U[0]);
    TEST_ASSERT_EQUAL_FLOAT(12.148683884213412F * 0.044721359549995794F, U[3]);
    TEST_ASSERT_EQUAL_FLOAT(17.359696243617796F * 0.044721359549995794F, U[6]);
    TEST_ASSERT_EQUAL_FLOAT(10.377927639167212F * 0.06453770853720169F, U[1]);
    TEST_ASSERT_EQUAL_FLOAT(4.465184969209458F * 0.06453770853720169F, U[4]);
    TEST_ASSERT_EQUAL_FLOAT(-5.815474679374772F * 0.06453770853720169F, U[7]);
    TEST_ASSERT_EQUAL_FLOAT(7.380165283066456F * 0.08218726670430199F, U[2]);
    TEST_ASSERT_EQUAL_FLOAT(-6.392954043510022F * 0.08218726670430199F, U[5]);
    TEST_ASSERT_EQUAL_FLOAT(5.097885236011874 * 0.08218726670430199, U[8]);

    //const xyz_t pos = V*Sinverse*(U.transpose()*b);

    const xyz_t Utb = U.transpose()*b;
    TEST_ASSERT_EQUAL_FLOAT(141.7921F, Utb.x);
    TEST_ASSERT_EQUAL_FLOAT(44.21372F, Utb.y);
    TEST_ASSERT_EQUAL_FLOAT(39.93832F, Utb.z);

    const xyz_t SinverseUtb = Sinverse*Utb;
    TEST_ASSERT_EQUAL_FLOAT(6.341136F, SinverseUtb.x);
    TEST_ASSERT_EQUAL_FLOAT(2.853452F, SinverseUtb.y);
    TEST_ASSERT_EQUAL_FLOAT(3.282421F, SinverseUtb.z);

    const xyz_t pos = V*SinverseUtb;
    // Final Estimated Position x ≈ (7.045, 2.308, 3.766)
    TEST_ASSERT_EQUAL_FLOAT(7.070413F, pos.x);
    TEST_ASSERT_EQUAL_FLOAT(2.278322F, pos.y);
    TEST_ASSERT_EQUAL_FLOAT(3.792658F, pos.z);

}

void test_trilaterate_simple()
{
    const xyz_t P0 {0.0F, 0.0F, 0.0F};
    const float r0 = 7.1F;

    const xyz_t P1 {8.0F, 2.0F, 1.0F};
    const float r1 = 6.5F;

    const xyz_t P2 {1.0F, 7.0F, 3.0F};
    const float r2 = 5.9F;

    const xyz_t P3 {2.0F, 3.0F, 9.0F};
    const float r3 = 8.2F;

    const Matrix3x3 A(2*(P1 - P0), 2*(P2- P0), 2*(P3 -P0));
    const xyz_t b {
        r0*r0 - r1*r1 - (P0.x*P0.x - P1.x*P1.x) - (P0.y*P0.y - P1.y*P1.y) - (P0.z*P0.z - P1.z*P1.z),
        r0*r0 - r2*r2 - (P0.x*P0.x - P2.x*P2.x) - (P0.y*P0.y - P2.y*P2.y) - (P0.z*P0.z - P2.z*P2.z),
        r0*r0 - r3*r3 - (P0.x*P0.x - P3.x*P3.x) - (P0.y*P0.y - P3.y*P3.y) - (P0.z*P0.z - P3.z*P3.z),
    };
    const Matrix3x3 At = A.transpose();
    const Matrix3x3 AtA = At*A;
    const xyz_t Atb = At*b;
    const xyz_t pos = AtA.inverse() * Atb;
    //const xyz_t pos = AtA.inverse() * At * b;
    TEST_ASSERT_EQUAL_FLOAT(3.578879F, pos.x);
    TEST_ASSERT_EQUAL_FLOAT(3.874229F, pos.y);
    TEST_ASSERT_EQUAL_FLOAT(2.200506F, pos.z);
}

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();

    RUN_TEST(test_trilaterate3);
    RUN_TEST(test_trilaterate3b);
    RUN_TEST(test_trilaterate);
    RUN_TEST(test_trilaterate_simple);

    UNITY_END();
}

