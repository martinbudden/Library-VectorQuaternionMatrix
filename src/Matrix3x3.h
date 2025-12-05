#pragma once

#include "Quaternion.h"
#include <array>
#include <limits>

class Matrix3x3 {
public:
    inline Matrix3x3() {}
    inline explicit Matrix3x3(float diagonal) { _a.fill(0.0F); _a[0] = diagonal; _a[4] = diagonal; _a[8] = diagonal; }
    inline Matrix3x3(float d0, float d1, float d2) { _a.fill(0.0F); _a[0] = d0; _a[4] = d1; _a[8] = d2; } //<! Set the matrix diagonal
    inline explicit Matrix3x3(const std::array<float, 9>& a) : _a(a) {}
    inline explicit Matrix3x3(const float a[9]) : _a({{ a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8] }}) {}
    inline Matrix3x3(float a0, float a1, float a2, float a3, float a4, float a5, float a6, float a7, float a8) : _a({{ a0, a1, a2, a3, a4, a5, a6, a7, a8 }}) {}
    //! Create rotation matrix from quaternion
    inline explicit Matrix3x3(const Quaternion& q) {
        const float w = q.getW();
        const float x = q.getX();
        const float y = q.getY();
        const float z = q.getZ();
        // see [Quaternion-derived rotation matrix](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Quaternion-derived_rotation_matrix), uses Hamilton convention
        _a[0] = 1.0F - 2.0F*(y*y + z*z);
        _a[1] = 2.0F*(x*y - w*z);
        _a[2] = 2.0F*(w*y + x*z);
        _a[3] = 2.0F*(w*z + x*y);
        _a[4] = 1.0F - 2.0F*(x*x + z*z);
        _a[5] = 2.0F*(y*z - w*x);
        _a[6] = 2.0F*(x*z - w*y);
        _a[7] = 2.0F*(w*x + y*z);
        _a[8] = 1.0F - 2.0F*(x*x + y*y);
    }
    static Matrix3x3 fromEulerAnglesRadians(float rollRadians, float pitchRadians, float yawRadians);
    static Matrix3x3 fromEulerAnglesDegrees(float rollDegrees, float pitchDegrees, float yawDegrees);
public:
    inline float operator[](size_t pos) const { return _a[pos]; } //<! Index operator
    inline float& operator[](size_t pos) { return _a[pos]; } //<! Index operator

    // Equality operators
    inline bool operator!=(const Matrix3x3& m) const { for (size_t ii = 0; ii < _a.size(); ++ii) { if (_a[ii] != m[ii]) {return true;} } return false; } //<! Inequality operator
    inline bool operator==(const Matrix3x3& m) const { return !operator!=(m); } //<! Equality operator

    // Unary operations
    inline Matrix3x3 operator+() const { return *this; } //<! Unary plus
    inline Matrix3x3 operator-() const { Matrix3x3 m; for (size_t ii = 0; ii < _a.size(); ++ii) {m[ii] = -_a[ii];} return m; } //<! Unary negation

    // cppcheck-suppress useStlAlgorithm
    inline Matrix3x3 operator*=(float k) { for (float& a : _a) { a*=k; } return *this; } //<! Multiplication by a scalar
    inline Matrix3x3 operator/=(float k) { const float r = 1.0F/k; return operator*=(r); } //<! Division by a scalar

    inline Matrix3x3 operator+=(const Matrix3x3& m) { for (size_t ii = 0; ii < _a.size(); ++ii) {_a[ii] += m[ii];} return *this; } //<! Unary addition
    inline Matrix3x3 operator-=(const Matrix3x3& m) { for (size_t ii = 0; ii < _a.size(); ++ii) {_a[ii] -= m[ii];} return *this; } //<! Unary subtraction
    //! Unary multiplication
    inline Matrix3x3 operator*=(const Matrix3x3& m) {
        std::array<float, 9> a {{
            _a[0]*m[0] + _a[1]*m[3] + _a[2]*m[6],
            _a[0]*m[1] + _a[1]*m[4] + _a[2]*m[7],
            _a[0]*m[2] + _a[1]*m[5] + _a[2]*m[8],
            _a[3]*m[0] + _a[4]*m[3] + _a[5]*m[6],
            _a[3]*m[1] + _a[4]*m[4] + _a[5]*m[7],
            _a[3]*m[2] + _a[4]*m[5] + _a[5]*m[8],
            _a[6]*m[0] + _a[7]*m[3] + _a[8]*m[6],
            _a[6]*m[1] + _a[7]*m[4] + _a[8]*m[7],
            _a[6]*m[2] + _a[7]*m[5] + _a[8]*m[8]
        }};
        _a = a;
        return *this;
    }

    // Binary operations
    inline Matrix3x3 operator*(float k) const { Matrix3x3 m; for (size_t ii = 0; ii < _a.size(); ++ii) { m[ii] = _a[ii]*k; } return m; } //<! Multiplication by a scalar
    inline friend Matrix3x3 operator*(float k, const Matrix3x3& m) { return m*k; } //<! Pre-multiplication by a scalar
    inline Matrix3x3 operator/(float k) const { const float r = 1.0F/k; return *this*r; } //<! Division by a scalar

    inline xyz_t operator*(const xyz_t& v) const { return xyz_t { _a[0]*v.x + _a[1]*v.y + _a[2]*v.z, _a[3]*v.x + _a[4]*v.y + _a[5]*v.z, _a[6]*v.x + _a[7]*v.y + _a[8]*v.z, }; } //<! Multiplication of a vector

    inline Matrix3x3 operator+(const Matrix3x3& m) const { Matrix3x3 ret; for (size_t ii = 0; ii < _a.size(); ++ii) { ret[ii] = _a[ii] + m[ii]; } return ret; } //<! Addition
    inline Matrix3x3 operator-(const Matrix3x3& m) const { Matrix3x3 ret; for (size_t ii = 0; ii < _a.size(); ++ii) { ret[ii] = _a[ii] - m[ii]; } return ret; } //<! Subtraction
    //! Multiplication
    inline Matrix3x3 operator*(const Matrix3x3& m) const {
        return Matrix3x3 (
            _a[0]*m[0] + _a[1]*m[3] + _a[2]*m[6],
            _a[0]*m[1] + _a[1]*m[4] + _a[2]*m[7],
            _a[0]*m[2] + _a[1]*m[5] + _a[2]*m[8],
            _a[3]*m[0] + _a[4]*m[3] + _a[5]*m[6],
            _a[3]*m[1] + _a[4]*m[4] + _a[5]*m[7],
            _a[3]*m[2] + _a[4]*m[5] + _a[5]*m[8],
            _a[6]*m[0] + _a[7]*m[3] + _a[8]*m[6],
            _a[6]*m[1] + _a[7]*m[4] + _a[8]*m[7],
            _a[6]*m[2] + _a[7]*m[5] + _a[8]*m[8]
        );
    }

    inline void addToDiagonalInPlace(const xyz_t& v) { _a[0]+=v.x; _a[4]+=v.y; _a[8]+=v.z; } //<! Add vector to diagonal of matrix, in-place
    inline void subtractFromDiagonalInPlace(const xyz_t& v) { _a[0]-=v.x, _a[4]-=v.y; _a[8]-=v.z; } //<! Subtract vector from diagonal of matrix, in-place
    //! Multiply by matrix, in-place, assuming both matrices are diagonal
     inline void multiplyAssumingDiagonalInPlace(const Matrix3x3& m) { _a[0]*=m[0]; _a[4]*=m[4]; _a[8]*=m[8]; }

    inline Matrix3x3 addToDiagonal(const xyz_t& v) const { return Matrix3x3 (_a[0]+v.x, _a[1], _a[2], _a[3], _a[4]+v.y, _a[5], _a[6], _a[7], _a[8]+v.z); } //<! Add vector to diagonal of matrix
    inline Matrix3x3 subtractFromDiagonal(const xyz_t& v) const { return Matrix3x3 (_a[0]-v.x, _a[1], _a[2], _a[3], _a[4]-v.y, _a[5], _a[6], _a[7], _a[8]-v.z); } //<! Subtract vector from diagonal of matrix
    //! Multiply by matrix, assuming both matrices are diagonal
    inline Matrix3x3 multiplyAssumingDiagonal(const Matrix3x3& m) const { return Matrix3x3 (_a[0]*m[0], 0.0F, 0.0F, 0.0F, _a[4]*m[4], 0.0F, 0.0F, 0.0F, _a[8]*m[8]); }

    inline void setToIdentity() { _a.fill(0.0F); _a[0] = 1.0F; _a[4] = 1.0F; _a[8] = 1.0F; } //<! Sets matrix to identity matrix
    inline void setToScaledIdentity(float d) { _a.fill(0.0F); _a[0] = d; _a[4] = d; _a[8] = d; } //<! Sets diagonal of matrix to d

    inline void transposeInPlace() { float t = _a[1]; _a[1]= _a[3]; _a[3] = t; t = _a[2]; _a[2]= _a[6]; _a[6] = t; t = _a[5]; _a[5]= _a[7]; _a[7] = t; } //<! Transposes matrix, in=place
    inline Matrix3x3 transpose() const { return Matrix3x3(_a[0], _a[3], _a[6], _a[1], _a[4], _a[7], _a[2], _a[5], _a[8]); } //<! Returns transpose of matrix

    //! Invert matrix, in-place
    inline bool invertInPlace() {
        // a b c
        // d e f
        // g h i
        const float A =   _a[4]*_a[8] - _a[5]*_a[7];  //  (e*i - f*h)
        const float B = -(_a[3]*_a[8] - _a[5]*_a[6]); // -(d*i - f*g)
        const float C =   _a[3]*_a[7] - _a[4]*_a[6];  //  (d*h - e*g)
        const float D = -(_a[1]*_a[8] - _a[2]*_a[7]); // -(b*i - c*h)
        const float E =   _a[0]*_a[8] - _a[2]*_a[6];  //  (a*i - c*g)
        const float F = -(_a[0]*_a[7] - _a[1]*_a[6]); // -(a*h - b*g)
        const float G =   _a[1]*_a[5] - _a[2]*_a[4];  //  (b*f - c*e)
        const float H = -(_a[0]*_a[5] - _a[2]*_a[3]); // -(a*f - c*d)
        const float I =   _a[0]*_a[4] - _a[1]*_a[3];  //  (a*e - b*d)

        const float det = _a[0]*A + _a[1]*B + _a[2]*C; // a*A + b*B + c*C;

        if ((fabsf(det) <= std::numeric_limits<float>::epsilon())) {
            return false;
        }

        // A D G
        // B E H
        // C F I
        _a[0] = A/det;
        _a[1] = D/det;
        _a[2] = G/det;
        _a[3] = B/det;
        _a[4] = E/det;
        _a[5] = H/det;
        _a[6] = C/det;
        _a[7] = F/det;
        _a[8] = I/det;

        return true;
    }
    inline Matrix3x3 inverse() const { Matrix3x3 ret = *this; (void)ret.invertInPlace(); return ret; } //<! Returns inverse of matrix

    inline void invertInPlaceAssumingDiagonal() { _a[0] = 1.0F / _a[0]; _a[4] = 1.0F / _a[4]; _a[8] = 1.0F / _a[8]; } //<! Invert matrix in-place, assuming it is a diagonal matrix
    inline Matrix3x3 inverseAssumingDiagonal() const { Matrix3x3 ret = *this; ret.invertInPlaceAssumingDiagonal(); return ret; } //<! Returns inverse of matrix, assuming it is diagonal

    inline float determinant() const { return _a[0]*(_a[4]*_a[8] - _a[5]*_a[7]) - _a[1]*(_a[3]*_a[8] - _a[5]*_a[6]) + _a[2]*(_a[3]*_a[7] - _a[4]*_a[6]); } //<! Matrix determinant

    Quaternion quaternion() const;
protected:
    std::array<float, 9> _a;
};
