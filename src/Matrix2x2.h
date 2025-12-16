#pragma once

#include "xy_type.h"
#include <array>
#include <limits>

class Matrix2x2 {
public:
    inline Matrix2x2() {}
    inline explicit Matrix2x2(float diagonal) { _a.fill(0.0F); _a[0] = diagonal; _a[3] = diagonal; }
    inline Matrix2x2(float d0, float d1) { _a.fill(0.0F); _a[0] = d0; _a[3] = d1; } //<! Set the matrix diagonal
    inline explicit Matrix2x2(const std::array<float, 4>& a) : _a(a) {}
    inline explicit Matrix2x2(const float a[4]) : _a({{ a[0], a[1], a[2], a[3] }}) {}
    inline Matrix2x2(float a0, float a1, float a2, float a3) : _a({{ a0, a1, a2, a3 }}) {}
public:
    inline float operator[](size_t pos) const { return _a[pos]; } //<! Index operator
    inline float& operator[](size_t pos) { return _a[pos]; } //<! Index operator

    // Equality operators
    inline bool operator!=(const Matrix2x2& m) const { for (size_t ii = 0; ii < _a.size(); ++ii) { if (_a[ii] != m[ii]) {return true;} } return false; } //<! Inequality operator
    inline bool operator==(const Matrix2x2& m) const { return !operator!=(m); } //<! Equality operator

    // Unary operations
    inline Matrix2x2 operator+() const { return *this; } //<! Unary plus
    inline Matrix2x2 operator-() const { Matrix2x2 m; for (size_t ii = 0; ii < _a.size(); ++ii) {m[ii] = -_a[ii];} return m; } //<! Unary negation

    // cppcheck-suppress useStlAlgorithm
    inline Matrix2x2 operator*=(float k) { for (float& a : _a) { a*=k; } return *this; } //<! Multiplication by a scalar
    inline Matrix2x2 operator/=(float k) { const float r = 1.0F/k; return operator*=(r); } //<! Division by a scalar

    inline Matrix2x2 operator+=(const Matrix2x2& m) { for (size_t ii = 0; ii < _a.size(); ++ii) {_a[ii] += m[ii];} return *this; } //<! Unary addition
    inline Matrix2x2 operator-=(const Matrix2x2& m) { for (size_t ii = 0; ii < _a.size(); ++ii) {_a[ii] -= m[ii];} return *this; } //<! Unary subtraction
    //! Unary multiplication
    Matrix2x2 operator*=(const Matrix2x2& m) {
        std::array<float, 4> a {{
            _a[0]*m[0] + _a[1]*m[2],
            _a[0]*m[1] + _a[1]*m[3],
            _a[2]*m[0] + _a[3]*m[2],
            _a[2]*m[1] + _a[3]*m[3],
        }};
        _a = a;
        return *this;
    }

    // Binary operations
    inline Matrix2x2 operator*(float k) const { Matrix2x2 m; for (size_t ii = 0; ii < _a.size(); ++ii) { m[ii] = _a[ii]*k; } return m; } //<! Multiplication by a scalar
    inline friend Matrix2x2 operator*(float k, const Matrix2x2& m) { return m*k; } //<! Pre-multiplication by a scalar
    inline Matrix2x2 operator/(float k) const { const float r = 1.0F/k; return *this*r; } //<! Division by a scalar

    inline xy_t operator*(const xy_t& v) const { return xy_t { _a[0]*v.x + _a[1]*v.y, _a[2]*v.x + _a[3]*v.y }; } //<! Multiplication of a vector

    inline Matrix2x2 operator+(const Matrix2x2& m) const { Matrix2x2 ret; for (size_t ii = 0; ii < _a.size(); ++ii) { ret[ii] = _a[ii] + m[ii]; } return ret; } //<! Addition
    inline Matrix2x2 operator-(const Matrix2x2& m) const { Matrix2x2 ret; for (size_t ii = 0; ii < _a.size(); ++ii) { ret[ii] = _a[ii] - m[ii]; } return ret; } //<! Subtraction
    //! Multiplication
    Matrix2x2 operator*(const Matrix2x2& m) const {
        return Matrix2x2(
            _a[0]*m[0] + _a[1]*m[2],
            _a[0]*m[1] + _a[1]*m[3],
            _a[2]*m[0] + _a[3]*m[2],
            _a[2]*m[1] + _a[3]*m[3]
        );
    }

    inline void addToDiagonalInPlace(const xy_t& v) { _a[0]+=v.x; _a[3]+=v.y; } //<! Add vector to diagonal of matrix, in-place
    inline void subtractFromDiagonalInPlace(const xy_t& v) { _a[0]-=v.x, _a[3]-=v.y; } //<! Subtract vector from diagonal of matrix, in-place
    //! Multiply by matrix, in-place, assuming both matrices are diagonal
     inline void multiplyAssumingDiagonalInPlace(const Matrix2x2& m) { _a[0]*=m[0]; _a[3]*=m[3]; }

    inline Matrix2x2 addToDiagonal(const xy_t& v) const { return Matrix2x2 (_a[0]+v.x, _a[1], _a[2], _a[3]+v.y); } //<! Add vector to diagonal of matrix
    inline Matrix2x2 subtractFromDiagonal(const xy_t& v) const { return Matrix2x2 (_a[0]-v.x, _a[1], _a[2], _a[3]-v.y); } //<! Subtract vector from diagonal of matrix
    //! Multiply by matrix, assuming both matrices are diagonal
    inline Matrix2x2 multiplyAssumingDiagonal(const Matrix2x2& m) const { return Matrix2x2 (_a[0]*m[0], 0.0F, 0.0F, _a[3]*m[3]); }

    inline void setToIdentity() { _a.fill(0.0F); _a[0] = 1.0F; _a[3] = 1.0F; } //<! Sets matrix to identity matrix
    inline void setToScaledIdentity(float d) { _a.fill(0.0F); _a[0] = d; _a[3] = d; } //<! Sets diagonal of matrix to d

    inline void transposeInPlace() { float t = _a[1]; _a[1]= _a[2]; _a[2] = t; } //<! Transposes matrix, in=place
    inline Matrix2x2 transpose() const { return Matrix2x2(_a[0], _a[2], _a[1], _a[3]); } //<! Returns transpose of matrix

    //! Invert matrix, in-place
    bool invertInPlace() {
        const float det = _a[0]*_a[3] -_a[1]*_a[2];
        if ((fabsf(det) <= std::numeric_limits<float>::epsilon())) {
            return false;
        }
        const float t = _a[0];
        _a[0] = _a[3]/det;
        _a[1] = -_a[1]/det;
        _a[2] = -_a[2]/det;
        _a[3] = t/det;
        return true;
    }
    Matrix2x2 inverse() const { Matrix2x2 ret = *this; (void)ret.invertInPlace(); return ret; } //<! Returns inverse of matrix

    void invertInPlaceAssumingDiagonal() { _a[0] = 1.0F / _a[0]; _a[3] = 1.0F / _a[3]; } //<! Invert matrix in-place, assuming it is a diagonal matrix
    Matrix2x2 inverseAssumingDiagonal() const { Matrix2x2 ret = *this; ret.invertInPlaceAssumingDiagonal(); return ret; } //<! Returns inverse of matrix, assuming it is diagonal

    float determinant() const { return _a[0]*_a[3] -_a[1]*_a[2]; } //<! Matrix determinant
protected:
    std::array<float, 4> _a;
};
