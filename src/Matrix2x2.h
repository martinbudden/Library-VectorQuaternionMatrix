#pragma once

#include "xy_type.h"
#include <array>
#include <limits>

class Matrix2x2 {
public:
    Matrix2x2() { _a.fill(0.0F); }
    explicit Matrix2x2(float diagonal) { _a.fill(0.0F); _a[0] = diagonal; _a[3] = diagonal; }
    Matrix2x2(float d0, float d1) { _a.fill(0.0F); _a[0] = d0; _a[3] = d1; } //<! Set the matrix diagonal
    explicit Matrix2x2(const std::array<float, 4>& a) : _a(a) {}
    explicit Matrix2x2(const float a[4]) : _a({{ a[0], a[1], a[2], a[3] }}) {}
    Matrix2x2(float a0, float a1, float a2, float a3) : _a({{ a0, a1, a2, a3 }}) {}
    Matrix2x2(const xy_t& v0, const xy_t& v1) { _a[0] = v0.x; _a[1] = v0.y, _a[2] = v1.x; _a[3] = v1.y; }
public:
    // Equality operators
    bool operator!=(const Matrix2x2& m) const { for (size_t ii = 0; ii < _a.size(); ++ii) { if (_a[ii] != m[ii]) {return true;} } return false; } //<! Inequality operator
    bool operator==(const Matrix2x2& m) const { return !operator!=(m); } //<! Equality operator

    // Index operators
    float operator[](size_t pos) const { return _a[pos]; } //<! Index operator
    float& operator[](size_t pos) { return _a[pos]; } //<! Index operator

    // Unary operations
    Matrix2x2 operator+() const { return *this; } //<! Unary plus
    Matrix2x2 operator-() const { Matrix2x2 m; for (size_t ii = 0; ii < _a.size(); ++ii) {m[ii] = -_a[ii];} return m; } //<! Unary negation

    // cppcheck-suppress useStlAlgorithm
    Matrix2x2 operator*=(float k) { for (float& a : _a) { a*=k; } return *this; } //<! Multiplication by a scalar
    Matrix2x2 operator/=(float k) { const float r = 1.0F/k; return operator*=(r); } //<! Division by a scalar

    Matrix2x2 operator+=(const Matrix2x2& m) { for (size_t ii = 0; ii < _a.size(); ++ii) {_a[ii] += m[ii];} return *this; } //<! Unary addition
    Matrix2x2 operator-=(const Matrix2x2& m) { for (size_t ii = 0; ii < _a.size(); ++ii) {_a[ii] -= m[ii];} return *this; } //<! Unary subtraction
    //! Unary multiplication
    Matrix2x2 operator*=(const Matrix2x2& m) {
        std::array<float, 4> a {{
            _a[0]*m[0] + _a[1]*m[2],    _a[0]*m[1] + _a[1]*m[3],
            _a[2]*m[0] + _a[3]*m[2],    _a[2]*m[1] + _a[3]*m[3],
        }};
        _a = a;
        return *this;
    }

    // Binary operations
    Matrix2x2 operator*(float k) const { Matrix2x2 m; for (size_t ii = 0; ii < _a.size(); ++ii) { m[ii] = _a[ii]*k; } return m; } //<! Multiplication by a scalar
    friend Matrix2x2 operator*(float k, const Matrix2x2& m) { return m*k; } //<! Pre-multiplication by a scalar
    Matrix2x2 operator/(float k) const { const float r = 1.0F/k; return *this*r; } //<! Division by a scalar

    xy_t operator*(const xy_t& v) const { return xy_t { _a[0]*v.x + _a[1]*v.y, _a[2]*v.x + _a[3]*v.y }; } //<! Multiplication of a vector

    Matrix2x2 operator+(const Matrix2x2& m) const { Matrix2x2 ret; for (size_t ii = 0; ii < _a.size(); ++ii) { ret[ii] = _a[ii] + m[ii]; } return ret; } //<! Addition
    Matrix2x2 operator-(const Matrix2x2& m) const { Matrix2x2 ret; for (size_t ii = 0; ii < _a.size(); ++ii) { ret[ii] = _a[ii] - m[ii]; } return ret; } //<! Subtraction
    //! Multiplication
    Matrix2x2 operator*(const Matrix2x2& m) const {
        return Matrix2x2(
            _a[0]*m[0] + _a[1]*m[2],    _a[0]*m[1] + _a[1]*m[3],
            _a[2]*m[0] + _a[3]*m[2],    _a[2]*m[1] + _a[3]*m[3]
        );
    }

    void setZero() { _a.fill(0.0F); }
    void setOnes() { _a.fill(1.0F); }
    void setConstant(float value) { _a.fill(value); }

    void setToIdentity() { _a.fill(0.0F); _a[0] = 1.0F; _a[3] = 1.0F; } //<! Sets matrix to identity matrix
    void setToScaledIdentity(float d) { _a.fill(0.0F); _a[0] = d; _a[3] = d; } //<! Sets diagonal of matrix to d

    void setRow(size_t row, const xy_t& value) {
        if (row == 0) {
            _a[0] = value.x; _a[1] = value.y;
        } else {
            _a[2] = value.x; _a[3] = value.y;
        }
    }
    xy_t getRow(size_t row) { return (row == 0) ? xy_t{_a[0],_a[1]} : xy_t{_a[2],_a[3]}; }
    void setColumn(size_t column, const xy_t& value) {
        if (column == 0) {
            _a[0] = value.x; _a[2] = value.y;
        } else {
            _a[1] = value.x; _a[3] = value.y;
        }
    } 
    xy_t getColumn(size_t column) { return (column == 0) ? xy_t{_a[0],_a[2]} : xy_t{_a[1],_a[3]}; }

    void addToDiagonalInPlace(const xy_t& v) { _a[0]+=v.x; _a[3]+=v.y; } //<! Add vector to diagonal of matrix, in-place
    void subtractFromDiagonalInPlace(const xy_t& v) { _a[0]-=v.x, _a[3]-=v.y; } //<! Subtract vector from diagonal of matrix, in-place
    //! Multiply by matrix, in-place, assuming both matrices are diagonal
     void multiplyAssumingDiagonalInPlace(const Matrix2x2& m) { _a[0]*=m[0]; _a[3]*=m[3]; }

    Matrix2x2 addToDiagonal(const xy_t& v) const { return Matrix2x2 (_a[0]+v.x, _a[1], _a[2], _a[3]+v.y); } //<! Add vector to diagonal of matrix
    Matrix2x2 subtractFromDiagonal(const xy_t& v) const { return Matrix2x2 (_a[0]-v.x, _a[1], _a[2], _a[3]-v.y); } //<! Subtract vector from diagonal of matrix
    //! Multiply by matrix, assuming both matrices are diagonal
    Matrix2x2 multiplyAssumingDiagonal(const Matrix2x2& m) const { return Matrix2x2 (_a[0]*m[0], 0.0F, 0.0F, _a[3]*m[3]); }

    void transposeInPlace() { float t = _a[1]; _a[1]= _a[2]; _a[2] = t; } //<! Transposes matrix, in=place
    Matrix2x2 transpose() const { return Matrix2x2(_a[0], _a[2], _a[1], _a[3]); } //<! Returns transpose of matrix

    void adjointInPlace() { float t = _a[0]; _a[0] = _a[3], _a[1]= -_a[1]; _a[2] = -_a[2], _a[3] = t; } //<! Transposes matrix, in=place
    Matrix2x2 adjoint() const { return Matrix2x2(_a[3], -_a[1], -_a[2], _a[0]); } //<! Returns transpose of matrix

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

    float sum() const { return _a[0] + _a[1] + _a[2] + _a[3]; } 
    float mean() const { return sum()/4.0F; } 
    float prod() const { return _a[0]*_a[1]*_a[2]*_a[3]; } 
    float trace() const { return _a[0] + _a[3]; } 
    float discriminant() const { const float t = trace(); return t*t - 4.0F*determinant(); }
protected:
    std::array<float, 4> _a;
};
