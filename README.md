![license](https://img.shields.io/badge/license-MIT-green) ![open source](https://badgen.net/badge/open/source/blue?icon=github)

# IMU_TYPES library

This library includes some basic types for use by Inertial Measurement Unit (IMU) and Attitude and Heading Reference Systems (AHRS), namely:

1. `xyz_t`, a vector of 3 `floats`, `{x, y, z}` (named `xyz_t` rather than (say) `vector_3d_t` to avoid confusion with the C++ `vector` class.
2. `Quaternion`, a basic [quaternion](https://en.wikipedia.org/wiki/Quaternion) class.
3. `Matrix3x3`, a basic 3x3 matrix class.

The library uses inlining, operator overloading, and return value optimization (RVO) to facilitate performant readable code.

Additionally care has been taken to avoid inadvertent double promotion so the code runs efficiently on microcontrollers  with single precision floating point coprocessors.

## Example code

```cpp
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

    // matrix arithmetic
    Matrix3x3 P = M*N;
    P += M;
    P *= 2;
    P = P + N*M;

    // multiplication of a vector by a matrix
    const xyz_t v = P*a;

    const Quaternion q{2, 3, 5, 7};
    const Quaternion r{11, 13, 17, 23};

    // quaternion arithmetic
    const Quaternion s = q + r;
    Quaternion t = (s - q)*2.0F;
    t += s;
    t = s - t;
    t = s * t;
```

See the [test code](test) for more examples.
