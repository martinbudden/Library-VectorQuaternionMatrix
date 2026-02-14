#pragma once

#include <cmath>

class FastTrigonometry {
private:
    // see [Optimized Trigonometric Functions on TI Arm Cores](https://www.ti.com/lit/an/sprad27a/sprad27a.pdf)
    // for explanation of range mapping and coefficients
    // r (remainder) is in range [-0.5, 0.5] and pre-scaled by 2/PI
    static inline float sin_poly5r(float r) {
        static constexpr float c1 =  1.57078719139F;
        static constexpr float c3 = -0.64568519592F;
        static constexpr float c5 =  0.077562883496F;
        const float r2 = r * r;
        return r*(c1 + r2*(c3 + r2*c5));
    }
    static inline float cos_poly6r(float r) {
        static constexpr float c2 = -1.23369765282F;
        static constexpr float c4 =  0.25360107422F;
        static constexpr float c6 = -0.020408373326F;
        const float r2 = r * r;
        return 1.0F + r2*(c2 + r2*(c4 + r2*c6));
    }
    // For sin/cos quadrant helper functions:
    // 2 least significant bits of q are quadrant index, ie [0, 1, 2, 3].
    static inline float sin_quadrant(float r, int q) {
        if (q & 1) {
            // odd quadrant: use cos
            const float c = cos_poly6r(r);
            return (q & 2) ? -c : c; // q=3 -cos, q=1 +cos
        }
        // even quadrant: use sin
        const float s = sin_poly5r(r);
        return (q & 2) ? -s : s; // q=4 -sin, q=2 +sin
    }
    static inline float cos_quadrant(float r, int q) {
        if (q & 1) {
            // odd quadrant: use sin
            const float s = sin_poly5r(r);
            return (q & 2) ? s : -s; // q=3 +sin, q=1 -sin
        }
        // even quadrant: use cos
        const float c = cos_poly6r(r);
        return (q & 2) ? -c : c; // q=4 -cos, q=2 +cos
    }
    static inline void sin_cos_quadrant(float r, int q, float& sin, float& cos) {
        const float sb = sin_poly5r(r);
        const float cb = cos_poly6r(r);

        // map values according to quadrant
        const float s = (q & 1) ?  cb : sb;
        const float c = (q & 1) ? -sb : cb;

        if (q & 2) { // negate for quadrants 2 and 3
            sin = -s;
            cos = -c;
        } else {
            sin = s;
            cos = c;
        }
    }
public:
    static inline float sin(float x) {
        const float t = x * TWO_OVER_PI;
        const float q = roundf(t);
        const float r = t - q;
        return sin_quadrant(r, static_cast<int>(q));
    }
    static inline float cos(float x) {
        const float t = x * TWO_OVER_PI;
        const float q = roundf(t);
        const float r = t - q;
        return cos_quadrant(r, static_cast<int>(q));
    }
    static inline void sin_cos(float x, float& sin, float& cos) {
        const float t = x * TWO_OVER_PI; // so remainder will be scaled from range [-PI/4, PI/4] ([-45, 45] degrees) to [-0.5, 0.5]
        const float q = roundf(t);       // nearest quadrant
        const float r = t - q;           // remainder in range [-0.5, 0.5]
        sin_cos_quadrant(r, static_cast<int>(q), sin, cos);
    }
public:
    static constexpr float M_PI_F = 3.141592653589793F;
    static constexpr float TWO_OVER_PI = 2.0F / M_PI_F;
};
