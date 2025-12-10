#pragma once

#include <cmath>

// Fast math that gathered from blog and sleef library

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_1_PI
#define M_1_PI 0.31830988618379067154
#endif

#ifndef M_PI_2
#define M_PI_2 1.5707963267948966;
#endif

#ifndef M_PI_4
#define M_PI_4 0.7853981633974483;
#endif

static inline double FastArcTan(double x) {
	return M_PI_4 * x - x * (fabs(x) - 1) * (0.2447 + 0.0663 * fabs(x));
}

static inline double FastArcTan2(double y, double x) {
    if (x == 0.0 && y == 0.0) {
        return 0.0;
    }

	if (x >= 0) { // -pi/2 .. pi/2
		if (y >= 0) { // 0 .. pi/2
			if (y < x) { // 0 .. pi/4
				return FastArcTan(y / x);
			} else { // pi/4 .. pi/2
				return M_PI_2 - FastArcTan(x / y);
			}
		} else {
			if (-y < x) { // -pi/4 .. 0
				return FastArcTan(y / x);
			} else { // -pi/2 .. -pi/4
				return -M_PI_2 - FastArcTan(x / y);
			}
		}
	} else { // -pi..-pi/2, pi/2..pi
		if (y >= 0) { // pi/2 .. pi
			if (y < -x) { // pi*3/4 .. pi
				return FastArcTan(y / x) + M_PI;
			} else { // pi/2 .. pi*3/4
				return M_PI_2 - FastArcTan(x / y);
			}
		} else { // -pi .. -pi/2
			if (-y < -x) { // -pi .. -pi*3/4
				return FastArcTan(y / x) - M_PI;
			} else { // -pi*3/4 .. -pi/2
				return -M_PI_2 - FastArcTan(x / y);
			}
		}
	}
}

/*#define M_PI_4_P_0273	1.05839816339744830962
static inline double FastArcTan2(double y, double x) {
    double absx, absy;
  absy = fabs(y);
  absx = fabs(x);
  short octant = ((x<0) << 2) + ((y<0) << 1 ) + (absx <= absy);
  switch (octant) {
    case 0: {
        if (x == 0 && y == 0)
          return 0;
        double val = absy/absx;
        return (M_PI_4_P_0273 - 0.273*val)*val; //1st octant
        break;
      }
    case 1:{
        if (x == 0 && y == 0)
          return 0.0;
        double val = absx/absy;
        return M_PI_2 - (M_PI_4_P_0273 - 0.273*val)*val; //2nd octant
        break;
      }
    case 2: {
        double val =absy/absx;
        return -(M_PI_4_P_0273 - 0.273*val)*val; //8th octant
        break;
      }
    case 3: {
        double val =absx/absy;
        return -M_PI_2 + (M_PI_4_P_0273 - 0.273*val)*val;//7th octant
        break;
      }
    case 4: {
        double val =absy/absx;
        return  M_PI - (M_PI_4_P_0273 - 0.273*val)*val;  //4th octant
      }
    case 5: {
        double val =absx/absy;
        return  M_PI_2 + (M_PI_4_P_0273 - 0.273*val)*val;//3rd octant
        break;
      }
    case 6: {
        double val =absy/absx;
        return -M_PI + (M_PI_4_P_0273 - 0.273*val)*val; //5th octant
        break;
      }
    case 7: {
        double val =absx/absy;
        return -M_PI_2 - (M_PI_4_P_0273 - 0.273*val)*val; //6th octant
        break;
      }
    default:
      return 0.0;
    }
}*/

/*
#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f

static inline double FastArcTan2(double y, double x) {
    if ( x == 0.0f )
	{
		if ( y > 0.0f ) return PIBY2_FLOAT;
		if ( y == 0.0f ) return 0.0f;
		return -PIBY2_FLOAT;
	}
	float atan;
	float z = y/x;
	if ( fabs( z ) < 1.0f )
	{
		atan = z/(1.0f + 0.28f*z*z);
		if ( x < 0.0f )
		{
			if ( y < 0.0f ) return atan - PI_FLOAT;
			return atan + PI_FLOAT;
		}
	}
	else
	{
		atan = PIBY2_FLOAT - z/(z*z + 0.28f);
		if ( y < 0.0f ) return atan - PI_FLOAT;
	}
	return atan;
}*/



// Got from SLEEF
static inline float mlaf(float x, float y, float z) {
#if defined(__FMA__) || defined(__FMA)
    return std::fma(x, y, z);
#else
    return x * y + z;
#endif
}

static inline int rintfk(float x) {
    return static_cast<int>(std::round(x));
}

static inline float fabsfk(float x) {
    return std::fabs(x);
}

static inline float xfastsinf_u3500(float d) {
    int q;
    float u, s;

    q = rintfk(d * (float)M_1_PI);
    d = mlaf((float)q, -(float)M_PI, d);

    s = d * d;

    u = -0.1881748176e-3f;
    u = mlaf(u, s, +0.8323502727e-2f);
    u = mlaf(u, s, -0.1666651368e+0f);
    u = mlaf(s * d, u, d);

    if ((q & 1) != 0) u = -u;

    return u;
}

static inline float xfastcosf_u3500(float d) {
    int q;
    float u, s;

    q = rintfk(mlaf(d, (float)M_1_PI, -0.5f));
    d = mlaf((float)q, -(float)M_PI, d - (float)M_PI * 0.5f);

    s = d * d;

    u = -0.1881748176e-3f;
    u = mlaf(u, s, +0.8323502727e-2f);
    u = mlaf(u, s, -0.1666651368e+0f);
    u = mlaf(s * d, u, d);

    if ((q & 1) == 0) u = -u;

    return u;
}

#ifndef FAST_MATH

static inline void sincos(double phase, double &s, double &c) {
    float pf = static_cast<float>(phase);
    s = std::sin(pf);
    c = std::cos(pf);
}
#else

static inline void sincos(double phase, double &s, double &c) {
    float pf = static_cast<float>(phase);
    float sf = xfastsinf_u3500(pf);
    float cf = xfastcosf_u3500(pf);
    s = static_cast<double>(sf);
    c = static_cast<double>(cf);
}

#endif /* FAST_MATH */