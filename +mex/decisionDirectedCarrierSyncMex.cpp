#include "mex.h"
#include <cmath>
#include <vector>

#include "defines.hpp"
#include "include/math.hpp"

constexpr double SQRT_2 = std::sqrt(2.0);

static inline void wrap_phase(double &phase) {
    const double TWO_PI = 2.0 * M_PI;
    double k = std::floor((phase + M_PI) / TWO_PI);
    phase -= k * TWO_PI;
    if (phase <= -M_PI) phase += TWO_PI;
    else if (phase > M_PI) phase -= TWO_PI;
}

static inline void decideQpsk(double yr, double yi, double &dr, double &di)
{
    const double invSqrt2 = 1.0 / SQRT_2;
    double reHat = (yr >= 0.0) ? 1.0 : -1.0;
    double imHat = (yi >= 0.0) ? 1.0 : -1.0;

    dr = reHat * invSqrt2;
    di = imHat * invSqrt2;
}

static inline void buildQamLevels(int M, int &L, std::vector<double> &levels,
                                  double &EsAvg, double &scaleQam, double &idxOffset)
{
    double Ld = std::sqrt(static_cast<double>(M));
    L = static_cast<int>(std::round(Ld)); // Level
    if (std::fabs(Ld - L) > 1e-9) {
        mexErrMsgIdAndTxt("decisionDirectedCarrierSyncMex:InvalidM",
                          "For M>4 only square QAM (M = L^2) is supported.");
    }

    levels.resize(L);
    double halfIdx = (static_cast<double>(L) - 1.0) / 2.0;
    for (int i = 0; i < L; ++i) {
        double m = static_cast<double>(i) - halfIdx;
        levels[i] = 2.0 * m;   // e.g. L=4 -> [-3 -1 1 3]
    }

    // Average energy
    EsAvg = 0.0;
    for (int i = 0; i < L; ++i) {
        EsAvg += levels[i] * levels[i];
    }
    EsAvg /= static_cast<double>(L);

    // Overall scaling
    scaleQam  = 1.0 / std::sqrt(2.0 * EsAvg);
    idxOffset = static_cast<double>(halfIdx);
}

static inline double quantizeQamScalar(double x,
                                       const std::vector<double> &levels,
                                       int L,
                                       double idxOffset)
{
    double kd = std::round(x / 2.0 + idxOffset);
    int    k  = static_cast<int>(kd);

    if (k < 0) {
        k = 0;
    } else if (k > (L - 1)) {
        k = L - 1;
    }
    return levels[k];
}

static void loopSingle(mxComplexSingle *xData,
                       mxComplexSingle *yData,
                       mwSize N,
                       int M,
                       double Kp,
                       double Ki,
                       double &phase,
                       double &freq)
{
    bool isQpsk = (M == 4);
    int L = 0;
    std::vector<double> levels;
    double EsAvg     = 1.0;
    double scaleQam  = 1.0;
    double idxOffset = 0.0;

    if (!isQpsk) {
        buildQamLevels(M, L, levels, EsAvg, scaleQam, idxOffset);
    }

    for (mwSize k = 0; k < N; ++k) {
        double xr = static_cast<double>(xData[k].real);
        double xi = static_cast<double>(xData[k].imag);

        double c, s;
        sincos(phase, s, c);

        double yr = xr * c + xi * s;
        double yi = xi * c - xr * s;

        yData[k].real = static_cast<float>(yr);
        yData[k].imag = static_cast<float>(yi);

#ifndef FAST_DECIDE
        double dr, di;
        if (isQpsk) {
            decideQpsk(yr, yi, dr, di);
        } else {
            double reHat = quantizeQamScalar(yr, levels, L, idxOffset);
            double imHat = quantizeQamScalar(yi, levels, L, idxOffset);
            dr = reHat * scaleQam;
            di = imHat * scaleQam;
        }
#else
        double dr, di;
        decideQpsk(yr, yi, dr, di);
#endif /* FAST_DECIDE */


#ifndef FAST_MATH
        double re = yr * dr + yi * di;
        double im = yi * dr - yr * di;
        double e  = std::atan2(im, re);
#else
    
    #ifndef APPROX
        double re = yr * dr + yi * di;
        double im = yi * dr - yr * di;
        double e  = FastArcTan2(im, re);
    #else
        double ei = yi * dr - yr * di;
        double e  = ei;
    #endif /* APPROX */

#endif /* FAST_MATH */

        // PLL update
        freq  += Ki * e;
        phase += freq + Kp * e;

        // Wrap phase
        wrap_phase(phase);
    }
}

static void loopDoubleFull(mxComplexDouble *xData,
                           mxComplexDouble *yData,
                           mwSize N,
                           int M,
                           double Kp,
                           double Ki,
                           double &phase,
                           double &freq)
{
    bool isQpsk = (M == 4);
    int L = 0;
    std::vector<double> levels;
    double EsAvg     = 1.0;
    double scaleQam  = 1.0;
    double idxOffset = 0.0;

    if (!isQpsk) {
        buildQamLevels(M, L, levels, EsAvg, scaleQam, idxOffset);
    }

    for (mwSize k = 0; k < N; ++k) {
        double xr = xData[k].real;
        double xi = xData[k].imag;

        double c, s;
        sincos(phase, s, c);

        // Apply rotation: y = x * exp(-j*phase)
        double yr = xr * c + xi * s;
        double yi = xi * c - xr * s;

        yData[k].real = yr;
        yData[k].imag = yi;

#ifndef FAST_DECIDE
        double dr, di;
        if (isQpsk) {
            decideQpsk(yr, yi, dr, di);
        } else {
            double reHat = quantizeQamScalar(yr, levels, L, idxOffset);
            double imHat = quantizeQamScalar(yi, levels, L, idxOffset);
            dr = reHat * scaleQam;
            di = imHat * scaleQam;
        }
#else
        double dr, di;
        decideQpsk(yr, yi, dr, di);
#endif /* FAST_DECIDE */


#ifndef FAST_MATH
        double re = yr * dr + yi * di;
        double im = yi * dr - yr * di;
        double e  = std::atan2(im, re);
#else
    
    #ifndef APPROX
        double re = yr * dr + yi * di;
        double im = yi * dr - yr * di;
        double e  = FastArcTan2(im, re);
    #else
        double ei = yi * dr - yr * di;
        double e  = ei;
    #endif /* APPROX */

#endif /* FAST_MATH */

        // PLL update
        freq  += Ki * e;
        phase += freq + Kp * e;

        // Wrap phase
        wrap_phase(phase);
    }
}

// Entry point
void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    if (nrhs != 6) {
        mexErrMsgIdAndTxt("decisionDirectedCarrierSyncMex:InvalidNumInputs",
            "Expected 6 inputs: x, M, Kp, Ki, phase0, freq0.");
    }
    if (nlhs != 3) {
        mexErrMsgIdAndTxt("decisionDirectedCarrierSyncMex:InvalidNumOutputs",
            "Expected 3 outputs: y, phaseOut, freqOut.");
    }

    // Input signal x (in form of symbols)
    const mxArray *x_in = prhs[0];

    if (!mxIsComplex(x_in) ||
        !(mxIsDouble(x_in) || mxIsSingle(x_in))) {
        mexErrMsgIdAndTxt("decisionDirectedCarrierSyncMex:InvalidX",
            "Input x must be complex double or complex single.");
    }

    const bool isDoubleIn = mxIsDouble(x_in);
    const bool isSingleIn = mxIsSingle(x_in);

    mwSize N = mxGetNumberOfElements(x_in);

    // Loop parameters
    int    M     = static_cast<int>(mxGetScalar(prhs[1]));
    double Kp    = mxGetScalar(prhs[2]);
    double Ki    = mxGetScalar(prhs[3]);
    double phase = mxGetScalar(prhs[4]);
    double freq  = mxGetScalar(prhs[5]);

    if (M <= 0) {
        mexErrMsgIdAndTxt("decisionDirectedCarrierSyncMex:InvalidM",
                          "M must be positive.");
    }

    // Output y has same class and shape as x
    mwSize nDims       = mxGetNumberOfDimensions(x_in);
    const mwSize *dims = mxGetDimensions(x_in);

    mxClassID outClass = isDoubleIn ? mxDOUBLE_CLASS : mxSINGLE_CLASS;
    plhs[0] = mxCreateNumericArray(nDims, dims, outClass, mxCOMPLEX);

    if (isDoubleIn) {
        mxComplexDouble *xData = mxGetComplexDoubles(x_in);
        mxComplexDouble *yData = mxGetComplexDoubles(plhs[0]);

        loopDoubleFull(xData, yData, N, M, Kp, Ki, phase, freq);
    } else {
        mxComplexSingle *xData = mxGetComplexSingles(x_in);
        mxComplexSingle *yData = mxGetComplexSingles(plhs[0]);

        loopSingle(xData, yData, N, M, Kp, Ki, phase, freq);
    }

    // Phase and freq always returned as double scalars
    plhs[1] = mxCreateDoubleScalar(phase);
    plhs[2] = mxCreateDoubleScalar(freq);
}
