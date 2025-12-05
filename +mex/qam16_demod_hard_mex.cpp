#include "mex.h"
#include <vector>
#include <cmath>

#include "include/simd.hpp"
#include "defines.hpp"

constexpr double normFactor = std::sqrt(10.0);

static void demod16qam_core(const SHReal* xReal,
                            const SHReal* xImag,
                            mwSize N,
                            double* bitsOut)
{
    if (N == 0) return;

    mwSize i = 0;
    double sumZ2 = 0.0;
    SHVec vsum = SH_SETZERO();
    for (; i + SH_LANES <= N; i += SH_LANES) {
        SHVec xr  = SH_LOAD(xReal + i);
        SHVec xi  = SH_LOAD(xImag + i);
        SHVec xr2 = SH_MUL(xr, xr);
        SHVec xi2 = SH_MUL(xi, xi);
        SHVec mag2 = SH_ADD(xr2, xi2);
        vsum = SH_ADD(vsum, mag2);
    }

    SHReal tmp[SH_LANES];
    SH_STORE(tmp, vsum);
    for (int k = 0; k < SH_LANES; ++k) {
        sumZ2 += static_cast<double>(tmp[k]);
    }

    // tail data
    for (; i < N; ++i) {
        double xr = static_cast<double>(xReal[i]);
        double xi = static_cast<double>(xImag[i]);
        sumZ2 += xr * xr + xi * xi;
    }

    double Pz = sumZ2 / static_cast<double>(N);
    if (Pz <= 0.0) {
        // Degenerate case: return all zeros
        for (mwSize n = 0; n < 4 * N; ++n) {
            bitsOut[n] = 0.0;
        }
        return;
    }

    double gainMag = std::sqrt(Pz);
    double scale_d = normFactor / gainMag;

    // Loop up table
    static const int lut[4][2] = {
        {0, 0},   // -3
        {0, 1},   // -1
        {1, 1},   // +1
        {1, 0}    // +3
    };

    // Mapping
    for (mwSize n = 0; n < N; ++n) {
        double xr = static_cast<double>(xReal[n]);
        double xi = static_cast<double>(xImag[n]);

        // Scale so that sRe/sIm ~= {-3,-1,+1,+3}
        double sRe = xr * scale_d;
        double sIm = xi * scale_d;

        int idxI;
        if      (sRe < -2.0) idxI = 0;
        else if (sRe <  0.0) idxI = 1;
        else if (sRe <  2.0) idxI = 2;
        else                  idxI = 3;

        int idxQ;
        if      (sIm < -2.0) idxQ = 0;
        else if (sIm <  0.0) idxQ = 1;
        else if (sIm <  2.0) idxQ = 2;
        else                  idxQ = 3;

        int b3 = lut[idxQ][0];   // Q msb
        int b2 = lut[idxQ][1];   // Q lsb
        int b1 = lut[idxI][0];   // I msb
        int b0 = lut[idxI][1];   // I lsb

        mwSize base = 4 * n;
        bitsOut[base    ] = static_cast<double>(b3);
        bitsOut[base + 1] = static_cast<double>(b2);
        bitsOut[base + 2] = static_cast<double>(b1);
        bitsOut[base + 3] = static_cast<double>(b0);
    }
}

// Mex entry point
void mexFunction(int nlhs, mxArray* plhs[],
                 int nrhs, const mxArray* prhs[])
{
    if (nrhs != 1) {
        mexErrMsgIdAndTxt("qam16_demod_hard_mex:InvalidNumInputs",
                          "Expected 1 input: symbols.");
    }
    if (nlhs != 1) {
        mexErrMsgIdAndTxt("qam16_demod_hard_mex:InvalidNumOutputs",
                          "Expected 1 output: bits.");
    }

    const mxArray* x_in = prhs[0];

    if (!mxIsComplex(x_in)) {
        mexErrMsgIdAndTxt("qam16_demod_hard_mex:InvalidX",
                          "Input symbols must be complex.");
    }

    mwSize N = mxGetNumberOfElements(x_in);

    // Output: double column vector, length = 4*N
    plhs[0] = mxCreateDoubleMatrix(4 * N, 1, mxREAL);
    double* bitsOut = mxGetPr(plhs[0]);

    if (N == 0) {
        return;
    }

#ifdef SH_USE_FLOAT
    // Expect complex single
    if (!mxIsSingle(x_in)) {
        mexErrMsgIdAndTxt("qam16_demod_hard_mex:InvalidXType",
                          "With SH_USE_FLOAT, input must be complex single.");
    }

    mxComplexSingle* data = mxGetComplexSingles(x_in);

    std::vector<SHReal> re(N), im(N);
    for (mwSize n = 0; n < N; ++n) {
        re[n] = static_cast<SHReal>(data[n].real);
        im[n] = static_cast<SHReal>(data[n].imag);
    }

#else
    // Expect complex double
    if (!mxIsDouble(x_in)) {
        mexErrMsgIdAndTxt("qam16_demod_hard_mex:InvalidXType",
                          "Without SH_USE_FLOAT, input must be complex double.");
    }

    mxComplexDouble* data = mxGetComplexDoubles(x_in);

    std::vector<SHReal> re(N), im(N);
    for (mwSize n = 0; n < N; ++n) {
        re[n] = static_cast<SHReal>(data[n].real);
        im[n] = static_cast<SHReal>(data[n].imag);
    }
#endif

    demod16qam_core(re.data(), im.data(), N, bitsOut);
}
