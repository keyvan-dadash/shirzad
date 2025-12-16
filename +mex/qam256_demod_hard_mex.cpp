#include "mex.h"
#include <vector>
#include <cmath>

#include "include/simd.hpp"
#include "defines.hpp"

constexpr double normFactor = std::sqrt(170.0);

static void demod256qam_core(const SHReal* xReal,
                             const SHReal* xImag,
                             mwSize N,
                             double* bitsOut)
{
    if (N == 0) return;

    mwSize i = 0;
    double sumZ2 = 0.0;
    SHVec vsum = SH_SETZERO();
    for (; i + SH_LANES <= N; i += SH_LANES) {
        SHVec xr = SH_LOAD(xReal + i);
        SHVec xi = SH_LOAD(xImag + i);
        vsum = SH_ADD(vsum, SH_ADD(SH_MUL(xr, xr), SH_MUL(xi, xi)));
    }
    SHReal tmp[SH_LANES];
    SH_STORE(tmp, vsum);
    for (int k = 0; k < SH_LANES; ++k) sumZ2 += static_cast<double>(tmp[k]);
    for (; i < N; ++i) sumZ2 += xReal[i]*xReal[i] + xImag[i]*xImag[i];

    double Pz = sumZ2 / static_cast<double>(N);
    if (Pz <= 0.0) {
        std::fill(bitsOut, bitsOut + 8 * N, 0.0);
        return;
    }

    double gainMag = std::sqrt(Pz);
    double scale_d = normFactor / gainMag;

    // --- 16-PAM LUT (4 bits per axis) ---
    static const int lut[16][4] = {
        {0,0,0,0}, // -15
        {0,0,0,1}, // -13
        {0,0,1,0}, // -11
        {0,0,1,1}, // -9
        {0,1,0,0}, // -7
        {0,1,0,1}, // -5
        {0,1,1,0}, // -3
        {0,1,1,1}, // -1
        {1,1,1,1}, // +1
        {1,1,1,0}, // +3
        {1,1,0,1}, // +5
        {1,1,0,0}, // +7
        {1,0,1,1}, // +9
        {1,0,1,0}, // +11
        {1,0,0,1}, // +13
        {1,0,0,0}  // +15
    };

    auto slice4 = [](double x) {
        if (x < -14) return 0;
        else if (x < -12) return 1;
        else if (x < -10) return 2;
        else if (x < -8)  return 3;
        else if (x < -6)  return 4;
        else if (x < -4)  return 5;
        else if (x < -2)  return 6;
        else if (x < 0)   return 7;
        else if (x < 2)   return 8;
        else if (x < 4)   return 9;
        else if (x < 6)   return 10;
        else if (x < 8)   return 11;
        else if (x < 10)  return 12;
        else if (x < 12)  return 13;
        else if (x < 14)  return 14;
        else               return 15;
    };

    for (mwSize n = 0; n < N; ++n) {
        double sRe = xReal[n] * scale_d;
        double sIm = xImag[n] * scale_d;

        int idxI = slice4(sRe);
        int idxQ = slice4(sIm);

        const int* qBits = lut[idxQ];
        const int* iBits = lut[idxI];

        mwSize base = 8 * n;
        bitsOut[base    ] = qBits[0];
        bitsOut[base + 1] = qBits[1];
        bitsOut[base + 2] = qBits[2];
        bitsOut[base + 3] = qBits[3];
        bitsOut[base + 4] = iBits[0];
        bitsOut[base + 5] = iBits[1];
        bitsOut[base + 6] = iBits[2];
        bitsOut[base + 7] = iBits[3];
    }
}

void mexFunction(int nlhs, mxArray* plhs[],
                 int nrhs, const mxArray* prhs[])
{
    if (nrhs != 1)
        mexErrMsgIdAndTxt("qam256_demod_hard_mex:InvalidNumInputs", "Expected 1 input: symbols.");
    if (nlhs != 1)
        mexErrMsgIdAndTxt("qam256_demod_hard_mex:InvalidNumOutputs", "Expected 1 output: bits.");

    const mxArray* x_in = prhs[0];
    if (!mxIsComplex(x_in))
        mexErrMsgIdAndTxt("qam256_demod_hard_mex:InvalidX", "Input symbols must be complex.");

    mwSize N = mxGetNumberOfElements(x_in);
    plhs[0] = mxCreateDoubleMatrix(8 * N, 1, mxREAL);
    double* bitsOut = mxGetPr(plhs[0]);
    if (N == 0) return;

#ifndef SH_USE_FLOAT
    if (!mxIsDouble(x_in))
        mexErrMsgIdAndTxt("qam256_demod_hard_mex:InvalidXType", "Input must be complex double.");
    mxComplexDouble* data = mxGetComplexDoubles(x_in);
#else
    if (!mxIsSingle(x_in))
        mexErrMsgIdAndTxt("qam256_demod_hard_mex:InvalidXType", "Input must be complex single.");
    mxComplexSingle* data = mxGetComplexSingles(x_in);
#endif

    std::vector<SHReal> re(N), im(N);
    for (mwSize n = 0; n < N; ++n) {
        re[n] = static_cast<SHReal>(data[n].real);
        im[n] = static_cast<SHReal>(data[n].imag);
    }

    demod256qam_core(re.data(), im.data(), N, bitsOut);
}
