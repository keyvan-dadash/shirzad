#include "mex.h"
#include <vector>
#include <cmath>

#include "include/simd.hpp"
#include "defines.hpp"

constexpr double normFactor = std::sqrt(42.0);

static void demod64qam_core(const SHReal* xReal,
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
        SHVec mag2 = SH_ADD(SH_MUL(xr, xr), SH_MUL(xi, xi));
        vsum = SH_ADD(vsum, mag2);
    }

    SHReal tmp[SH_LANES];
    SH_STORE(tmp, vsum);
    for (int k = 0; k < SH_LANES; ++k) sumZ2 += static_cast<double>(tmp[k]);
    for (; i < N; ++i) sumZ2 += xReal[i] * xReal[i] + xImag[i] * xImag[i];

    double Pz = sumZ2 / static_cast<double>(N);
    if (Pz <= 0.0) {
        std::fill(bitsOut, bitsOut + 6 * N, 0.0);
        return;
    }

    double gainMag = std::sqrt(Pz);
    double scale_d = normFactor / gainMag;

    // --- Decoding LUT (3 bits per axis) ---
    static const int lut[8][3] = {
        {0,0,0}, // idx 0 -> -7 -> bits 000
        {0,0,1}, // idx 1 -> -5 -> bits 001
        {0,1,0}, // idx 2 -> -3 -> bits 010
        {0,1,1}, // idx 3 -> -1 -> bits 011
        {1,1,1}, // idx 4 -> +1 -> bits 111
        {1,1,0}, // idx 5 -> +3 -> bits 110
        {1,0,1}, // idx 6 -> +5 -> bits 101
        {1,0,0}  // idx 7 -> +7 -> bits 100
    };

    for (mwSize n = 0; n < N; ++n) {
        double sRe = xReal[n] * scale_d;
        double sIm = xImag[n] * scale_d;

        auto slice3 = [](double x) {
            if (x < -6) return 0;
            else if (x < -4) return 1;
            else if (x < -2) return 2;
            else if (x < 0)  return 3;
            else if (x < 2)  return 4;
            else if (x < 4)  return 5;
            else if (x < 6)  return 6;
            else             return 7;
        };

        int idxI = slice3(sRe);
        int idxQ = slice3(sIm);

        const int* qBits = lut[idxQ];
        const int* iBits = lut[idxI];

        mwSize base = 6 * n;
        bitsOut[base    ] = qBits[0];
        bitsOut[base + 1] = qBits[1];
        bitsOut[base + 2] = qBits[2];
        bitsOut[base + 3] = iBits[0];
        bitsOut[base + 4] = iBits[1];
        bitsOut[base + 5] = iBits[2];
    }
}

void mexFunction(int nlhs, mxArray* plhs[],
                 int nrhs, const mxArray* prhs[])
{
    if (nrhs != 1)
        mexErrMsgIdAndTxt("qam64_demod_hard_mex:InvalidNumInputs", "Expected 1 input: symbols.");
    if (nlhs != 1)
        mexErrMsgIdAndTxt("qam64_demod_hard_mex:InvalidNumOutputs", "Expected 1 output: bits.");

    const mxArray* x_in = prhs[0];
    if (!mxIsComplex(x_in))
        mexErrMsgIdAndTxt("qam64_demod_hard_mex:InvalidX", "Input symbols must be complex.");

    mwSize N = mxGetNumberOfElements(x_in);
    plhs[0] = mxCreateDoubleMatrix(6 * N, 1, mxREAL);
    double* bitsOut = mxGetPr(plhs[0]);
    if (N == 0) return;

#ifndef SH_USE_FLOAT
    if (!mxIsDouble(x_in))
        mexErrMsgIdAndTxt("qam64_demod_hard_mex:InvalidXType", "Input must be complex double.");
    mxComplexDouble* data = mxGetComplexDoubles(x_in);
#else
    if (!mxIsSingle(x_in))
        mexErrMsgIdAndTxt("qam64_demod_hard_mex:InvalidXType", "Input must be complex single.");
    mxComplexSingle* data = mxGetComplexSingles(x_in);
#endif

    std::vector<SHReal> re(N), im(N);
    for (mwSize n = 0; n < N; ++n) {
        re[n] = static_cast<SHReal>(data[n].real);
        im[n] = static_cast<SHReal>(data[n].imag);
    }

    demod64qam_core(re.data(), im.data(), N, bitsOut);
}
