#include "mex.h"
#include <vector>
#include <cmath>
#include <cfloat>
#include <algorithm>
#include <limits>


#include "include/simd.hpp"
#include "include/math.hpp"
#include "defines.hpp"

static inline SHReal sc_epsilon()
{
    return static_cast<SHReal>(std::numeric_limits<SHReal>::epsilon());
}

struct Candidate
{
    double startSample;      // StartSample (1-based, in samples)
    int    sampleOffset;     // SampleOffset (0..sps-1)
    int    preambleStartSym; // PreambleStartSym (1-based, in symbols)
    double metric;           // M(d)
    double windowPower;      // R(d)
    double cfoRadPerSym;     // CFO estimate per symbol (rad/sym)
};

static mxArray* createCandidateStructArray(std::size_t n)
{
    const char* fieldNames[] = {
        "StartSample",
        "SampleOffset",
        "PreambleStartSym",
        "Metric",
        "WindowPower",
        "CfoRadPerSym"
    };
    constexpr int nFields = 6;
    return mxCreateStructMatrix(1, static_cast<mwSize>(n), nFields, fieldNames);
}

void mexFunction(int nlhs, mxArray* plhs[],
                 int nrhs, const mxArray* prhs[])
{
    if (nrhs != 5) {
        mexErrMsgIdAndTxt("schmidlCoxDetectMex:InvalidNumInputs",
                          "Expected 5 inputs: y, sps, Lh, metricThreshold, minWindowPower.");
    }
    if (nlhs > 1) {
        mexErrMsgIdAndTxt("schmidlCoxDetectMex:InvalidNumOutputs",
                          "One output (struct array) expected.");
    }

    const mxArray* y_in = prhs[0];

#ifdef SH_USE_FLOAT
    if (!mxIsSingle(y_in)) {
        mexErrMsgIdAndTxt("schmidlCoxDetectMex:InvalidY",
                          "Input y must be single (real or complex) when compiled with SH_USE_FLOAT.");
    }
#else
    if (!mxIsDouble(y_in)) {
        mexErrMsgIdAndTxt("schmidlCoxDetectMex:InvalidY",
                          "Input y must be double (real or complex).");
    }
#endif

    const mwSize N = mxGetNumberOfElements(y_in);
    const bool isComplex = mxIsComplex(y_in);

#ifdef SH_USE_FLOAT
    mxComplexSingle* yc = nullptr;
    SHReal*          yr = nullptr;

    if (isComplex) {
        yc = mxGetComplexSingles(y_in);   // interleaved complex single
    } else {
        yr = reinterpret_cast<SHReal*>(mxGetData(y_in));  // purely real, imag=0
    }
#else
    mxComplexDouble* yc = nullptr;
    SHReal*          yr = nullptr;

    if (isComplex) {
        yc = mxGetComplexDoubles(y_in);   // interleaved complex double
    } else {
        yr = mxGetDoubles(y_in);   // purely real, imag=0
    }
#endif

    int sps = static_cast<int>(mxGetScalar(prhs[1]));
    int Lh  = static_cast<int>(mxGetScalar(prhs[2]));
    double metricThreshold = mxGetScalar(prhs[3]);
    double minWindowPower  = mxGetScalar(prhs[4]);

    if (sps <= 0 || Lh <= 0) {
        mexErrMsgIdAndTxt("schmidlCoxDetectMex:InvalidParams",
                          "sps and Lh must be positive.");
    }

    // There is no sample
    if (N == 0) {
        plhs[0] = createCandidateStructArray(0);
        return;
    }

    const int Nint = static_cast<int>(N);
    const int Lh2  = 2 * Lh; // full preamble length in symbols
    const double Lpre    = 2.0 * static_cast<double>(Lh);
    const double maxSpan = Lpre / 2.0; // grouping span in samples in chosing candidates (Lpre/2)

    std::vector<SHReal> ySymRe;
    std::vector<SHReal> ySymIm;
    std::vector<SHReal> qRe, qIm;
    std::vector<SHReal> pow;
    std::vector<SHReal> PRe, PIm;
    std::vector<SHReal> R;
    std::vector<SHReal> M;

    // For faster execution
    ySymRe.reserve(N);
    ySymIm.reserve(N);
    qRe.reserve(N);
    qIm.reserve(N);
    pow.reserve(N);
    PRe.reserve(N);
    PIm.reserve(N);
    R.reserve(N);
    M.reserve(N);

    std::vector<Candidate> candRaw;
    candRaw.reserve(64);

    for (int off = 0; off < sps; ++off) {
        if (off >= Nint) {
            break;
        }

        // Number of symbols at this offset
        int Ns = 1 + (Nint - 1 - off) / sps;
        if (Ns < (2 * Lh + 1)) {
            continue;
        }

        int Lwin = Ns - 2 * Lh;   // number of positions for sliding window
        if (Lwin <= 0) {
            continue;
        }

        ySymRe.resize(Ns);
        ySymIm.resize(Ns);

        // Downsample to symbol rate for this offset
        for (int n = 0, idx = off; n < Ns; ++n, idx += sps) {
            if (isComplex) {
#ifdef SH_USE_FLOAT
                ySymRe[n] = yc[idx].real;
                ySymIm[n] = yc[idx].imag;
#else
                ySymRe[n] = yc[idx].real;
                ySymIm[n] = yc[idx].imag;
#endif
            } else {
                ySymRe[n] = yr[idx];
                ySymIm[n] = static_cast<SHReal>(0);
            }
        }

        const int Nq = Ns - Lh;
        if (Nq <= 0) {
            continue;
        }

        qRe.resize(Nq);
        qIm.resize(Nq);
        pow.resize(Ns);

        // ----------------------------------------------------
        // q(n) = y(n) * conj(y(n+Lh)), pow(n) = |y(n)|^2
        // ----------------------------------------------------
        int n = 0;
        for (; n + (SH_LANES - 1) < Nq; n += SH_LANES) {
            SHVec ar = SH_LOAD(ySymRe.data() + n);
            SHVec ai = SH_LOAD(ySymIm.data() + n);

            SHVec br = SH_LOAD(ySymRe.data() + n + Lh);
            SHVec bi = SH_LOAD(ySymIm.data() + n + Lh);

            SHVec arbr = SH_MUL(ar, br);
            SHVec aibi = SH_MUL(ai, bi);
            SHVec qr   = SH_ADD(arbr, aibi);

            SHVec aibr = SH_MUL(ai, br);
            SHVec arbi = SH_MUL(ar, bi);
            SHVec qi   = SH_SUB(aibr, arbi);  // imag: ai*br - ar*bi

            SH_STORE(qRe.data() + n, qr);
            SH_STORE(qIm.data() + n, qi);
        }
        // tail for q
        for (; n < Nq; ++n) {
            SHReal ar = ySymRe[n];
            SHReal ai = ySymIm[n];
            SHReal br = ySymRe[n + Lh];
            SHReal bi = ySymIm[n + Lh];

            SHReal qr = ar * br + ai * bi;
            SHReal qi = -ar * bi + ai * br;

            qRe[n] = qr;
            qIm[n] = qi;
        }

        // pow(n) = |y(n)|^2
        n = 0;
        for (; n + (SH_LANES - 1) < Ns; n += SH_LANES) {
            SHVec ar = SH_LOAD(ySymRe.data() + n);
            SHVec ai = SH_LOAD(ySymIm.data() + n);

            SHVec ar2  = SH_MUL(ar, ar);
            SHVec ai2  = SH_MUL(ai, ai);
            SHVec mag2 = SH_ADD(ar2, ai2);

            SH_STORE(pow.data() + n, mag2);
        }
        for (; n < Ns; ++n) {
            SHReal ar = ySymRe[n];
            SHReal ai = ySymIm[n];
            pow[n]    = ar * ar + ai * ai;
        }

        // ----------------------------------------------------
        // Sliding sums for P(d) over q (length Lh)
        // and R(d) over pow (length 2*Lh)
        // ----------------------------------------------------
        PRe.resize(Lwin);
        PIm.resize(Lwin);
        R.resize(Lwin);

        // P(d) sliding sum
        SHVec sumPr_v = SH_SETZERO();
        SHVec sumPi_v = SH_SETZERO();

        int m = 0;
        for (; m + (SH_LANES - 1) < Lh; m += SH_LANES) {
            SHVec re = SH_LOAD(qRe.data() + m);
            SHVec im = SH_LOAD(qIm.data() + m);

            sumPr_v = SH_ADD(sumPr_v, re);
            sumPi_v = SH_ADD(sumPi_v, im);
        }

        SHReal tmpPr[SH_LANES];
        SHReal tmpPi[SH_LANES];
        SH_STORE(tmpPr, sumPr_v);
        SH_STORE(tmpPi, sumPi_v);

        SHReal sumPr = static_cast<SHReal>(0);
        SHReal sumPi = static_cast<SHReal>(0);
        for (int i = 0; i < SH_LANES; ++i) {
            sumPr += tmpPr[i];
            sumPi += tmpPi[i];
        }

        // tail data
        for (; m < Lh; ++m) {
            sumPr += qRe[m];
            sumPi += qIm[m];
        }

        PRe[0] = sumPr;
        PIm[0] = sumPi;

        int d = 1;
        for (; d + (SH_LANES - 1) < Lwin; d += SH_LANES) {
            SHVec hiRe = SH_LOAD(qRe.data() + (d + Lh - 1));
            SHVec hiIm = SH_LOAD(qIm.data() + (d + Lh - 1));

            SHVec loRe = SH_LOAD(qRe.data() + (d - 1));
            SHVec loIm = SH_LOAD(qIm.data() + (d - 1));

            SHVec diffRe_v = SH_SUB(hiRe, loRe);
            SHVec diffIm_v = SH_SUB(hiIm, loIm);

            SHReal diffRe[SH_LANES];
            SHReal diffIm[SH_LANES];
            SH_STORE(diffRe, diffRe_v);
            SH_STORE(diffIm, diffIm_v);

            for (int i = 0; i < SH_LANES; ++i) {
                sumPr += diffRe[i];
                sumPi += diffIm[i];

                int di = d + i;
                PRe[di] = sumPr;
                PIm[di] = sumPi;
            }
        }

        // tail data
        for (; d < Lwin; ++d) {
            sumPr += qRe[d + Lh - 1] - qRe[d - 1];
            sumPi += qIm[d + Lh - 1] - qIm[d - 1];

            PRe[d] = sumPr;
            PIm[d] = sumPi;
        }

        // R(d) sliding sum over pow, length 2*Lh
        SHVec sumR_v = SH_SETZERO();
        m = 0;
        for (; m + (SH_LANES - 1) < Lh2; m += SH_LANES) {
            SHVec v = SH_LOAD(pow.data() + m);
            sumR_v  = SH_ADD(sumR_v, v);
        }

        SHReal tmpR[SH_LANES];
        SH_STORE(tmpR, sumR_v);
        SHReal sumR = static_cast<SHReal>(0);
        for (int i = 0; i < SH_LANES; ++i) {
            sumR += tmpR[i];
        }

        for (; m < Lh2; ++m) {
            sumR += pow[m];
        }

        R[0] = sumR;

        d = 1;
        for (; d + (SH_LANES - 1) < Lwin; d += SH_LANES) {
            SHVec hi = SH_LOAD(pow.data() + (d + Lh2 - 1));
            SHVec lo = SH_LOAD(pow.data() + (d - 1));

            SHVec diff_v = SH_SUB(hi, lo);

            SHReal diff[SH_LANES];
            SH_STORE(diff, diff_v);

            for (int i = 0; i < SH_LANES; ++i) {
                sumR += diff[i];
                int di = d + i;
                R[di] = sumR;
            }
        }

        // tail data
        for (; d < Lwin; ++d) {
            sumR += pow[d + Lh2 - 1] - pow[d - 1];
            R[d] = sumR;
        }

        // ----------------------------------------------------
        // Compute M(d) scalar and pick local maxima as candidates
        // ----------------------------------------------------
        M.resize(Lwin);
        const SHReal eps = sc_epsilon();
        for (int k = 0; k < Lwin; ++k) {
            SHReal Rv = R[k];
            SHReal Pr = PRe[k];
            SHReal Pi = PIm[k];

            SHReal magP2 = Pr * Pr + Pi * Pi;
            SHReal denom = Rv * Rv + eps;
            M[k]         = magP2 / denom;
        }

        for (int k = 0; k < Lwin; ++k) {
            SHReal Rv     = R[k];
            SHReal metric = M[k];

            if (static_cast<double>(Rv) <= minWindowPower) {
                continue;
            }
            if (static_cast<double>(metric) <= metricThreshold) {
                continue;
            }

            // Local maxima condition: M(k) > M(k-1) and M(k) > M(k+1)
            if (k > 0 && metric <= M[k - 1]) {
                continue;
            }
            if (k < Lwin - 1 && metric <= M[k + 1]) {
                continue;
            }

            SHReal Pr = PRe[k];
            SHReal Pi = PIm[k];

            // TODO: make this faster
#ifndef FAST_MATH
            double phi = std::atan2(static_cast<double>(Pi),
                                    static_cast<double>(Pr));
#else
            double phi = FastArcTan2(static_cast<double>(Pi),
                                    static_cast<double>(Pr));
#endif /* FAST_MATH */
            double cfoRadPerSym = phi / static_cast<double>(Lh);

            // Convert (off, k) to absolute sample index (1-based) in y
            double startSample = 1.0
                               + static_cast<double>(off)
                               + static_cast<double>(k) * static_cast<double>(sps);

            Candidate c;
            c.startSample      = startSample;
            c.sampleOffset     = off;
            c.preambleStartSym = k + 1;  // 1-based
            c.metric           = static_cast<double>(metric);
            c.windowPower      = static_cast<double>(Rv);
            c.cfoRadPerSym     = cfoRadPerSym;

            candRaw.push_back(c);
        }
    }

    if (candRaw.empty()) {
        plhs[0] = createCandidateStructArray(0);
        return;
    }

    // Sort candidates by their starting sample
    std::sort(candRaw.begin(), candRaw.end(),
              [](const Candidate& a, const Candidate& b) {
                  return a.startSample < b.startSample;
              });

    // Merge Candidates
    std::vector<Candidate> candMerged;
    candMerged.reserve(candRaw.size());

    const int Ncand = static_cast<int>(candRaw.size());

    int    groupStart = 0;
    double dist       = 0.0;

    for (int i = 1; i < Ncand; ++i) {
        double gap = candRaw[i].startSample - candRaw[i - 1].startSample;

        if (dist + gap <= maxSpan * sps) {
            // Still within same preamble plateau
            dist += gap;
        } else {
            // Close group [groupStart .. i-1]
            int    bestIdx = groupStart;
            double bestM   = candRaw[groupStart].metric;
            for (int j = groupStart + 1; j <= i - 1; ++j) {
                if (candRaw[j].metric > bestM) {
                    bestM   = candRaw[j].metric;
                    bestIdx = j;
                }
            }
            candMerged.push_back(candRaw[bestIdx]);

            // Start new group at i
            groupStart = i;
            dist       = 0.0;
        }
    }

    // Close last group [groupStart .. Ncand-1]
    {
        int    bestIdx = groupStart;
        double bestM   = candRaw[groupStart].metric;
        for (int j = groupStart + 1; j <= Ncand - 1; ++j) {
            if (candRaw[j].metric > bestM) {
                bestM   = candRaw[j].metric;
                bestIdx = j;
            }
        }
        candMerged.push_back(candRaw[bestIdx]);
    }

    mxArray* out = createCandidateStructArray(candMerged.size());

    for (mwSize i = 0; i < candMerged.size(); ++i) {
        const Candidate& c = candMerged[i];

        mxSetField(out, i, "StartSample",
                   mxCreateDoubleScalar(c.startSample));
        mxSetField(out, i, "SampleOffset",
                   mxCreateDoubleScalar(static_cast<double>(c.sampleOffset)));
        mxSetField(out, i, "PreambleStartSym",
                   mxCreateDoubleScalar(static_cast<double>(c.preambleStartSym)));
        mxSetField(out, i, "Metric",
                   mxCreateDoubleScalar(c.metric));
        mxSetField(out, i, "WindowPower",
                   mxCreateDoubleScalar(c.windowPower));
        mxSetField(out, i, "CfoRadPerSym",
                   mxCreateDoubleScalar(c.cfoRadPerSym));
    }

    plhs[0] = out;
}
