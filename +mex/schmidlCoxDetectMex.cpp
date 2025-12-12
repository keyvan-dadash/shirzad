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
    if (!mxIsSingle(y_in) || !mxIsComplex(y_in)) {
        mexErrMsgIdAndTxt("schmidlCoxDetectMex:InvalidY",
                          "Input y must be complex single when compiled with SH_USE_FLOAT.");
    }
    mxComplexSingle* yc = mxGetComplexSingles(y_in);   // interleaved complex single
#else
    if (!mxIsDouble(y_in) || !mxIsComplex(y_in)) {
        mexErrMsgIdAndTxt("schmidlCoxDetectMex:InvalidY",
                          "Input y must be complex double.");
    }
    mxComplexDouble* yc = mxGetComplexDoubles(y_in);   // interleaved complex double
#endif

    const mwSize N = mxGetNumberOfElements(y_in);

    int sps = static_cast<int>(mxGetScalar(prhs[1]));
    int Lh  = static_cast<int>(mxGetScalar(prhs[2]));
    double metricThreshold = mxGetScalar(prhs[3]);
    double minWindowPower  = mxGetScalar(prhs[4]);

    if (sps <= 0 || Lh <= 0) {
        mexErrMsgIdAndTxt("schmidlCoxDetectMex:InvalidParams",
                          "sps and Lh must be positive.");
    }

    // No samples
    if (N == 0) {
        plhs[0] = createCandidateStructArray(0);
        return;
    }

    const int Nint = static_cast<int>(N);
    const int Lh2  = 2 * Lh; // full preamble length in symbols
    const double Lpre    = 2.0 * static_cast<double>(Lh);
    const double maxSpan = Lpre / 2.0; // grouping span in samples in choosing candidates (Lpre/2)

    // Symbol-rate data is now accessed directly from yc (no ySymRe/ySymIm).
    std::vector<SHReal> qRe, qIm;
    std::vector<SHReal> pow;
    std::vector<SHReal> PRe, PIm;
    std::vector<SHReal> R;
    std::vector<SHReal> M;

    // Reserve for speed
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

        const int Nq = Ns - Lh;
        if (Nq <= 0) {
            continue;
        }

        qRe.resize(Nq);
        qIm.resize(Nq);
        pow.resize(Ns);

        // ----------------------------------------------------
        // q(n) = y(n) * conj(y(n+Lh)), pow(n) = |y(n)|^2
        // Directly from yc with stride = sps (no ySymRe/ySymIm).
        // ----------------------------------------------------
        int n = 0;

        // q and pow for n = 0..Nq-1
        int Nq4 = Nq & ~3;   // largest multiple of 4 <= Nq

        for (; n < Nq4; n += 4) {
            // ----- n -----
            int idx1_0 = off + (n    ) * sps;
            int idx2_0 = off + (n+Lh ) * sps;
#ifdef SH_USE_FLOAT
            const mxComplexSingle& a0 = yc[idx1_0];
            const mxComplexSingle& b0 = yc[idx2_0];
#else
            const mxComplexDouble& a0 = yc[idx1_0];
            const mxComplexDouble& b0 = yc[idx2_0];
#endif
            SHReal ar0 = static_cast<SHReal>(a0.real);
            SHReal ai0 = static_cast<SHReal>(a0.imag);
            SHReal br0 = static_cast<SHReal>(b0.real);
            SHReal bi0 = static_cast<SHReal>(b0.imag);

            SHReal qr0 = ar0 * br0 + ai0 * bi0;
            SHReal qi0 = -ar0 * bi0 + ai0 * br0;
            qRe[n    ] = qr0;
            qIm[n    ] = qi0;
            pow[n    ] = ar0 * ar0 + ai0 * ai0;

            // ----- n+1 -----
            int n1 = n + 1;
            int idx1_1 = off + n1 * sps;
            int idx2_1 = off + (n1 + Lh) * sps;
#ifdef SH_USE_FLOAT
            const mxComplexSingle& a1 = yc[idx1_1];
            const mxComplexSingle& b1 = yc[idx2_1];
#else
            const mxComplexDouble& a1 = yc[idx1_1];
            const mxComplexDouble& b1 = yc[idx2_1];
#endif
            SHReal ar1 = static_cast<SHReal>(a1.real);
            SHReal ai1 = static_cast<SHReal>(a1.imag);
            SHReal br1 = static_cast<SHReal>(b1.real);
            SHReal bi1 = static_cast<SHReal>(b1.imag);

            SHReal qr1 = ar1 * br1 + ai1 * bi1;
            SHReal qi1 = -ar1 * bi1 + ai1 * br1;
            qRe[n1] = qr1;
            qIm[n1] = qi1;
            pow[n1] = ar1 * ar1 + ai1 * ai1;

            // ----- n+2 -----
            int n2 = n + 2;
            int idx1_2 = off + n2 * sps;
            int idx2_2 = off + (n2 + Lh) * sps;
#ifdef SH_USE_FLOAT
            const mxComplexSingle& a2 = yc[idx1_2];
            const mxComplexSingle& b2 = yc[idx2_2];
#else
            const mxComplexDouble& a2 = yc[idx1_2];
            const mxComplexDouble& b2 = yc[idx2_2];
#endif
            SHReal ar2 = static_cast<SHReal>(a2.real);
            SHReal ai2 = static_cast<SHReal>(a2.imag);
            SHReal br2 = static_cast<SHReal>(b2.real);
            SHReal bi2 = static_cast<SHReal>(b2.imag);

            SHReal qr2 = ar2 * br2 + ai2 * bi2;
            SHReal qi2 = -ar2 * bi2 + ai2 * br2;
            qRe[n2] = qr2;
            qIm[n2] = qi2;
            pow[n2] = ar2 * ar2 + ai2 * ai2;

            // ----- n+3 -----
            int n3 = n + 3;
            int idx1_3 = off + n3 * sps;
            int idx2_3 = off + (n3 + Lh) * sps;
#ifdef SH_USE_FLOAT
            const mxComplexSingle& a3 = yc[idx1_3];
            const mxComplexSingle& b3 = yc[idx2_3];
#else
            const mxComplexDouble& a3 = yc[idx1_3];
            const mxComplexDouble& b3 = yc[idx2_3];
#endif
            SHReal ar3 = static_cast<SHReal>(a3.real);
            SHReal ai3 = static_cast<SHReal>(a3.imag);
            SHReal br3 = static_cast<SHReal>(b3.real);
            SHReal bi3 = static_cast<SHReal>(b3.imag);

            SHReal qr3 = ar3 * br3 + ai3 * bi3;
            SHReal qi3 = -ar3 * bi3 + ai3 * br3;
            qRe[n3] = qr3;
            qIm[n3] = qi3;
            pow[n3] = ar3 * ar3 + ai3 * ai3;
        }

        // Remainder for n = Nq4..Nq-1
        for (; n < Nq; ++n) {
            int idx1 = off + n * sps;
            int idx2 = off + (n + Lh) * sps;
#ifdef SH_USE_FLOAT
            const mxComplexSingle& a = yc[idx1];
            const mxComplexSingle& b = yc[idx2];
#else
            const mxComplexDouble& a = yc[idx1];
            const mxComplexDouble& b = yc[idx2];
#endif
            SHReal ar = static_cast<SHReal>(a.real);
            SHReal ai = static_cast<SHReal>(a.imag);
            SHReal br = static_cast<SHReal>(b.real);
            SHReal bi = static_cast<SHReal>(b.imag);

            SHReal qr = ar * br + ai * bi;
            SHReal qi = -ar * bi + ai * br;

            qRe[n] = qr;
            qIm[n] = qi;
            pow[n] = ar * ar + ai * ai;
        }

        // Now fill pow for n = Nq..Ns-1 (no qRe/qIm needed here)
        int m = Nq;
        int Ns4 = Ns & ~3;   // largest multiple of 4 <= Ns

        for (; m < Ns4; m += 4) {
            int idx0 = off + (m    ) * sps;
            int idx1 = off + (m + 1) * sps;
            int idx2 = off + (m + 2) * sps;
            int idx3 = off + (m + 3) * sps;
#ifdef SH_USE_FLOAT
            const mxComplexSingle& a0 = yc[idx0];
            const mxComplexSingle& a1 = yc[idx1];
            const mxComplexSingle& a2 = yc[idx2];
            const mxComplexSingle& a3 = yc[idx3];
#else
            const mxComplexDouble& a0 = yc[idx0];
            const mxComplexDouble& a1 = yc[idx1];
            const mxComplexDouble& a2 = yc[idx2];
            const mxComplexDouble& a3 = yc[idx3];
#endif
            SHReal ar0 = static_cast<SHReal>(a0.real);
            SHReal ai0 = static_cast<SHReal>(a0.imag);
            SHReal ar1 = static_cast<SHReal>(a1.real);
            SHReal ai1 = static_cast<SHReal>(a1.imag);
            SHReal ar2 = static_cast<SHReal>(a2.real);
            SHReal ai2 = static_cast<SHReal>(a2.imag);
            SHReal ar3 = static_cast<SHReal>(a3.real);
            SHReal ai3 = static_cast<SHReal>(a3.imag);

            pow[m    ] = ar0 * ar0 + ai0 * ai0;
            pow[m + 1] = ar1 * ar1 + ai1 * ai1;
            pow[m + 2] = ar2 * ar2 + ai2 * ai2;
            pow[m + 3] = ar3 * ar3 + ai3 * ai3;
        }

        // Remainder for m = Ns4..Ns-1
        for (; m < Ns; ++m) {
            int idx = off + m * sps;
#ifdef SH_USE_FLOAT
            const mxComplexSingle& a = yc[idx];
#else
            const mxComplexDouble& a = yc[idx];
#endif
            SHReal ar = static_cast<SHReal>(a.real);
            SHReal ai = static_cast<SHReal>(a.imag);
            pow[m]    = ar * ar + ai * ai;
        }

        // ----------------------------------------------------
        // Sliding sums for P(d) over q (length Lh)
        // and R(d) over pow (length 2*Lh)
        // ----------------------------------------------------
        PRe.resize(Lwin);
        PIm.resize(Lwin);
        R.resize(Lwin);

        // P(d) sliding sum over qRe/qIm
        SHVec sumPr_v = SH_SETZERO();
        SHVec sumPi_v = SH_SETZERO();

        int t = 0;
        for (; t + (SH_LANES - 1) < Lh; t += SH_LANES) {
            SHVec re = SH_LOAD(qRe.data() + t);
            SHVec im = SH_LOAD(qIm.data() + t);

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

        // tail data for initial window
        for (; t < Lh; ++t) {
            sumPr += qRe[t];
            sumPi += qIm[t];
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

        // Tail for P(d)
        for (; d < Lwin; ++d) {
            sumPr += qRe[d + Lh - 1] - qRe[d - 1];
            sumPi += qIm[d + Lh - 1] - qIm[d - 1];

            PRe[d] = sumPr;
            PIm[d] = sumPi;
        }

        // R(d) sliding sum over pow, length 2*Lh
        SHVec sumR_v = SH_SETZERO();
        int u = 0;
        for (; u + (SH_LANES - 1) < Lh2; u += SH_LANES) {
            SHVec vv = SH_LOAD(pow.data() + u);
            sumR_v   = SH_ADD(sumR_v, vv);
        }

        SHReal tmpR[SH_LANES];
        SH_STORE(tmpR, sumR_v);
        SHReal sumR = static_cast<SHReal>(0);
        for (int i = 0; i < SH_LANES; ++i) {
            sumR += tmpR[i];
        }

        for (; u < Lh2; ++u) {
            sumR += pow[u];
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

        // Tail for R(d)
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

        // --------- CANDIDATE PICKING + ADVANCED CFO ESTIMATE ----------
        const double PI = 3.14159265358979323846;
        const int CFO_RADIUS = 8;   // number of neighbors on each side to use

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

            // --- Advanced CFO: use neighboring P(d) with unwrap+average ---
            int d0 = k;
            int left  = std::max(0, d0 - CFO_RADIUS);
            int right = std::min(Lwin - 1, d0 + CFO_RADIUS);

            double sumPhi = 0.0;
            int    count  = 0;

            // Start with central window
            SHReal Pr0 = PRe[d0];
            SHReal Pi0 = PIm[d0];
#ifndef FAST_MATH
            double originalPhiPrev = std::atan2(static_cast<double>(Pi0),
                                        static_cast<double>(Pr0));
            double phiPrev = originalPhiPrev;
#else
            double originalPhiPrev = FastArcTan2(static_cast<double>(Pi0),
                                         static_cast<double>(Pr0));
            double phiPrev = originalPhiPrev;
#endif
            sumPhi += phiPrev;
            count   = 1;

            // Walk neighbors, unwrap relative to previous
            for (int dd = d0 - 1; dd >= left; --dd) {
                SHReal Prd = PRe[dd];
                SHReal Pid = PIm[dd];
#ifndef FAST_MATH
                double phi = std::atan2(static_cast<double>(Pid),
                                        static_cast<double>(Prd));
#else
                double phi = FastArcTan2(static_cast<double>(Pid),
                                         static_cast<double>(Prd));
#endif
                double diff = phi - phiPrev;
                if (diff > PI) {
                    phi -= 2.0 * PI;
                } else if (diff < -PI) {
                    phi += 2.0 * PI;
                }
                sumPhi += phi;
                phiPrev = phi;
                ++count;
            }

            // Reset previous to central again for the right side
            phiPrev = originalPhiPrev;

            for (int dd = d0 + 1; dd <= right; ++dd) {
                SHReal Prd = PRe[dd];
                SHReal Pid = PIm[dd];
#ifndef FAST_MATH
                double phi = std::atan2(static_cast<double>(Pid),
                                        static_cast<double>(Prd));
#else
                double phi = FastArcTan2(static_cast<double>(Pid),
                                         static_cast<double>(Prd));
#endif
                double diff = phi - phiPrev;
                if (diff > PI) {
                    phi -= 2.0 * PI;
                } else if (diff < -PI) {
                    phi += 2.0 * PI;
                }
                sumPhi += phi;
                phiPrev = phi;
                ++count;
            }

            double phiMean = sumPhi / static_cast<double>(count);
            double cfoRadPerSym = phiMean / static_cast<double>(Lh);
            // ---------------------------------------------------------

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
