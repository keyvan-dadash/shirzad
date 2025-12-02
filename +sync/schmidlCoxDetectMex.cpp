#include "mex.h"
#include <vector>
#include <cmath>
#include <cfloat>
#include <immintrin.h>
#include <algorithm>

// Candidate struct to mirror MATLAB detectCandidates fields
struct Candidate
{
    double startSample;      // StartSample (1-based, in samples)
    int    sampleOffset;     // SampleOffset (0..sps-1)
    int    preambleStartSym; // PreambleStartSym (1-based, in symbols)
    double metric;           // M(d)
    double windowPower;      // R(d)
    double cfoRadPerSym;     // CFO estimate per symbol (rad/sym)
};

// Create an empty or filled struct array in MATLAB
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
    if (!mxIsDouble(y_in)) {
        mexErrMsgIdAndTxt("schmidlCoxDetectMex:InvalidY",
                          "Input y must be double (real or complex).");
    }

    const mwSize N = mxGetNumberOfElements(y_in);
    const bool isComplex = mxIsComplex(y_in);

    mxComplexDouble* yc = nullptr;
    double*          yr = nullptr;

    if (isComplex) {
        yc = mxGetComplexDoubles(y_in);  // interleaved complex
    } else {
        yr = mxGetDoubles(y_in);         // purely real, imag=0
    }

    int sps = static_cast<int>(mxGetScalar(prhs[1]));
    int Lh  = static_cast<int>(mxGetScalar(prhs[2]));
    double metricThreshold = mxGetScalar(prhs[3]);
    double minWindowPower  = mxGetScalar(prhs[4]);

    if (sps <= 0 || Lh <= 0) {
        mexErrMsgIdAndTxt("schmidlCoxDetectMex:InvalidParams",
                          "sps and Lh must be positive.");
    }

    // If no samples, immediately return empty struct array
    if (N == 0) {
        plhs[0] = createCandidateStructArray(0);
        return;
    }

    const int Nint = static_cast<int>(N);
    const int Lh2  = 2 * Lh;            // full preamble length in symbols
    const double Lpre      = 2.0 * static_cast<double>(Lh);
    const double maxSpan   = Lpre / 2.0; // grouping span in *samples* (Lpre/2)

    // Workspaces reused for each offset
    std::vector<double> ySymRe;
    std::vector<double> ySymIm;
    std::vector<double> qRe, qIm;
    std::vector<double> pow;
    std::vector<double> PRe, PIm;
    std::vector<double> R;
    std::vector<double> M;   // Metric per k

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

    // Main Schmidl–Cox loop over sample offsets 0..sps-1
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
                ySymRe[n] = yc[idx].real;
                ySymIm[n] = yc[idx].imag;
            } else {
                ySymRe[n] = yr[idx];
                ySymIm[n] = 0.0;
            }
        }

        const int Nq = Ns - Lh;
        if (Nq <= 0) {
            continue;
        }

        qRe.resize(Nq);
        qIm.resize(Nq);
        pow.resize(Ns);

        // q(n) = y(n) * conj(y(n+Lh)), pow(n) = |y(n)|^2, with AVX where possible
        int n = 0;
        for (; n + 3 < Nq; n += 4) {
            __m256d ar = _mm256_loadu_pd(ySymRe.data() + n);
            __m256d ai = _mm256_loadu_pd(ySymIm.data() + n);

            __m256d br = _mm256_loadu_pd(ySymRe.data() + n + Lh);
            __m256d bi = _mm256_loadu_pd(ySymIm.data() + n + Lh);

            __m256d arbr = _mm256_mul_pd(ar, br);
            __m256d aibi = _mm256_mul_pd(ai, bi);
            __m256d qr   = _mm256_add_pd(arbr, aibi);

            __m256d aibr = _mm256_mul_pd(ai, br);
            __m256d arbi = _mm256_mul_pd(ar, bi);
            __m256d qi   = _mm256_sub_pd(aibr, arbi);  // imag: ai*br - ar*bi

            _mm256_storeu_pd(qRe.data() + n, qr);
            _mm256_storeu_pd(qIm.data() + n, qi);
        }
        // tail for q
        for (; n < Nq; ++n) {
            double ar = ySymRe[n];
            double ai = ySymIm[n];
            double br = ySymRe[n + Lh];
            double bi = ySymIm[n + Lh];

            double qr = ar * br + ai * bi;
            double qi = -ar * bi + ai * br;

            qRe[n] = qr;
            qIm[n] = qi;
        }

        // pow(n) = |y(n)|^2
        n = 0;
        for (; n + 3 < Ns; n += 4) {
            __m256d ar = _mm256_loadu_pd(ySymRe.data() + n);
            __m256d ai = _mm256_loadu_pd(ySymIm.data() + n);

            __m256d ar2  = _mm256_mul_pd(ar, ar);
            __m256d ai2  = _mm256_mul_pd(ai, ai);
            __m256d mag2 = _mm256_add_pd(ar2, ai2);

            _mm256_storeu_pd(pow.data() + n, mag2);
        }
        for (; n < Ns; ++n) {
            double ar = ySymRe[n];
            double ai = ySymIm[n];
            pow[n]    = ar * ar + ai * ai;
        }

        // Sliding sums for P(d) over q (length Lh) and R(d) over pow (length 2*Lh)
        PRe.resize(Lwin);
        PIm.resize(Lwin);
        R.resize(Lwin);

        // P(d) sliding sum
        __m256d sumPr_v = _mm256_setzero_pd();
        __m256d sumPi_v = _mm256_setzero_pd();

        int m = 0;
        for (; m + 3 < Lh; m += 4) {
            __m256d re = _mm256_loadu_pd(qRe.data() + m);
            __m256d im = _mm256_loadu_pd(qIm.data() + m);

            sumPr_v = _mm256_add_pd(sumPr_v, re);
            sumPi_v = _mm256_add_pd(sumPi_v, im);
        }

        double tmpPr[4];
        double tmpPi[4];
        _mm256_storeu_pd(tmpPr, sumPr_v);
        _mm256_storeu_pd(tmpPi, sumPi_v);

        double sumPr = tmpPr[0] + tmpPr[1] + tmpPr[2] + tmpPr[3];
        double sumPi = tmpPi[0] + tmpPi[1] + tmpPi[2] + tmpPi[3];

        for (; m < Lh; ++m) {
            sumPr += qRe[m];
            sumPi += qIm[m];
        }

        PRe[0] = sumPr;
        PIm[0] = sumPi;

        int d = 1;
        for (; d + 3 < Lwin; d += 4) {
            __m256d hiRe = _mm256_loadu_pd(qRe.data() + (d + Lh - 1));
            __m256d hiIm = _mm256_loadu_pd(qIm.data() + (d + Lh - 1));

            __m256d loRe = _mm256_loadu_pd(qRe.data() + (d - 1));
            __m256d loIm = _mm256_loadu_pd(qIm.data() + (d - 1));

            __m256d diffRe_v = _mm256_sub_pd(hiRe, loRe);
            __m256d diffIm_v = _mm256_sub_pd(hiIm, loIm);

            double diffRe[4];
            double diffIm[4];
            _mm256_storeu_pd(diffRe, diffRe_v);
            _mm256_storeu_pd(diffIm, diffIm_v);

            for (int i = 0; i < 4; ++i) {
                sumPr += diffRe[i];
                sumPi += diffIm[i];

                int di = d + i;
                PRe[di] = sumPr;
                PIm[di] = sumPi;
            }
        }
        for (; d < Lwin; ++d) {
            sumPr += qRe[d + Lh - 1] - qRe[d - 1];
            sumPi += qIm[d + Lh - 1] - qIm[d - 1];

            PRe[d] = sumPr;
            PIm[d] = sumPi;
        }

        // R(d) sliding sum over pow, length 2*Lh
        __m256d sumR_v = _mm256_setzero_pd();
        m = 0;
        for (; m + 3 < Lh2; m += 4) {
            __m256d v = _mm256_loadu_pd(pow.data() + m);
            sumR_v = _mm256_add_pd(sumR_v, v);
        }

        double tmpR[4];
        _mm256_storeu_pd(tmpR, sumR_v);
        double sumR = tmpR[0] + tmpR[1] + tmpR[2] + tmpR[3];

        for (; m < Lh2; ++m) {
            sumR += pow[m];
        }

        R[0] = sumR;

        d = 1;
        for (; d + 3 < Lwin; d += 4) {
            __m256d hi = _mm256_loadu_pd(pow.data() + (d + Lh2 - 1));
            __m256d lo = _mm256_loadu_pd(pow.data() + (d - 1));

            __m256d diff_v = _mm256_sub_pd(hi, lo);

            double diff[4];
            _mm256_storeu_pd(diff, diff_v);

            for (int i = 0; i < 4; ++i) {
                sumR += diff[i];
                int di = d + i;
                R[di] = sumR;
            }
        }
        for (; d < Lwin; ++d) {
            sumR += pow[d + Lh2 - 1] - pow[d - 1];
            R[d] = sumR;
        }

        // ----- Compute M(d) scalar and pick local maxima as candidates -----
        M.resize(Lwin);
        for (int k = 0; k < Lwin; ++k) {
            double Rv = R[k];
            double Pr = PRe[k];
            double Pi = PIm[k];

            double magP2  = Pr * Pr + Pi * Pi;
            double denom  = Rv * Rv + DBL_EPSILON;
            M[k]          = magP2 / denom;
        }

        for (int k = 0; k < Lwin; ++k) {
            double Rv     = R[k];
            double metric = M[k];

            if (Rv <= minWindowPower) {
                continue;
            }
            if (metric <= metricThreshold) {
                continue;
            }

            // Local maxima condition: M(k) > M(k-1) and M(k) > M(k+1)
            if (k > 0 && metric <= M[k - 1]) {
                continue;
            }
            if (k < Lwin - 1 && metric <= M[k + 1]) {
                continue;
            }

            double Pr = PRe[k];
            double Pi = PIm[k];

            double phi         = std::atan2(Pi, Pr);
            double cfoRadPerSym = phi / static_cast<double>(Lh);

            // Convert (off, k) to absolute sample index (1-based) in y
            double startSample = 1.0 + static_cast<double>(off) +
                                 static_cast<double>(k) * static_cast<double>(sps);

            Candidate c;
            c.startSample      = startSample;
            c.sampleOffset     = off;
            c.preambleStartSym = k + 1;  // 1-based
            c.metric           = metric;
            c.windowPower      = Rv;
            c.cfoRadPerSym     = cfoRadPerSym;

            candRaw.push_back(c);
        }
    }

    // ----- If no candidates, return empty struct -----
    if (candRaw.empty()) {
        plhs[0] = createCandidateStructArray(0);
        return;
    }

    // ----- Sort candidates by StartSample (time order) -----
    std::sort(candRaw.begin(), candRaw.end(),
              [](const Candidate& a, const Candidate& b) {
                  return a.startSample < b.startSample;
              });

    // ----- Merge nearby candidates (span <= Lpre/2 in samples) -----
    std::vector<Candidate> candMerged;
    candMerged.reserve(candRaw.size());

    const int Ncand = static_cast<int>(candRaw.size());

    int    groupStart = 0;
    double dist       = 0.0;

    for (int i = 1; i < Ncand; ++i) {
        double gap = candRaw[i].startSample - candRaw[i - 1].startSample;

        if (dist + gap <= maxSpan) {
            // Still within same preamble plateau
            dist += gap;
        } else {
            // Close group [groupStart .. i-1]
            int bestIdx   = groupStart;
            double bestM  = candRaw[groupStart].metric;
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
        int bestIdx   = groupStart;
        double bestM  = candRaw[groupStart].metric;
        for (int j = groupStart + 1; j <= Ncand - 1; ++j) {
            if (candRaw[j].metric > bestM) {
                bestM   = candRaw[j].metric;
                bestIdx = j;
            }
        }
        candMerged.push_back(candRaw[bestIdx]);
    }

    // ----- Build MATLAB struct array -----
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
