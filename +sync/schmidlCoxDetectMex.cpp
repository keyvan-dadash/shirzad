#include "mex.h"
#include <vector>
#include <cmath>
#include <cfloat>
#include <immintrin.h>

static void createResultStruct(mxArray*& out,
                               double metric,
                               int sampleOffset,
                               int preambleStartSym,
                               double windowPower,
                               double cfoRadPerSym,
                               bool found)
{
    const char* fieldNames[] = {
        "Metric",
        "SampleOffset",
        "PreambleStartSym",
        "WindowPower",
        "Found",
        "CfoRadPerSym"
    };
    constexpr int nFields = 6;

    out = mxCreateStructMatrix(1, 1, nFields, fieldNames);

    mxSetFieldByNumber(out, 0, 0, mxCreateDoubleScalar(metric));
    mxSetFieldByNumber(out, 0, 1, mxCreateDoubleScalar(static_cast<double>(sampleOffset)));
    mxSetFieldByNumber(out, 0, 2, mxCreateDoubleScalar(static_cast<double>(preambleStartSym)));
    mxSetFieldByNumber(out, 0, 3, mxCreateDoubleScalar(windowPower));
    mxSetFieldByNumber(out, 0, 4, mxCreateLogicalScalar(found));
    mxSetFieldByNumber(out, 0, 5, mxCreateDoubleScalar(cfoRadPerSym));
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
                          "One output (struct) expected.");
    }

    const mxArray* y_in = prhs[0];
    if (!mxIsDouble(y_in)) {
        mexErrMsgIdAndTxt("schmidlCoxDetectMex:InvalidY",
                          "Input y must be double (real or complex).");
    }

    const mwSize N = mxGetNumberOfElements(y_in);
    const bool isComplex = mxIsComplex(y_in);

    mxComplexDouble* yc = nullptr;
    double* yr = nullptr;

    if (isComplex) {
        yc = mxGetComplexDoubles(y_in);  // interleaved complex
    } else {
        yr = mxGetDoubles(y_in);         // purely real
    }

    int sps = static_cast<int>(mxGetScalar(prhs[1]));
    int Lh  = static_cast<int>(mxGetScalar(prhs[2]));
    double metricThreshold = mxGetScalar(prhs[3]);
    double minWindowPower  = mxGetScalar(prhs[4]);

    if (sps <= 0 || Lh <= 0) {
        mexErrMsgIdAndTxt("schmidlCoxDetectMex:InvalidParams",
                          "sps and Lh must be positive.");
    }

    double bestMetric         = 0.0;
    int    bestSampleOffset   = 0;
    int    bestPreambleStart  = 0;  // 1-based
    double bestWindowPower    = 0.0;
    double bestCfoRadPerSym   = 0.0;
    bool   found              = false;

    if (N == 0) {
        createResultStruct(plhs[0],
                           bestMetric,
                           bestSampleOffset,
                           bestPreambleStart,
                           bestWindowPower,
                           bestCfoRadPerSym,
                           false);
        return;
    }

    // Workspace; sizes adjusted per offset
    std::vector<double> ySymRe;
    std::vector<double> ySymIm;
    std::vector<double> qRe, qIm;
    std::vector<double> pow;
    std::vector<double> PRe, PIm;
    std::vector<double> R;

    ySymRe.reserve(N);
    ySymIm.reserve(N);
    qRe.reserve(N);
    qIm.reserve(N);
    pow.reserve(N);
    PRe.reserve(N);
    PIm.reserve(N);
    R.reserve(N);

    const int Nint = static_cast<int>(N);

    // Main Schmidl-Cox loop: over all sample offsets 0 .. sps-1
    for (int off = 0; off < sps; ++off) {
        if (off >= Nint) {
            break;  // no more samples at this offset
        }

        // Ns = number of symbols at this offset
        int Ns = 1 + (Nint - 1 - off) / sps;
        if (Ns < (2 * Lh + 1)) {
            continue;
        }

        const int Lwin = Ns - 2 * Lh;
        if (Lwin <= 0) {
            continue;
        }

        ySymRe.resize(Ns);
        ySymIm.resize(Ns);

        for (int n = 0, idx = off; n < Ns; ++n, idx += sps) {
            ySymRe[n] = yc[idx].real;
            ySymIm[n] = yc[idx].imag;
        }

        const int Nq = Ns - Lh;
        if (Nq <= 0) {
            continue;
        }

        qRe.resize(Nq);
        qIm.resize(Nq);
        pow.resize(Ns);

        //  Compute q(n) = y(n)*conj(y(n+Lh)) and pow(n)=|y(n)|^2
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
            __m256d qi   = _mm256_sub_pd(aibr, arbi);  // qi = ai*br - ar*bi

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

        // pow
        n = 0;
        for (; n + 3 < Ns; n += 4) {
            __m256d ar = _mm256_loadu_pd(ySymRe.data() + n);
            __m256d ai = _mm256_loadu_pd(ySymIm.data() + n);

            __m256d ar2  = _mm256_mul_pd(ar, ar);
            __m256d ai2  = _mm256_mul_pd(ai, ai);
            __m256d mag2 = _mm256_add_pd(ar2, ai2);

            _mm256_storeu_pd(pow.data() + n, mag2);
        }

        // tail for pow
        for (; n < Ns; ++n) {
            double ar = ySymRe[n];
            double ai = ySymIm[n];
            pow[n] = ar * ar + ai * ai;
        }

        const int Lh2 = 2 * Lh;

        PRe.resize(Lwin);
        PIm.resize(Lwin);
        R.resize(Lwin);

        // Sliding sum for P(d) over q (length Lh)
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

        // tail for sum
        for (; d < Lwin; ++d) {
            sumPr += qRe[d + Lh - 1] - qRe[d - 1];
            sumPi += qIm[d + Lh - 1] - qIm[d - 1];

            PRe[d] = sumPr;
            PIm[d] = sumPi;
        }

        // Sliding sum for R(d) over pow (length 2*Lh)
        __m256d sumR_v = _mm256_setzero_pd();
        m = 0;

        for (; m + 3 < Lh2; m += 4) {
            __m256d v = _mm256_loadu_pd(pow.data() + m);
            sumR_v = _mm256_add_pd(sumR_v, v);
        }

        double tmp[4];
        _mm256_storeu_pd(tmp, sumR_v);
        double sumR = tmp[0] + tmp[1] + tmp[2] + tmp[3];

        // tail for sum
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

        // Compute M(d) and track best candidate
        int k = 0;

        // Constants as vectors
        __m256d eps_v          = _mm256_set1_pd(DBL_EPSILON);
        __m256d minPower_v     = _mm256_set1_pd(minWindowPower);
        __m256d metricThresh_v = _mm256_set1_pd(metricThreshold);
        
        // Process 4 k's at a time
        for (; k + 3 < Lwin; k += 4) {
            // Load R, PRe, PIm
            __m256d Rv_v  = _mm256_loadu_pd(R.data()   + k);
            __m256d Pr_v  = _mm256_loadu_pd(PRe.data() + k);
            __m256d Pi_v  = _mm256_loadu_pd(PIm.data() + k);
        
            // magP2 = Pr*Pr + Pi*Pi
            __m256d Pr2_v   = _mm256_mul_pd(Pr_v, Pr_v);
            __m256d Pi2_v   = _mm256_mul_pd(Pi_v, Pi_v);
            __m256d magP2_v = _mm256_add_pd(Pr2_v, Pi2_v);
        
            // denom = Rv*Rv + DBL_EPSILON
            __m256d R2_v    = _mm256_mul_pd(Rv_v, Rv_v);
            __m256d denom_v = _mm256_add_pd(R2_v, eps_v);
        
            // metric = magP2 / denom
            __m256d metric_v = _mm256_div_pd(magP2_v, denom_v);
        
            // Store to temporaries so we can handle the branching scalarly
            double Rv_arr[4];
            double Pr_arr[4];
            double Pi_arr[4];
            double metric_arr[4];
        
            _mm256_storeu_pd(Rv_arr,     Rv_v);
            _mm256_storeu_pd(Pr_arr,     Pr_v);
            _mm256_storeu_pd(Pi_arr,     Pi_v);
            _mm256_storeu_pd(metric_arr, metric_v);
        
            // Now handle 4 lanes one by one
            for (int i = 0; i < 4; ++i) {
                double Rv     = Rv_arr[i];
                double metric = metric_arr[i];
        
                if (Rv <= minWindowPower) {
                    continue;
                }
        
                if (metric <= metricThreshold || metric <= bestMetric) {
                    continue;
                }
        
                double Pr = Pr_arr[i];
                double Pi = Pi_arr[i];
        
                bestMetric        = metric;
                bestWindowPower   = Rv;
                bestSampleOffset  = off;
                int idx           = k + i;
                bestPreambleStart = idx + 1;
                bestCfoRadPerSym  = std::atan2(Pi, Pr) / static_cast<double>(Lh);
                found             = true;
            }
        }
        
        // Scalar tail for leftover k (if Lwin % 4 != 0)
        for (; k < Lwin; ++k) {
            double Rv = R[k];
            if (Rv <= minWindowPower) {
                continue;
            }
        
            double Pr = PRe[k];
            double Pi = PIm[k];
        
            double magP2  = Pr * Pr + Pi * Pi;
            double denom  = Rv * Rv + DBL_EPSILON;
            double metric = magP2 / denom;
        
            if (metric > metricThreshold && metric > bestMetric) {
                bestMetric        = metric;
                bestWindowPower   = Rv;
                bestSampleOffset  = off;
                bestPreambleStart = k + 1;
                bestCfoRadPerSym  = std::atan2(Pi, Pr) / static_cast<double>(Lh);
                found             = true;
            }
        }
    }

    createResultStruct(plhs[0],
                       bestMetric,
                       bestSampleOffset,
                       bestPreambleStart,
                       bestWindowPower,
                       bestCfoRadPerSym,
                       found);
}
