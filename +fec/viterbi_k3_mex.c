/*
 * viterbi_k3_mex.c
 *
 * Specialized hard-decision Viterbi decoder for:
 *   - rate-1/2 convolutional code
 *   - constraint length K=3 (memory=2)
 *   - generators G = [111; 101]
 *
 * MATLAB usage:
 *   uHat = viterbi_k3_mex(v);
 *
 * where:
 *   v   : vector of bits (0/1), length N, N must be even (2 bits per trellis step).
 *         Can be double, logical, or uint8. Nonzero -> 1.
 *   uHat: decoded info bits, length T-2 (double 0/1),
 *         where T = N/2 is the number of trellis steps.
 *
 * The encoder is assumed to terminate by appending 2 zeros.
 * This matches fec.ConvEncoder.rateHalf_K3 + fec.ViterbiDecoder.rateHalf_K3.
 */

#include "mex.h"
#include <stdint.h>
#include <limits.h>

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    if (nrhs != 1) {
        mexErrMsgIdAndTxt("fec:viterbi_k3:NumInputs",
                          "Usage: uHat = viterbi_k3_mex(v)");
    }
    if (nlhs > 1) {
        mexErrMsgIdAndTxt("fec:viterbi_k3:NumOutputs",
                          "One output argument expected.");
    }

    const mxArray *vIn = prhs[0];
    mwSize N = mxGetNumberOfElements(vIn);

    if (N == 0) {
        plhs[0] = mxCreateDoubleMatrix(0, 0, mxREAL);
        return;
    }

    if (N % 2 != 0) {
        mexErrMsgIdAndTxt("fec:viterbi_k3:Length",
                          "Input length must be even (2 coded bits per trellis step).");
    }

    int T = (int)(N / 2);  /* trellis steps */
    const int MEM = 2;
    if (T <= MEM) {
        mexErrMsgIdAndTxt("fec:viterbi_k3:TooShort",
                          "Sequence too short: T=%d, needs > %d.", T, MEM);
    }

    /* Convert v to rx bits in {0,1} */
    uint8_t *rx = (uint8_t*)mxCalloc(N, sizeof(uint8_t));

    if (mxIsLogical(vIn)) {
        const mxLogical *p = mxGetLogicals(vIn);
        for (mwSize i = 0; i < N; ++i) {
            rx[i] = p[i] ? 1 : 0;
        }
    } else if (mxIsUint8(vIn)) {
        const uint8_t *p = (const uint8_t*)mxGetData(vIn);
        for (mwSize i = 0; i < N; ++i) {
            rx[i] = (p[i] != 0) ? 1 : 0;
        }
    } else if (mxIsDouble(vIn)) {
        const double *p = mxGetPr(vIn);
        for (mwSize i = 0; i < N; ++i) {
            rx[i] = (p[i] != 0.0) ? 1 : 0;
        }
    } else {
        mxFree(rx);
        mexErrMsgIdAndTxt("fec:viterbi_k3:InputType",
                          "v must be logical, uint8, or double.");
    }

    /* Trellis: K=3, memory=2, 4 states (00,01,10,11) */
    const int S = 4;
    const int BIG = INT_MAX / 4;  /* large metric */

    int PM_prev[S];
    int PM_curr[S];

    /* PrevState[t*S + state], PrevInput[t*S + state] */
    uint8_t *PrevState = (uint8_t*)mxCalloc((mwSize)T * S, sizeof(uint8_t));
    uint8_t *PrevInput = (uint8_t*)mxCalloc((mwSize)T * S, sizeof(uint8_t));

    /* Initialize path metrics: start from state 0 with metric 0 */
    for (int s = 0; s < S; ++s) {
        PM_prev[s] = (s == 0) ? 0 : BIG;
    }

    /* --- Forward DP --- */
    for (int t = 0; t < T; ++t) {
        int r0 = rx[2 * t];
        int r1 = rx[2 * t + 1];

        for (int s = 0; s < S; ++s) {
            PM_curr[s] = BIG;
        }

        for (int s_prev = 0; s_prev < S; ++s_prev) {
            int pmOld = PM_prev[s_prev];
            if (pmOld >= BIG) {
                continue;
            }

            uint8_t m1 = (uint8_t)((s_prev >> 1) & 1);
            uint8_t m2 = (uint8_t)(s_prev & 1);

            for (int b = 0; b <= 1; ++b) {
                uint8_t inp  = (uint8_t)b;

                /* Generators [111; 101] over [inp, m1, m2] */
                uint8_t out0 = (uint8_t)((inp ^ m1 ^ m2) & 1);
                uint8_t out1 = (uint8_t)((inp ^ m2) & 1);

                uint8_t nextState = (uint8_t)(((int)inp << 1) | m1);

                /* Hamming distance branch metric */
                int bm = ((out0 != r0) ? 1 : 0) + ((out1 != r1) ? 1 : 0);
                int metric = pmOld + bm;

                if (metric < PM_curr[nextState]) {
                    PM_curr[nextState]                   = metric;
                    PrevState[(mwSize)t * S + nextState] = (uint8_t)s_prev;
                    PrevInput[(mwSize)t * S + nextState] = (uint8_t)b;
                }
            }
        }

        /* Swap PM_prev and PM_curr */
        for (int s = 0; s < S; ++s) {
            PM_prev[s] = PM_curr[s];
        }
    }

    /* For a terminated code, final state should be 0.
     * But we can still pick the best among all states.
     */
    int bestState = 0;
    int bestMetric = PM_prev[0];
    for (int s = 1; s < S; ++s) {
        if (PM_prev[s] < bestMetric) {
            bestMetric = PM_prev[s];
            bestState  = s;
        }
    }

    /* --- Backtrace --- */
    uint8_t *u_full = (uint8_t*)mxCalloc(T, sizeof(uint8_t));

    uint8_t s = (uint8_t)bestState;
    for (int t = T - 1; t >= 0; --t) {
        uint8_t b = PrevInput[(mwSize)t * S + s];
        u_full[t] = b;
        s         = PrevState[(mwSize)t * S + s];
    }

    int infoLen = T - MEM;
    if (infoLen < 0) {
        mxFree(rx);
        mxFree(PrevState);
        mxFree(PrevInput);
        mxFree(u_full);
        mexErrMsgIdAndTxt("fec:viterbi_k3:InfoLen",
                          "Info length became negative; T=%d, MEM=%d.", T, MEM);
    }

    /* Create MATLAB output, as double 0/1 */
    plhs[0] = mxCreateDoubleMatrix((mwSize)infoLen, 1, mxREAL);
    double *out = mxGetPr(plhs[0]);
    for (int i = 0; i < infoLen; ++i) {
        out[i] = (double)u_full[i];  /* first T-MEM bits */
    }

    mxFree(rx);
    mxFree(PrevState);
    mxFree(PrevInput);
    mxFree(u_full);
}
