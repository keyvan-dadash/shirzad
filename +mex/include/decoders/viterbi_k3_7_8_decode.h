#pragma once

#include "mex.h"
#include <vector>
#include <algorithm>
#include <stdint.h>
#include <limits.h>

inline std::vector<std::uint8_t>
viterbi_k3_7_8_decode(const std::vector<std::uint8_t>& v)
{
    const std::size_t N = v.size();
    if (N == 0) {
        return {};
    }

    // 1) Basic length check
    if (N % 8 != 0) {
        goLog(
            "viterbi_k3_7_8_decode:LenNotMultipleOf8",
            "Punctured length N = %zu is not a multiple of 8 for 7/8 code.",
            N);
    }

    const int blocks = static_cast<int>(N / 8);
    const int T      = 7 * blocks;   // trellis steps
    const int MEM    = 2;
    const int S      = 4;
    const int BIG    = 1 << 29;

    if (T <= MEM) {
        goLog(
            "viterbi_k3_7_8_decode:TooShort",
            "T = %d trellis steps is too small (must be > MEM=%d).",
            T, MEM);
    }

    // 2) Expand punctured stream into per-step r0,r1 plus masks
    std::vector<std::uint8_t> r0(T), r1(T), has0(T), has1(T);

    const int P0[7] = {1,1,0,1,0,1,1};
    const int P1[7] = {1,0,1,0,1,0,0};

    int bitsPerPeriod = 0;
    for (int i = 0; i < 7; ++i) {
        bitsPerPeriod += P0[i] + P1[i];
    }
    if (bitsPerPeriod != 8) {
        goLog(
            "viterbi_k3_7_8_decode:PatternBits",
            "Puncture pattern has %d bits per 7 steps (expected 8).",
            bitsPerPeriod);
    }

    std::size_t pos = 0;
    for (int t = 0; t < T; ++t) {
        int idx = t % 7;

        if (P0[idx]) {
            if (pos >= N) {
                goLog(
                    "viterbi_k3_7_8_decode:UnderflowP0",
                    "Stream underflow for g0 at t=%d (pos=%zu, N=%zu).",
                    t, pos, N);
            }
            has0[t] = 1;
            r0[t]   = (v[pos] != 0) ? 1u : 0u;
            ++pos;
        } else {
            has0[t] = 0;
            r0[t]   = 0;
        }

        if (P1[idx]) {
            if (pos >= N) {
                goLog(
                    "viterbi_k3_7_8_decode:UnderflowP1",
                    "Stream underflow for g1 at t=%d (pos=%zu, N=%zu).",
                    t, pos, N);
            }
            has1[t] = 1;
            r1[t]   = (v[pos] != 0) ? 1u : 0u;
            ++pos;
        } else {
            has1[t] = 0;
            r1[t]   = 0;
        }
    }

    if (pos != N) {
        goLog(
            "viterbi_k3_7_8_decode:LengthMismatch",
            "After expansion, pos=%zu but N=%zu (pattern mismatch or wrong N).",
            pos, N);
    }

    // 3) Viterbi data structures
    std::vector<int> PM_prev(S), PM_curr(S);
    std::vector<std::uint8_t> PrevState(static_cast<std::size_t>(T) * S);
    std::vector<std::uint8_t> PrevInput(static_cast<std::size_t>(T) * S);

    for (int s = 0; s < S; ++s) {
        PM_prev[s] = (s == 0) ? 0 : BIG;
    }

    // 4) Viterbi forward recursion
    for (int t = 0; t < T; ++t) {
        int rr0 = r0[t];
        int rr1 = r1[t];
        int m0  = has0[t];
        int m1  = has1[t];

        std::fill(PM_curr.begin(), PM_curr.end(), BIG);

        for (int s_prev = 0; s_prev < S; ++s_prev) {
            int pmOld = PM_prev[s_prev];
            if (pmOld >= BIG) continue;  // unreachable state

            // State bits: [m1bit m2bit]
            std::uint8_t m1bit = static_cast<std::uint8_t>((s_prev >> 1) & 1);
            std::uint8_t m2bit = static_cast<std::uint8_t>(s_prev & 1);

            for (int b = 0; b <= 1; ++b) {
                std::uint8_t inp = static_cast<std::uint8_t>(b);

                std::uint8_t out0 =
                    static_cast<std::uint8_t>((inp ^ m1bit ^ m2bit) & 1);
                std::uint8_t out1 =
                    static_cast<std::uint8_t>((inp ^ m2bit) & 1);

                std::uint8_t nextState =
                    static_cast<std::uint8_t>((static_cast<int>(inp) << 1) | m1bit);

                int bm = 0;
                if (m0 && (out0 != rr0)) bm++;
                if (m1 && (out1 != rr1)) bm++;

                int metric = pmOld + bm;

                if (metric < PM_curr[nextState]) {
                    PM_curr[nextState]           = metric;
                    PrevState[t * S + nextState] = static_cast<std::uint8_t>(s_prev);
                    PrevInput[t * S + nextState] = static_cast<std::uint8_t>(b);
                }
            }
        }

        PM_prev.swap(PM_curr);
    }

    // 5) Pick best ending state
    int bestState  = 0;
    int bestMetric = PM_prev[0];
    for (int s = 1; s < S; ++s) {
        if (PM_prev[s] < bestMetric) {
            bestMetric = PM_prev[s];
            bestState  = s;
        }
    }

    // 6) Backtrace
    std::vector<std::uint8_t> u_full(T);
    std::uint8_t s = static_cast<std::uint8_t>(bestState);
    for (int t = T - 1; t >= 0; --t) {
        std::uint8_t b = PrevInput[t * S + s];
        u_full[t]      = b;
        s              = PrevState[t * S + s];
    }

    const int infoLen = T - MEM;
    if (infoLen <= 0) {
        goLog(
            "viterbi_k3_7_8_decode:InfoLen",
            "infoLen = T - MEM = %d is not positive.", infoLen);
    }

    std::vector<std::uint8_t> out(infoLen);
    for (int i = 0; i < infoLen; ++i) {
        out[i] = u_full[i];
    }
    return out;
}
