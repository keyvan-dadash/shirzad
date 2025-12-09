#pragma once

#include <stdint.h>
#include <limits.h>

inline std::vector<std::uint8_t>
viterbi_k3_7_8_decode(const std::vector<std::uint8_t>& v)
{
    const std::size_t N = v.size();
    if (N == 0) return {};
    if (N % 8 != 0) {
        throw std::runtime_error("viterbi_k3_7_8_decode: coded length must be multiple of 8 (7/8 punctured).");
    }

    const int blocks = static_cast<int>(N / 8);
    const int T      = 7 * blocks;   // trellis steps
    const int MEM    = 2;
    const int S      = 4;
    const int BIG    = 1 << 29;

    // Expand punctured stream into per-step r0,r1 plus masks
    std::vector<std::uint8_t> r0(T), r1(T), has0(T), has1(T);
    const int P0[7] = {1,0,1,0,1,0,1};
    const int P1[7] = {0,1,0,1,0,1,1};

    std::size_t pos = 0;
    for (int t = 0; t < T; ++t) {
        int idx = t % 7;

        if (P0[idx]) {
            if (pos >= N) throw std::runtime_error("viterbi_k3_7_8_decode: stream underflow (P0).");
            has0[t] = 1;
            r0[t]   = (v[pos] != 0) ? 1u : 0u;
            ++pos;
        } else {
            has0[t] = 0;
            r0[t]   = 0;
        }

        if (P1[idx]) {
            if (pos >= N) throw std::runtime_error("viterbi_k3_7_8_decode: stream underflow (P1).");
            has1[t] = 1;
            r1[t]   = (v[pos] != 0) ? 1u : 0u;
            ++pos;
        } else {
            has1[t] = 0;
            r1[t]   = 0;
        }
    }

    if (pos != N) {
        throw std::runtime_error("viterbi_k3_7_8_decode: stream length mismatch after expansion.");
    }

    std::vector<int> PM_prev(S), PM_curr(S);
    std::vector<std::uint8_t> PrevState(static_cast<std::size_t>(T) * S);
    std::vector<std::uint8_t> PrevInput(static_cast<std::size_t>(T) * S);

    for (int s = 0; s < S; ++s) {
        PM_prev[s] = (s == 0) ? 0 : BIG;
    }

    for (int t = 0; t < T; ++t) {
        int rr0 = r0[t];
        int rr1 = r1[t];
        int m0  = has0[t];
        int m1  = has1[t];

        std::fill(PM_curr.begin(), PM_curr.end(), BIG);

        for (int s_prev = 0; s_prev < S; ++s_prev) {
            int pmOld = PM_prev[s_prev];
            if (pmOld >= BIG) continue;

            std::uint8_t m1bit = static_cast<std::uint8_t>((s_prev >> 1) & 1);
            std::uint8_t m2bit = static_cast<std::uint8_t>(s_prev & 1);

            for (int b = 0; b <= 1; ++b) {
                std::uint8_t inp = static_cast<std::uint8_t>(b);

                std::uint8_t out0 = static_cast<std::uint8_t>((inp ^ m1bit ^ m2bit) & 1);
                std::uint8_t out1 = static_cast<std::uint8_t>((inp ^ m2bit) & 1);

                std::uint8_t nextState =
                    static_cast<std::uint8_t>((static_cast<int>(inp) << 1) | m1bit);

                int bm = 0;
                if (m0 && (out0 != rr0)) bm++;
                if (m1 && (out1 != rr1)) bm++;

                int metric = pmOld + bm;

                if (metric < PM_curr[nextState]) {
                    PM_curr[nextState]                 = metric;
                    PrevState[t * S + nextState] = static_cast<std::uint8_t>(s_prev);
                    PrevInput[t * S + nextState] = static_cast<std::uint8_t>(b);
                }
            }
        }

        PM_prev.swap(PM_curr);
    }

    int bestState  = 0;
    int bestMetric = PM_prev[0];
    for (int s = 1; s < S; ++s) {
        if (PM_prev[s] < bestMetric) {
            bestMetric = PM_prev[s];
            bestState  = s;
        }
    }

    std::vector<std::uint8_t> u_full(T);
    std::uint8_t s = static_cast<std::uint8_t>(bestState);
    for (int t = T - 1; t >= 0; --t) {
        std::uint8_t b = PrevInput[t * S + s];
        u_full[t]      = b;
        s              = PrevState[t * S + s];
    }

    const int infoLen = T - MEM;
    if (infoLen < 0) {
        throw std::runtime_error("viterbi_k3_7_8_decode: info length negative.");
    }

    std::vector<std::uint8_t> out(infoLen);
    for (int i = 0; i < infoLen; ++i) {
        out[i] = u_full[i];
    }
    return out;
}
