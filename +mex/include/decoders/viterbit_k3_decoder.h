#include <stdint.h>
#include <limits.h>

inline std::vector<std::uint8_t>
viterbi_k3_decode(const std::vector<std::uint8_t>& v)
{
    const std::size_t N = v.size();
    if (N == 0) return {};
    if (N % 2 != 0) {
        throw std::runtime_error("viterbi_k3_decode: input length must be even (2 bits/step).");
    }

    const int T   = static_cast<int>(N / 2); // trellis steps
    const int MEM = 2;
    if (T <= MEM) {
        throw std::runtime_error("viterbi_k3_decode: sequence too short.");
    }

    const int S   = 4;             // 2^MEM states
    const int BIG = 1 << 29;       // large metric

    std::vector<int> PM_prev(S), PM_curr(S);
    std::vector<std::uint8_t> PrevState(T * S);
    std::vector<std::uint8_t> PrevInput(T * S);

    // Start in state 0
    for (int s = 0; s < S; ++s) {
        PM_prev[s] = (s == 0) ? 0 : BIG;
    }

    // Forward DP
    for (int t = 0; t < T; ++t) {
        int r0 = (v[2 * t]     != 0) ? 1 : 0;
        int r1 = (v[2 * t + 1] != 0) ? 1 : 0;

        std::fill(PM_curr.begin(), PM_curr.end(), BIG);

        for (int s_prev = 0; s_prev < S; ++s_prev) {
            int pmOld = PM_prev[s_prev];
            if (pmOld >= BIG) continue;

            std::uint8_t m1 = static_cast<std::uint8_t>((s_prev >> 1) & 1);
            std::uint8_t m2 = static_cast<std::uint8_t>(s_prev & 1);

            for (int b = 0; b <= 1; ++b) {
                std::uint8_t inp = static_cast<std::uint8_t>(b);

                // Generators [111; 101] over [inp, m1, m2]
                std::uint8_t out0 = static_cast<std::uint8_t>((inp ^ m1 ^ m2) & 1);
                std::uint8_t out1 = static_cast<std::uint8_t>((inp ^ m2) & 1);

                std::uint8_t nextState = static_cast<std::uint8_t>((static_cast<int>(inp) << 1) | m1);

                int bm     = ((out0 != r0) ? 1 : 0) + ((out1 != r1) ? 1 : 0);
                int metric = pmOld + bm;

                if (metric < PM_curr[nextState]) {
                    PM_curr[nextState]                   = metric;
                    PrevState[t * S + nextState] = static_cast<std::uint8_t>(s_prev);
                    PrevInput[t * S + nextState] = static_cast<std::uint8_t>(b);
                }
            }
        }

        PM_prev = PM_curr;
    }

    // Pick best ending state (terminated code should end in state 0,
    // but this is more robust).
    int bestState  = 0;
    int bestMetric = PM_prev[0];
    for (int s = 1; s < S; ++s) {
        if (PM_prev[s] < bestMetric) {
            bestMetric = PM_prev[s];
            bestState  = s;
        }
    }

    // Backtrace
    std::vector<std::uint8_t> u_full(T);
    std::uint8_t s = static_cast<std::uint8_t>(bestState);
    for (int t = T - 1; t >= 0; --t) {
        std::uint8_t b = PrevInput[t * S + s];
        u_full[t]      = b;
        s              = PrevState[t * S + s];
    }

    const int infoLen = T - MEM;
    if (infoLen < 0) {
        throw std::runtime_error("viterbi_k3_decode: info length negative.");
    }

    std::vector<std::uint8_t> out(infoLen);
    for (int i = 0; i < infoLen; ++i) {
        out[i] = u_full[i];
    }
    return out;
}
