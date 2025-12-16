#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <algorithm>
#include <cstring>

#include "payload_worker_common.hpp"
#include "utils.hpp"

// Payload that should match the TX side so we can calculate the ber.
#define FIXED_EXPECTED_PAYLOAD \
"Lorem ipsum dolor sit amet, consectetur adipiscing elit. Sed dictum augue sed lectus finibus tempor. Nulla eros risus, congue sit amet arcu vitae, porttitor molestie ipsum. Orci varius natoque penatibus et magnis dis parturient montes, nascetur ridiculus mus. Nunc iaculis eget ligula non consectetur. Curabitur lacus turpis, molestie cursus pellentesque non, scelerisque eget dui. Morbi vel malesuada odio, vitae lacinia urna. Ut iaculis neque eu blandit dignissim. Mauris pretium lacus metus, in euismod dui pulvinar nec. Nulla placerat auctor diam, vel dignissim erat ultricies vitae. Vestibulum malesuada neque leo, eu mattis dui eleifend id. Morbi vel commodo justo, quis volutpat lacus. Aliquam mollis nunc ante, maximus tristique eu."

struct FixedPayloadBerWorker : public IWorker {
    std::uint64_t pktCount    = 0;
    std::uint64_t goodPkts    = 0;
    std::uint64_t lenMismatch = 0;
    std::uint64_t emptyPkts   = 0;

    std::uint64_t bitErrors   = 0;
    std::uint64_t totalBits   = 0;

    // Build expected bytes once (ASCII)
    static inline const std::vector<std::uint8_t>& expected() {
        static const std::vector<std::uint8_t> exp = [] {
            const char* s = FIXED_EXPECTED_PAYLOAD;
            const std::size_t n = std::strlen(s);
            std::vector<std::uint8_t> v;
            v.reserve(n);
            for (std::size_t i = 0; i < n; ++i) {
                v.push_back(static_cast<std::uint8_t>(s[i]));
            }
            return v;
        }();
        return exp;
    }

    static inline std::uint32_t popcnt8(std::uint8_t x) {
#if defined(__GNUC__) || defined(__clang__)
        return static_cast<std::uint32_t>(__builtin_popcount(static_cast<unsigned>(x)));
#else
        // portable popcount
        x = x - ((x >> 1) & 0x55);
        x = (x & 0x33) + ((x >> 2) & 0x33);
        return (((x + (x >> 4)) & 0x0F) * 0x01);
#endif
    }

    void process(const Datagram& dg) override {
        pktCount++;

        const auto& exp = expected();
        const std::size_t nEx = exp.size();

        const std::size_t rxVecSize = dg.payload.size();
        std::size_t nRx = rxVecSize;

        if (dg.payloadLen != 0 && static_cast<std::size_t>(dg.payloadLen) < nRx) {
            nRx = static_cast<std::size_t>(dg.payloadLen);
        }

        if (nRx == 0) {
            emptyPkts++;
            if ((pktCount % 1000ull) == 0ull) {
                double per = 1.0 - (double(goodPkts) / double(pktCount));
                goLog("[FixedPayloadBER] StreamId=%u pkts=%llu good=%llu lenMis=%llu empty=%llu "
                      "BER=NA (no comparable bits yet) PER=%.3e expectedLen=%llu "
                      "payloadLenField=%u payloadVecSize=%llu",
                      (unsigned)dg.streamId,
                      (unsigned long long)pktCount,
                      (unsigned long long)goodPkts,
                      (unsigned long long)lenMismatch,
                      (unsigned long long)emptyPkts,
                      per,
                      (unsigned long long)nEx,
                      (unsigned)dg.payloadLen,
                      (unsigned long long)rxVecSize);
            }
            return;
        }

        const std::size_t nMin = std::min(nRx, nEx);

        std::uint64_t errThis = 0;
        for (std::size_t i = 0; i < nMin; ++i) {
            errThis += popcnt8(static_cast<std::uint8_t>(dg.payload[i] ^ exp[i]));
        }

        const std::uint64_t bitsThis = 8ull * static_cast<std::uint64_t>(nMin);

        if (nRx != nEx) {
            lenMismatch++;
        }

        bitErrors += errThis;
        totalBits += bitsThis;

        if (errThis == 0 && nRx == nEx) {
            goodPkts++;
        }

        if ((pktCount % 1000ull) == 0ull) {
            double ber = (totalBits > 0) ? (double(bitErrors) / double(totalBits)) : 0.0;
            double per = 1.0 - (double(goodPkts) / double(pktCount));
            std::uint64_t correctBits = (totalBits >= bitErrors) ? (totalBits - bitErrors) : 0;

            goLog("[FixedPayloadBER] StreamId=%u pkts=%llu good=%llu lenMis=%llu empty=%llu "
                  "BER=%.3e (err=%llu / tot=%llu, correct=%llu) PER=%.3e "
                  "expectedLen=%llu rxLenUsed=%llu payloadLenField=%u payloadVecSize=%llu",
                  (unsigned)dg.streamId,
                  (unsigned long long)pktCount,
                  (unsigned long long)goodPkts,
                  (unsigned long long)lenMismatch,
                  (unsigned long long)emptyPkts,
                  ber,
                  (unsigned long long)bitErrors,
                  (unsigned long long)totalBits,
                  (unsigned long long)correctBits,
                  per,
                  (unsigned long long)nEx,
                  (unsigned long long)nRx,
                  (unsigned)dg.payloadLen,
                  (unsigned long long)rxVecSize);
        }
    }
};
