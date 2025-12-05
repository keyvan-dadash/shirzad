#pragma once

#include <cstdint>
#include <vector>
#include <stdexcept>
#include <algorithm>
#include <string>

#include "payload_worker_common.hpp"
#include "utils.hpp"

// Console worker
struct ConsoleWorker : public IWorker {
    void process(const Datagram& dg) override
    {
        try {
            std::size_t nPrev = std::min<std::size_t>(dg.payloadLen, dg.payload.size());
            const std::size_t maxPreview = 64;
            nPrev = std::min<std::size_t>(nPrev, maxPreview);

            std::string ascii;
            ascii.reserve(nPrev);
            for (std::size_t i = 0; i < nPrev; ++i) {
                std::uint8_t c = dg.payload[i];
                if (c < 32 || c > 126) c = '.';
                ascii.push_back(static_cast<char>(c));
            }

            goLog("[DatagramConsoleWorker] StreamId=%u Len=%u Text=\"%s\"",
                  static_cast<unsigned>(dg.streamId),
                  static_cast<unsigned>(dg.payloadLen),
                  ascii.c_str());

        } catch (const std::exception& e) {
            goLog("[DatagramConsoleWorker] ERROR: %s", e.what());
        }
    }
};
