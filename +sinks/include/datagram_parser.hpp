#pragma once
#include <cstdint>
#include <vector>
#include <stdexcept>
#include <algorithm>

// Datagram is the basic protocol shirzad talks over the channel
struct Datagram {
    static constexpr std::size_t HEADER_BYTES = 5;

    std::uint8_t  streamId   = 0;
    std::uint16_t payloadLen = 0;
    std::uint16_t checksum   = 0;

    std::vector<std::uint8_t> payload;

    static std::uint16_t calcChecksum(const std::vector<std::uint8_t>& bytes)
    {
        std::vector<std::uint8_t> buf = bytes;
        if (buf.size() % 2 != 0) {
            buf.push_back(0);
        }

        std::uint32_t sum = 0;
        for (std::size_t i = 0; i < buf.size(); i += 2) {
            std::uint16_t hi = static_cast<std::uint16_t>(buf[i]);
            std::uint16_t lo = static_cast<std::uint16_t>(buf[i + 1]);
            std::uint16_t w  = static_cast<std::uint16_t>((hi << 8) | lo);

            sum += static_cast<std::uint32_t>(w);
            if (sum > 0xFFFFu) {
                sum = (sum & 0xFFFFu) + 1u;
            }
        }
        return static_cast<std::uint16_t>(~sum);
    }

    static bool fromBytes(const std::vector<std::uint8_t>& bytes,
                          Datagram& out)
    {
        if (bytes.size() < HEADER_BYTES) {
            return false;
        }

        std::uint8_t sid = bytes[0];
        std::uint16_t len =
            (static_cast<std::uint16_t>(bytes[1]) << 8) |
             static_cast<std::uint16_t>(bytes[2]);
        std::uint16_t cks =
            (static_cast<std::uint16_t>(bytes[3]) << 8) |
             static_cast<std::uint16_t>(bytes[4]);

        std::size_t avail = bytes.size() - HEADER_BYTES;
        if (len > avail) {
            len = static_cast<std::uint16_t>(avail);
        }

        out.streamId   = sid;
        out.payloadLen = len;
        out.checksum   = cks;
        out.payload.assign(bytes.begin() + HEADER_BYTES,
                           bytes.begin() + HEADER_BYTES + len);

        // The checksum is calculated when checksum is zero
        std::vector<std::uint8_t> hdrZero;
        hdrZero.reserve(HEADER_BYTES);
        hdrZero.push_back(sid);
        hdrZero.push_back(static_cast<std::uint8_t>((len >> 8) & 0xFF));
        hdrZero.push_back(static_cast<std::uint8_t>(len & 0xFF));
        hdrZero.push_back(0);
        hdrZero.push_back(0);

        std::vector<std::uint8_t> bytesForCksum;
        bytesForCksum.reserve(HEADER_BYTES + len);
        bytesForCksum.insert(bytesForCksum.end(),
                             hdrZero.begin(), hdrZero.end());
        bytesForCksum.insert(bytesForCksum.end(),
                             out.payload.begin(), out.payload.begin() + len);

        std::uint16_t calc = calcChecksum(bytesForCksum);
        return (calc == cks);
    }

    std::vector<std::uint8_t> toBytes() const
    {
        std::uint16_t len = payloadLen;
        if (len > payload.size()) {
            len = static_cast<std::uint16_t>(payload.size());
        }

        std::vector<std::uint8_t> bytes;
        bytes.reserve(HEADER_BYTES + len);

        bytes.push_back(streamId);
        bytes.push_back(static_cast<std::uint8_t>((len >> 8) & 0xFF));
        bytes.push_back(static_cast<std::uint8_t>(len & 0xFF));
        bytes.push_back(0); 
        bytes.push_back(0);

        bytes.insert(bytes.end(),
                     payload.begin(), payload.begin() + len);

        std::uint16_t cks = calcChecksum(bytes);
        bytes[3] = static_cast<std::uint8_t>((cks >> 8) & 0xFF);
        bytes[4] = static_cast<std::uint8_t>(cks & 0xFF);

        return bytes;
    }
};
