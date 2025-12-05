#pragma once

#include <cstdio> 
#include <cstdarg>
#include <cstring>
#include <mutex>
#include <vector>
#include <cstdint>
#include <stdexcept>

// removeFile is a file to remove the prevous log file upon start up
inline void removeFile(const char* filename)
{
    static FILE* file = nullptr;
    if (file) {
        return;
    }

    char pathbuf[1024];

    std::snprintf(pathbuf, sizeof(pathbuf), "%s", __FILE__);

    char* lastSlash = std::strrchr(pathbuf, '\\');

    if (lastSlash) {
        *(lastSlash + 1) = '\0';
        std::strncat(pathbuf, filename,
                     sizeof(pathbuf) - std::strlen(pathbuf) - 1);
    } else {
        std::snprintf(pathbuf, sizeof(pathbuf), "%s", filename);
    }

    std::remove(pathbuf);
}

// getLogFile from filename
inline FILE* getLogFile(const char* filename)
{
    static std::unordered_map<std::string, FILE*> files;

    auto it = files.find(filename);
    if (it != files.end()) {
        return it->second;
    }

    FILE* f = std::fopen(filename, "w");
    if (!f) {
        return nullptr;
    }

    files.emplace(filename, f);
    return f;
}

// Thread-safe loging mechanism
inline void goLog(const char* fmt, ...)
{
    static std::mutex logMutex;
    std::lock_guard<std::mutex> lock(logMutex);

    FILE* f = getLogFile("log.txt");
    if (!f) {
        return;
    }

    va_list args;
    va_start(args, fmt);
    std::vfprintf(f, fmt, args);
    std::fprintf(f, "\n");
    std::fflush(f);
    va_end(args);
}

// goLogWithoutLock logs without any log
inline void goLogWithoutLock(const char* filename, const char* fmt, ...)
{
    FILE* f = getLogFile(filename);
    if (!f) {
        return;
    }

    va_list args;
    va_start(args, fmt);
    std::vfprintf(f, fmt, args);
    std::fprintf(f, "\n");
    std::fflush(f);
    va_end(args);
}

inline std::vector<std::uint8_t>
bitsToBytesMSB(const std::vector<std::uint8_t>& bits)
{
    if (bits.empty()) return {};

    if (bits.size() % 8 != 0) {
        throw std::runtime_error("bitsToBytesMSB: number of bits not multiple of 8.");
    }

    const std::size_t nBytes = bits.size() / 8;
    std::vector<std::uint8_t> out(nBytes);

    for (std::size_t i = 0; i < nBytes; ++i) {
        std::uint8_t v = 0;
        for (int b = 0; b < 8; ++b) {
            v = static_cast<std::uint8_t>((v << 1) | (bits[i * 8 + b] & 1u));
        }
        out[i] = v;
    }

    return out;
}

// SCRAM_SEED_15 can be changed to handle larger datas
static constexpr std::uint16_t SCRAM_SEED_15 = 0x4001u; // 15-bit non-zero

// Return next PN bit (0/1) and update 15-bit LFSR state
inline std::uint8_t lfsr15_next_bit(std::uint16_t& state)
{
    std::uint8_t out = static_cast<std::uint8_t>((state >> 14) & 0x1u);

    std::uint8_t b14 = out;
    std::uint8_t b13 = static_cast<std::uint8_t>((state >> 13) & 0x1u);
    std::uint8_t fb  = static_cast<std::uint8_t>(b14 ^ b13);

    state = static_cast<std::uint16_t>(((state << 1) & 0x7FFFu) | fb);

    return out;
}

// Scramble the data
inline void scrambleDescrambleBytes(std::uint8_t* data, std::size_t len)
{
    if (!data || len == 0) return;

    std::uint16_t state = SCRAM_SEED_15;   // reset for each datagram

    for (std::size_t i = 0; i < len; ++i) {
        std::uint8_t mask = 0;

        // Build one byte of PN, MSB first:
        //   v = (v << 1) | bit;
        for (int b = 0; b < 8; ++b) {
            std::uint8_t bit = lfsr15_next_bit(state);
            mask = static_cast<std::uint8_t>((mask << 1) | bit);
        }

        data[i] ^= mask;
    }
}

inline void scrambleDescrambleBytes(std::vector<std::uint8_t>& bytes)
{
    if (bytes.empty()) return;
    scrambleDescrambleBytes(bytes.data(), bytes.size());
}
