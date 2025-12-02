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
    static FILE* file = nullptr;
    if (file) {
        return file;
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

    file = std::fopen(pathbuf, "a");
    if (!file) {
        file = std::fopen(filename, "a");
    }

    return file;
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
