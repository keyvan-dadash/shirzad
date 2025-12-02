#pragma once

#include <cstdint>
#include <vector>
#include <atomic>

#include "datagram_parser.hpp"

// Job is the unit of work in the system
struct Job {
    bool stop;
    std::vector<std::uint8_t> bytes;

    Job() : stop(false), bytes() {}
};

// Abstract worker interface.
// Implemenet the processing job functionality.
struct IWorker {
    virtual ~IWorker() {}
    virtual void process(const Datagram& dg) = 0;
};
