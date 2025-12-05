#include "mex.h"
#include <cstdint>
#include <vector>
#include <thread>
#include <atomic>
#include <string>
#include <stdexcept>
#include <memory>
#include <unordered_map>
#include <mutex>
#include <chrono>


#include "include/queue/blockingconcurrentqueue.h"
#include "include/datagram_console_worker.hpp"
#include "include/file_assembler_worker.hpp"
#include "include/payload_worker_common.hpp"
#include "include/datagram_parser.hpp"
#include "include/decoders/viterbit_k3_decoder.h"
#include "include/utils.hpp"

#define likely(x)      __builtin_expect(!!(x), 1)
#define unlikely(x)    __builtin_expect(!!(x), 0)

static std::atomic<bool> g_running(false);
static moodycamel::BlockingConcurrentQueue<Job> g_queue;
static std::vector<std::thread> g_threads;

struct WorkerEntry {
    std::unique_ptr<IWorker> worker;
    std::mutex mtx;
};

static std::unordered_map<std::uint8_t, WorkerEntry> g_workers;
static std::mutex g_workers_mutex;
static std::atomic<bool> g_allWorkersDone{false};
static std::unique_ptr<moodycamel::ProducerToken> g_prodTok;


static void workerLoop(int workerID)
{
    std::string logFile = std::string("worker") + std::to_string(workerID) + std::string("_log.txt");

    while (true) {
        Job job;
        g_queue.wait_dequeue(job);

        if (job.stop) {
            break;
        }

        // Decode datagram
        std::vector<std::uint8_t> codedBits(job.bytes.size());
        for (std::size_t i = 0; i < job.bytes.size(); ++i) {
            codedBits[i] = (job.bytes[i] != 0) ? 1 : 0;
        }

        std::vector<std::uint8_t> infoBits  = viterbi_k3_decode(codedBits);
        std::vector<std::uint8_t> dataBytes = bitsToBytesMSB(infoBits);

        // Descramble the data (by scrambling again)
        scrambleDescrambleBytes(dataBytes);

        Datagram dg;
        if (!Datagram::fromBytes(dataBytes, dg)) {
            goLogWithoutLock(logFile.c_str(), "[WorkerLoop] Bad checksum, StreamId=%u",
                  static_cast<unsigned>(dg.streamId));
            continue;
        }

        WorkerEntry* entry = nullptr;
        {
            std::lock_guard<std::mutex> lock(g_workers_mutex);
            auto it = g_workers.find(dg.streamId);
            if (it != g_workers.end() && it->second.worker) {
                entry = &it->second;
            }
        }

        if (!entry) {
            goLogWithoutLock(logFile.c_str(), "[WorkerLoop] No worker found for StreamId=%u",
                  static_cast<unsigned>(dg.streamId));
            continue;
        }

        IWorker* worker = entry->worker.get();

        if (worker->IsDone.load(std::memory_order_acquire)) {

            bool allDone = true;
            {
                std::lock_guard<std::mutex> lock(g_workers_mutex);
                for (auto &kv : g_workers) {
                    IWorker* w = kv.second.worker.get();
                    if (!w) continue;
                    if (!w->IsDone.load(std::memory_order_acquire)) {
                        allDone = false;
                        break;
                    }
                }
            }

            if (allDone) {
                g_allWorkersDone.store(true, std::memory_order_release);
            }

            continue;
        }

        {
            std::lock_guard<std::mutex> wlock(entry->mtx);
            worker->process(dg);
        }
    }
}

static std::string getCommand(int nrhs, const mxArray* prhs[])
{
    if (nrhs < 1 || !mxIsChar(prhs[0])) {
        mexErrMsgIdAndTxt("payload_worker_mex:InvalidInput",
                          "First argument must be a command string.");
    }
    char buf[64];
    mxGetString(prhs[0], buf, sizeof(buf));
    return std::string(buf);
}

static void cmdInit(int nrhs, const mxArray* prhs[])
{
    if (g_running.load()) {
        mexPrintf("payload_worker_mex: already initialized.\n");
        return;
    }

    int numThreads = 1;
    if (nrhs >= 2) {
        numThreads = static_cast<int>(mxGetScalar(prhs[1]));
        if (numThreads <= 0) numThreads = 1;
    }

    g_running.store(true);
    g_prodTok.reset(new moodycamel::ProducerToken(g_queue));

    try {
        g_threads.clear();
        g_threads.reserve(numThreads);

        for (int i = 0; i < numThreads; ++i) {
            g_threads.emplace_back(workerLoop, i);
        }

        mexPrintf("payload_worker_mex: started %d worker threads.\n", numThreads);
    } catch (const std::exception& e) {
        g_running.store(false);
        g_workers.clear();
        mexErrMsgIdAndTxt("payload_worker_mex:InitError",
                          "Failed to start worker threads: %s", e.what());
    }

    removeFile("log.txt");
}

// enqueue job
static void cmdEnqueue(int nrhs, const mxArray* prhs[])
{
    if (g_allWorkersDone.load()) {
        std::exit(-1);
    }

    if (!g_running.load()) {
        mexErrMsgIdAndTxt("payload_worker_mex:NotInitialized",
                          "Call payload_worker_mex('init', ...) first.");
    }

    if (nrhs < 2) {
        mexErrMsgIdAndTxt("payload_worker_mex:InvalidInput",
                          "enqueue requires a uint8 vector as second argument.");
    }

    const mxArray* arr = prhs[1];

    if (!mxIsUint8(arr)) {
        mexErrMsgIdAndTxt("payload_worker_mex:InvalidType",
                          "enqueue expects a uint8 vector.");
    }

    const std::uint8_t* data = static_cast<const std::uint8_t*>(mxGetData(arr));
    mwSize N = mxGetNumberOfElements(arr);

    Job job;
    job.stop = false;
    job.bytes.assign(data, data + N);  // we have to copy since matlab memory is different from c++

    if (likely(g_prodTok)) {
        g_queue.enqueue(*g_prodTok, std::move(job));
    } else {
        g_queue.enqueue(std::move(job));   // fallback, shouldn't normally happen
    }
}

static void cmdAddWorker(int nrhs, const mxArray* prhs[])
{
    if (nrhs < 3) {
        mexErrMsgIdAndTxt("payload_worker_mex:InvalidInput",
                          "Usage: payload_worker_mex('add_worker', streamId, type)");
    }

    int streamId = static_cast<int>(mxGetScalar(prhs[1]));
    if (streamId < 0 || streamId > 255) {
        mexErrMsgIdAndTxt("payload_worker_mex:InvalidStreamId",
                          "streamId must be in range [0,255].");
    }

    char typeBuf[32];
    mxGetString(prhs[2], typeBuf, sizeof(typeBuf));
    std::string type(typeBuf);

    std::unique_ptr<IWorker> worker;
    if (type == "console") {
        worker.reset(new ConsoleWorker());
    } else if (type == "file") {
        worker.reset(new FileAssemblerWorker());
    } else {
        mexErrMsgIdAndTxt("payload_worker_mex:InvalidType",
                          "Unknown worker type: %s. Use 'console' or 'file'.", type.c_str());
    }

    std::lock_guard<std::mutex> lock(g_workers_mutex);
    if (g_workers.find(streamId) != g_workers.end()) {
        mexPrintf("payload_worker_mex: Replacing existing worker for stream %d\n", streamId);
    }

    WorkerEntry &entry = g_workers[static_cast<std::uint8_t>(streamId)];
    entry.worker = std::move(worker);

    mexPrintf("payload_worker_mex: Added %s worker for stream %d\n", type.c_str(), streamId);
}

// Remove worker for a given stream id
static void cmdRemoveWorker(int nrhs, const mxArray* prhs[])
{
    if (nrhs < 2) {
        mexErrMsgIdAndTxt("payload_worker_mex:InvalidInput",
                          "Usage: payload_worker_mex('remove_worker', streamId)");
    }

    int streamId = static_cast<int>(mxGetScalar(prhs[1]));

    std::lock_guard<std::mutex> lock(g_workers_mutex);
    auto it = g_workers.find(static_cast<std::uint8_t>(streamId));
    if (it != g_workers.end()) {
        g_workers.erase(it);
        mexPrintf("payload_worker_mex: Removed worker for stream %d\n", streamId);
    } else {
        mexPrintf("payload_worker_mex: No worker found for stream %d\n", streamId);
    }
}

static void cmdShutdown()
{
    if (!g_running.load()) {
        return;
    }

    for (std::size_t i = 0; i < g_threads.size(); ++i) {
        Job j;
        j.stop = true;
        g_queue.enqueue(std::move(j));
    }

    for (auto& th : g_threads) {
        if (th.joinable()) {
            th.join();
        }
    }
    g_threads.clear();

    g_workers.clear();
    g_running.store(false);

    mexPrintf("payload_worker_mex: shutdown complete.\n");
}

static void atExitHandler()
{
    cmdShutdown();
}

// entry point
void mexFunction(int nlhs, mxArray* plhs[],
                 int nrhs, const mxArray* prhs[])
{
    static bool registeredAtExit = false;
    if (!registeredAtExit) {
        mexAtExit(atExitHandler);
        registeredAtExit = true;
    }

    if (nrhs < 1) {
        mexErrMsgIdAndTxt("payload_worker_mex:NoCommand",
                          "First argument must be a command string.");
    }

    std::string cmd = getCommand(nrhs, prhs);

    if (cmd == "init") {
        cmdInit(nrhs, prhs);
    } else if (cmd == "enqueue") {
        cmdEnqueue(nrhs, prhs);
    } else if (cmd == "add_worker") {
        cmdAddWorker(nrhs, prhs);
    } else if (cmd == "remove_worker") {
        cmdRemoveWorker(nrhs, prhs);
    } else if (cmd == "shutdown") {
        cmdShutdown();
    } else {
        mexErrMsgIdAndTxt("payload_worker_mex:UnknownCommand",
                        "Unknown command '%s'. Use 'init', 'enqueue', 'add_worker', 'remove_worker', or 'shutdown'.",
                        cmd.c_str());
    }
}
