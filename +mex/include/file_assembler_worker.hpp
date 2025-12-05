#pragma once

#include <cstdint>
#include <vector>
#include <map>
#include <unordered_map>
#include <memory>
#include <stdexcept>
#include <string>

#include "utils.hpp"
#include "payload_worker_common.hpp"

struct IByteWriter {
    virtual ~IByteWriter() {}
    virtual void write(const std::vector<std::uint8_t>& data) = 0;
    virtual void close() {}
};

// DebugByteWriter for logging the bytes we have written so far
struct DebugByteWriter : public IByteWriter {
    std::string name;
    explicit DebugByteWriter(const std::string& n) : name(n) {}

    void write(const std::vector<std::uint8_t>& data) override {
        goLog("[DebugByteWriter %s] wrote %zu bytes",
              name.c_str(),
              static_cast<std::size_t>(data.size()));
    }

    void close() override {
        goLog("[DebugByteWriter %s] close()", name.c_str());
    }
};

// FileByteWriter is a writter than write data to a file
struct FileByteWriter : public IByteWriter {
    std::string filename_;
    FILE*       file_;

    explicit FileByteWriter(const std::string& fname)
        : filename_(fname), file_(nullptr)
    {}

    ~FileByteWriter() override {
        close();
    }

    void write(const std::vector<std::uint8_t>& data) override {
        if (data.empty()) {
            return;
        }

        if (!file_) {
            file_ = std::fopen(filename_.c_str(), "wb");
            if (!file_) {
                goLog("[FileByteWriter] ERROR opening '%s' for writing", filename_.c_str());
                return;
            }
            goLog("[FileByteWriter] Opened '%s' for writing", filename_.c_str());
        }

        std::size_t written = std::fwrite(data.data(), 1, data.size(), file_);
        if (written != data.size()) {
            goLog("[FileByteWriter] ERROR writing to '%s' (written=%zu, expected=%zu)",
                  filename_.c_str(),
                  written,
                  static_cast<std::size_t>(data.size()));
        }
    }

    void close() override {
        if (file_) {
            std::fflush(file_);
            std::fclose(file_);
            goLog("[FileByteWriter] Closed '%s'", filename_.c_str());
            file_ = nullptr;
        }
    }
};

class FileAssembler {
public:
    std::uint8_t fileId;
    std::shared_ptr<IByteWriter> writer;

    FileAssembler(std::uint8_t id,
                  std::shared_ptr<IByteWriter> w)
        : fileId(id),
          writer(std::move(w)),
          nextOffset_(0),
          totalSize_(0),
          hasTotal_(false),
          cnt_(0)
    {
        logFileName_ = std::string("file")
                     + std::to_string(static_cast<int>(fileId))
                     + "_log.txt";
    }

    bool acceptChunk(std::uint32_t offset,
                     std::uint32_t totalSize,
                     bool isLast,
                     const std::vector<std::uint8_t>& data)
    {
        if (!writer) {
            return false;
        }

        std::uint32_t len = static_cast<std::uint32_t>(data.size());

        if (isLast) {
            totalSize_ = totalSize;
            hasTotal_  = true;
        }

        // Already written entirely?
        if (offset + len <= nextOffset_) {
            return false;
        }

        std::uint32_t startOff = offset;
        std::vector<std::uint8_t> d = data;

        // Get the part that is new and trip the overlap area
        if (startOff < nextOffset_) {
            std::uint32_t trim = nextOffset_ - startOff;
            if (trim >= d.size()) {
                return false;
            }
            d.erase(d.begin(), d.begin() + static_cast<std::ptrdiff_t>(trim));
            startOff = nextOffset_;
            len      = static_cast<std::uint32_t>(d.size());
        }

        if (len == 0) return false;

        auto it = chunks_.find(startOff);
        if (it == chunks_.end()) {
            chunks_.emplace(startOff, std::move(d));
            tmp_chunks_.push_back(startOff);
            cnt_++;
        }

        int wholeSize = 0;
        for (auto &kv : chunks_) {
            wholeSize += static_cast<int>(kv.second.size());
        }

        goLogWithoutLock(
                  logFileName_.c_str(),
                  "[FileAssembler] total buffered chunks bytes = %d the offset we got is %d",
                  wholeSize,
                  offset);

        return flush();
    }

    bool flush()
    {
        if (!writer) return false;

        for (;;) {
            auto it = chunks_.find(nextOffset_);
            if (it == chunks_.end()) break;

            std::uint32_t startOff = it->first;
            std::vector<std::uint8_t> data = std::move(it->second);
            chunks_.erase(it);

            goLogWithoutLock(
                  logFileName_.c_str(),
                  "[FileAssembler] FLUSH: fileId=%d offset=%u len=%zu",
                  static_cast<int>(fileId),
                  startOff,
                  static_cast<std::size_t>(data.size()));

            writer->write(data);
            nextOffset_ += static_cast<std::uint32_t>(data.size());
        }

        if (isComplete()) {
            writer->close();
            return true;
        }

        return false;
    }

    bool isComplete() const
    {
        return hasTotal_ && (nextOffset_ >= totalSize_);
    }

private:
    std::uint32_t nextOffset_;
    std::uint32_t totalSize_;
    bool          hasTotal_;
    std::map<std::uint32_t, std::vector<std::uint8_t>> chunks_;
    std::vector<std::uint32_t> tmp_chunks_;
    int cnt_;
    std::string logFileName_;
};

class FileAssemblerWorker : public IWorker {
public:
    FileAssemblerWorker() = default;

    void process(const Datagram& dg) override
    {
        if (isDone_) {
            return;
        }

        try {
            if (dg.payloadLen < 10 || dg.payload.size() < dg.payloadLen) {
                goLog("[FileAssemblerWorker] Payload too short for FileChunk header "
                      "(payloadLen=%u, actual=%zu)",
                      static_cast<unsigned>(dg.payloadLen),
                      static_cast<std::size_t>(dg.payload.size()));
                return;
            }

            // Parse the file protocol
            const std::vector<std::uint8_t>& p = dg.payload;
            std::uint8_t fileId = p[0];

            auto readU32 = [](const std::vector<std::uint8_t>& v, std::size_t off) -> std::uint32_t {
                return (std::uint32_t(v[off])   << 24) |
                       (std::uint32_t(v[off+1]) << 16) |
                       (std::uint32_t(v[off+2]) << 8)  |
                       (std::uint32_t(v[off+3]));
            };

            std::uint32_t offset    = readU32(p, 1);
            std::uint32_t totalSize = readU32(p, 5);
            bool isLast             = (p[9] != 0);

            std::vector<std::uint8_t> chunkData(
                p.begin() + 10,
                p.begin() + static_cast<std::size_t>(dg.payloadLen));

            // 6) Get or create assembler for this fileId
            auto it = assemblers_.find(fileId);
            if (it == assemblers_.end()) {
                std::string name = "file_" + std::to_string(static_cast<int>(fileId));
                auto writer = std::make_shared<FileByteWriter>(name);
                auto fa     = std::make_unique<FileAssembler>(fileId, writer);
                it = assemblers_.emplace(fileId, std::move(fa)).first;
            }

            FileAssembler* fa = it->second.get();
            bool IsDone = fa->acceptChunk(offset, totalSize, isLast, chunkData);
            if (IsDone) {
                this->IsDone.store(true);
                isDone_ = true;
            }

        } catch (const std::exception& e) {
            goLog("[FileAssemblerWorker] ERROR: %s", e.what());
        }
    }

private:
    // fileId -> FileAssembler
    std::unordered_map<std::uint8_t, std::unique_ptr<FileAssembler>> assemblers_;
    bool isDone_ = false;
};
