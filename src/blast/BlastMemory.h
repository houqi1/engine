#pragma once

#include "NvAllocatorCallback.h"
#include "NvErrorCallback.h"

#include <cstddef>
#include <cstdint>

namespace blast {

// 16-byte aligned allocator used for SDK objects and NvBlastGlobals.
// Tracks live bytes for P0 reset/leak checks. Not a production heap.
class TrackingAllocator final : public nvidia::NvAllocatorCallback {
public:
  void* allocate(size_t size, const char* typeName, const char* filename, int line) override;
  void deallocate(void* ptr) override;

  std::size_t liveBytes() const { return liveBytes_; }
  std::uint64_t liveAllocs() const { return liveAllocs_; }
  std::uint64_t totalAllocs() const { return totalAllocs_; }

private:
  std::size_t liveBytes_ = 0;
  std::uint64_t liveAllocs_ = 0;
  std::uint64_t totalAllocs_ = 0;
};

class LoggingErrorCallback final : public nvidia::NvErrorCallback {
public:
  void reportError(nvidia::NvErrorCode::Enum code, const char* message, const char* file, int line) override;

  int errorCount() const { return errorCount_; }
  int warningCount() const { return warningCount_; }
  const char* lastMessage() const { return lastMessage_; }
  nvidia::NvErrorCode::Enum lastCode() const { return lastCode_; }

private:
  int errorCount_ = 0;
  int warningCount_ = 0;
  nvidia::NvErrorCode::Enum lastCode_ = nvidia::NvErrorCode::eNO_ERROR;
  char lastMessage_[256]{};
};

// Process-wide Blast allocator and error callbacks. Survives scene reset.
// Create before any NvBlast object; shutdown after every StructureWorld is gone.
class BlastRuntime {
public:
  BlastRuntime() = default;
  ~BlastRuntime() { shutdown(); }
  BlastRuntime(const BlastRuntime&) = delete;
  BlastRuntime& operator=(const BlastRuntime&) = delete;
  BlastRuntime(BlastRuntime&&) = delete;
  BlastRuntime& operator=(BlastRuntime&&) = delete;

  bool init();
  void shutdown();
  bool initialized() const { return initialized_; }

  TrackingAllocator& allocator() { return alloc_; }
  const TrackingAllocator& allocator() const { return alloc_; }
  LoggingErrorCallback& errors() { return errors_; }
  const LoggingErrorCallback& errors() const { return errors_; }

  std::size_t liveBytes() const { return alloc_.liveBytes(); }
  std::size_t runtimeBaselineBytes() const { return baselineBytes_; }
  int errorCount() const { return errors_.errorCount(); }
  int warningCount() const { return errors_.warningCount(); }

private:
  TrackingAllocator alloc_;
  LoggingErrorCallback errors_;
  bool initialized_ = false;
  std::size_t baselineBytes_ = 0;
};

struct AlignedBlock {
  TrackingAllocator* alloc = nullptr;
  void* ptr = nullptr;

  AlignedBlock() = default;
  AlignedBlock(const AlignedBlock&) = delete;
  AlignedBlock& operator=(const AlignedBlock&) = delete;
  AlignedBlock(AlignedBlock&& o) noexcept : alloc(o.alloc), ptr(o.ptr) { o.ptr = nullptr; }
  AlignedBlock& operator=(AlignedBlock&& o) noexcept {
    if (this != &o) {
      reset();
      alloc = o.alloc;
      ptr = o.ptr;
      o.ptr = nullptr;
    }
    return *this;
  }
  ~AlignedBlock() { reset(); }

  void reset() {
    if (alloc && ptr) {
      alloc->deallocate(ptr);
      ptr = nullptr;
    }
  }
};

}  // namespace blast
