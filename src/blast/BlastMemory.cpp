#include "blast/BlastMemory.h"

#include "NvBlastGlobals.h"
#include "NvErrors.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>

#if defined(_MSC_VER)
#include <malloc.h>
#endif

namespace blast {
namespace {

constexpr std::size_t kAlign = 16;

struct alignas(kAlign) Header {
  std::size_t size;
};

void* platformAlloc(std::size_t bytes) {
#if defined(_MSC_VER)
  return _aligned_malloc(bytes, kAlign);
#else
  void* p = nullptr;
  if (posix_memalign(&p, kAlign, bytes) != 0) {
    return nullptr;
  }
  return p;
#endif
}

void platformFree(void* p) {
#if defined(_MSC_VER)
  _aligned_free(p);
#else
  std::free(p);
#endif
}

}  // namespace

void* TrackingAllocator::allocate(size_t size, const char* /*typeName*/, const char* /*filename*/, int /*line*/) {
  const std::size_t bytes = size + sizeof(Header);
  void* raw = platformAlloc(bytes);
  if (raw == nullptr) {
    std::fprintf(stderr, "TrackingAllocator: out of memory requesting %zu bytes\n", bytes);
    std::abort();
  }
  auto* header = static_cast<Header*>(raw);
  header->size = size;
  liveBytes_ += size;
  ++liveAllocs_;
  ++totalAllocs_;
  return static_cast<char*>(raw) + sizeof(Header);
}

void TrackingAllocator::deallocate(void* ptr) {
  if (ptr == nullptr) {
    return;
  }
  auto* header = reinterpret_cast<Header*>(static_cast<char*>(ptr) - sizeof(Header));
  if (liveBytes_ < header->size || liveAllocs_ == 0) {
    std::fprintf(stderr, "TrackingAllocator: unmatched free\n");
    std::abort();
  }
  liveBytes_ -= header->size;
  --liveAllocs_;
  platformFree(header);
}

void LoggingErrorCallback::reportError(nvidia::NvErrorCode::Enum code, const char* message, const char* file,
                                       int line) {
  const char* level = "info";
  if (code == nvidia::NvErrorCode::eINVALID_OPERATION || code == nvidia::NvErrorCode::eINTERNAL_ERROR ||
      code == nvidia::NvErrorCode::eABORT || code == nvidia::NvErrorCode::eOUT_OF_MEMORY) {
    level = "error";
    ++errorCount_;
  } else if (code == nvidia::NvErrorCode::eDEBUG_WARNING) {
    level = "warning";
    ++warningCount_;
  }
  lastCode_ = code;
  std::snprintf(lastMessage_, sizeof(lastMessage_), "%s %s:%d: %s", level, file ? file : "?", line,
                message ? message : "");
  std::cerr << "NvBlast " << lastMessage_ << "\n";
}

bool BlastRuntime::init() {
  if (initialized_) {
    return true;
  }
  NvBlastGlobalSetAllocatorCallback(&alloc_);
  NvBlastGlobalSetErrorCallback(&errors_);
  baselineBytes_ = alloc_.liveBytes();
  initialized_ = true;
  return true;
}

void BlastRuntime::shutdown() {
  if (!initialized_) {
    return;
  }
  if (alloc_.liveAllocs() != 0) {
    std::cerr << "BlastRuntime shutdown with liveAllocs=" << alloc_.liveAllocs()
              << " liveBytes=" << alloc_.liveBytes() << " (instances should be destroyed first)\n";
  }
  NvBlastGlobalSetAllocatorCallback(nullptr);
  NvBlastGlobalSetErrorCallback(nullptr);
  initialized_ = false;
}

}  // namespace blast
