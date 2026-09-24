#pragma once

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace physics {

// Minimal persistent worker pool for data-parallel physics loops. The caller
// thread participates; work is handed out in small chunks through an atomic
// counter so uneven pair costs balance. One job at a time (physics is
// single-threaded outside these loops).
class WorkerPool {
public:
  static WorkerPool& instance() {
    static WorkerPool pool;
    return pool;
  }

  size_t workerCount() const { return workers_.size(); }

  void run(size_t count, size_t chunk, const std::function<void(size_t)>& fn) {
    if (count == 0) return;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      job_ = &fn;
      count_ = count;
      chunk_ = std::max<size_t>(1, chunk);
      next_.store(0, std::memory_order_relaxed);
      active_ = workers_.size();
      ++generation_;
    }
    wake_.notify_all();
    drain();
    std::unique_lock<std::mutex> lock(mutex_);
    done_.wait(lock, [&] { return active_ == 0; });
    job_ = nullptr;
  }

  WorkerPool(const WorkerPool&) = delete;
  WorkerPool& operator=(const WorkerPool&) = delete;

private:
  WorkerPool() {
    const unsigned hw = std::thread::hardware_concurrency();
    // Leave a core for the render/main thread's other work.
    const unsigned n = hw > 2 ? std::min(hw - 1, 15u) : 0u;
    for (unsigned i = 0; i < n; ++i) workers_.emplace_back([this] { loop(); });
  }
  ~WorkerPool() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      quit_ = true;
      ++generation_;
    }
    wake_.notify_all();
    for (std::thread& t : workers_) t.join();
  }

  void drain() {
    const std::function<void(size_t)>& fn = *job_;
    for (;;) {
      const size_t begin = next_.fetch_add(chunk_, std::memory_order_relaxed);
      if (begin >= count_) return;
      const size_t end = std::min(count_, begin + chunk_);
      for (size_t i = begin; i < end; ++i) fn(i);
    }
  }

  void loop() {
    uint64_t seen = 0;
    for (;;) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait(lock, [&] { return generation_ != seen; });
        seen = generation_;
        if (quit_) return;
      }
      drain();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (--active_ == 0) done_.notify_one();
      }
    }
  }

  std::vector<std::thread> workers_;
  std::mutex mutex_;
  std::condition_variable wake_, done_;
  const std::function<void(size_t)>* job_ = nullptr;
  size_t count_ = 0, chunk_ = 1, active_ = 0;
  std::atomic<size_t> next_{0};
  uint64_t generation_ = 0;
  bool quit_ = false;
};

// Runs fn(i) for i in [0, count). Falls back to a plain loop for small counts,
// where waking workers costs more than it saves.
template<class Fn>
void parallelFor(size_t count, size_t minParallel, Fn&& fn) {
  WorkerPool& pool = WorkerPool::instance();
  if (count < minParallel || pool.workerCount() == 0) {
    for (size_t i = 0; i < count; ++i) fn(i);
    return;
  }
  const std::function<void(size_t)> job = [&fn](size_t i) { fn(i); };
  const size_t threads = pool.workerCount() + 1;
  pool.run(count, std::max<size_t>(1, count / (threads * 8)), job);
}

}  // namespace physics
