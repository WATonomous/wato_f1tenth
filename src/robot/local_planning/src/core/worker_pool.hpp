#ifndef LOCAL_PLANNING_CORE_WORKER_POOL_HPP
#define LOCAL_PLANNING_CORE_WORKER_POOL_HPP

#include "BS_thread_pool.hpp"

#include <algorithm>
#include <cstddef>
#include <thread>
#include <utility>

namespace local_planning
{

// Persistent workers for the OVERTAKE product. Created once, live for the
// process. Cap at 6: that is every A78AE on the Orin Nano, and it leaves the
// ROS executor some room on a laptop that has more cores.
inline std::size_t workerCount()
{
  const unsigned hardware = std::thread::hardware_concurrency();
  if (hardware == 0) {
    return 6;
  }
  return std::min<std::size_t>(hardware, 6);
}

inline BS::light_thread_pool & workerPool()
{
  static BS::light_thread_pool pool(workerCount());
  return pool;
}

// Run fn(i) for i in [0, n). The caller waits. n == 0/1 stays on this thread
// so PASS does not wake the pool for one candidate. Do not nest: wait() waits
// for every task in the pool, and a worker calling this deadlocks.
template<typename Fn>
void parallelFor(std::size_t n, Fn && fn)
{
  if (n == 0) {
    return;
  }
  if (n == 1 || workerCount() <= 1) {
    for (std::size_t i = 0; i < n; ++i) {
      fn(i);
    }
    return;
  }
  workerPool().detach_loop(std::size_t{0}, n, std::forward<Fn>(fn));
  workerPool().wait();
}

}  // namespace local_planning

#endif  // LOCAL_PLANNING_CORE_WORKER_POOL_HPP
