# Vendored from bshoshany/thread-pool

`BS_thread_pool.hpp` is Barak Shoshany's **BS::thread_pool** v5.1.0
(<https://github.com/bshoshany/thread-pool>, MIT — see `LICENSE.txt`), the
single header the planner uses as a persistent `parallel_for`.

`AMENT_IGNORE` here keeps the linters off foreign code.

## Why a vendor rather than a system package

The planner needs workers that already exist when an OVERTAKE tick starts, then
a barrier: run `fn(i)` for `i in 0..n`. TBB and Taskflow also provide that, but
both are extra packages in the image. This header is C++17, has no `.so`, and
is the usual one-file pool.

Do not edit the header. Bump the tag in this file if it is replaced.
