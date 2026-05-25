#ifndef TIMER_HPP
#define TIMER_HPP

#include <chrono>

class Timer {
private:
  using Clock = std::chrono::steady_clock;

  bool is_counting{false};
  std::chrono::time_point<Clock> last{Clock::now()};
  Clock::duration elapsed_time{Clock::duration::zero()};

public:
  Timer() = default;

  void reset() { *this = Timer(); }

  void start() {
    if (!is_counting) {
      last = Clock::now();
      is_counting = true;
    }
  }

  void pause() {
    if (is_counting) {
      elapsed_time += Clock::now() - last;
      is_counting = false;
    }
  }

  double get_time() const {
    auto total = elapsed_time;
    if (is_counting) {
      total += Clock::now() - last;
    }
    return std::chrono::duration<double>(total).count();
  }
};

#endif
