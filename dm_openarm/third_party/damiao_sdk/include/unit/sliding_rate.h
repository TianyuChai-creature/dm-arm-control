#pragma once

#include <algorithm>
#include <chrono>
#include <deque>
#include <iterator>

namespace damiao::detail {

class SlidingRate
{
public:
  using clock = std::chrono::steady_clock;

  void reset() { samples_.clear(); }

  void record(clock::time_point now)
  {
    samples_.push_back(now);
    const auto cutoff = now - std::chrono::seconds(1);
    while(!samples_.empty() && samples_.front() < cutoff)
    {
      samples_.pop_front();
    }
  }

  double hz(clock::time_point now) const
  {
    const auto first = std::lower_bound(
      samples_.begin(), samples_.end(), now - std::chrono::seconds(1));
    const auto count = std::distance(first, samples_.end());
    if(count < 2)
    {
      return 0.0;
    }
    const double span = std::chrono::duration<double>(samples_.back() - *first).count();
    return span > 0.0 ? static_cast<double>(count - 1) / span : 0.0;
  }

private:
  std::deque<clock::time_point> samples_;
};

}  // namespace damiao::detail
