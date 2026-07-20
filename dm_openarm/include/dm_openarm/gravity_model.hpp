#pragma once

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace dm_openarm {

/// Per-joint gravity parameters: tau_i = amp * sin(q_i + phase) + bias
struct JointGravityParam {
  double amp{0.0};
  double phase{0.0};
  double bias{0.0};
};

/// Decoupled single-joint gravity model (identification-friendly first step).
/// Full coupling matrix can replace this later without changing callers.
class GravityModel {
public:
  GravityModel() = default;
  explicit GravityModel(std::vector<JointGravityParam> joints);

  std::size_t size() const noexcept { return joints_.size(); }
  const std::vector<JointGravityParam>& joints() const noexcept { return joints_; }

  /// Compute gravity torques [N·m] for joint positions q [rad], same order as config motors.
  std::vector<double> compute(const std::vector<double>& q) const;

  /// scale * compute(q)
  std::vector<double> compute(const std::vector<double>& q, double scale) const;

private:
  std::vector<JointGravityParam> joints_;
};

}  // namespace dm_openarm
