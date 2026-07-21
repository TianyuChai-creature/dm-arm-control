#pragma once

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace dm_openarm {

/// Per-joint gravity parameters: tau_i = amp * sin(q_i + phase) + bias
struct JointGravityParam {
  double amp{0.0};
  double phase{0.0};
  double bias{0.0};
};

enum class GravityMode {
  Decoupled,
  Coupled,
};

/// Coupled: tau = W * phi(q). basis names must match Python gravity_fit.
struct CoupledGravityParam {
  std::vector<std::string> basis;
  std::vector<std::vector<double>> weights;  // [joint][k]
};

/// Gravity model: decoupled sin or coupled linear basis.
class GravityModel {
public:
  GravityModel() = default;
  explicit GravityModel(std::vector<JointGravityParam> joints);
  GravityModel(
    GravityMode mode,
    std::vector<JointGravityParam> joints,
    CoupledGravityParam coupled);

  GravityMode mode() const noexcept { return mode_; }
  std::size_t size() const noexcept;
  const std::vector<JointGravityParam>& joints() const noexcept { return joints_; }
  const CoupledGravityParam& coupled() const noexcept { return coupled_; }

  /// Compute gravity torques [N·m] for joint positions q [rad], motor order.
  std::vector<double> compute(const std::vector<double>& q) const;

  /// scale * compute(q)
  std::vector<double> compute(const std::vector<double>& q, double scale) const;

  /// Evaluate one basis name at q (q size >= 5 padded with 0).
  static double eval_basis(const std::string& name, const std::vector<double>& q);

private:
  std::vector<double> compute_decoupled(const std::vector<double>& q) const;
  std::vector<double> compute_coupled(const std::vector<double>& q) const;

  GravityMode mode_{GravityMode::Decoupled};
  std::vector<JointGravityParam> joints_;
  CoupledGravityParam coupled_;
};

}  // namespace dm_openarm
