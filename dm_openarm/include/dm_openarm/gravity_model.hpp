#pragma once

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace dm_openarm {

/// Coupled gravity: tau = W * phi(q). basis names match Python gravity_fit.
struct CoupledGravityParam {
  std::vector<std::string> basis;
  std::vector<std::vector<double>> weights;  // [joint][k]
};

/// Multi-joint coupled gravity model only.
class GravityModel {
public:
  GravityModel() = default;
  explicit GravityModel(CoupledGravityParam coupled);

  std::size_t size() const noexcept { return coupled_.weights.size(); }
  const CoupledGravityParam& coupled() const noexcept { return coupled_; }

  /// Compute gravity torques [N·m] for joint positions q [rad], motor order.
  std::vector<double> compute(const std::vector<double>& q) const;

  /// scale * compute(q)
  std::vector<double> compute(const std::vector<double>& q, double scale) const;

  /// Evaluate one basis name at q (q size >= 5 padded with 0).
  static double eval_basis(const std::string& name, const std::vector<double>& q);

private:
  CoupledGravityParam coupled_;
};

}  // namespace dm_openarm
