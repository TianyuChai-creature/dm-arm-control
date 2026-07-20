#include "dm_openarm/gravity_model.hpp"

#include <utility>

namespace dm_openarm {

GravityModel::GravityModel(std::vector<JointGravityParam> joints)
  : joints_(std::move(joints))
{
}

std::vector<double> GravityModel::compute(const std::vector<double>& q) const
{
  if(q.size() != joints_.size())
  {
    throw std::invalid_argument("gravity model: q size must match joint count");
  }

  std::vector<double> tau(joints_.size(), 0.0);
  for(std::size_t i = 0; i < joints_.size(); ++i)
  {
    const auto& p = joints_[i];
    tau[i] = p.amp * std::sin(q[i] + p.phase) + p.bias;
  }
  return tau;
}

std::vector<double> GravityModel::compute(const std::vector<double>& q, double scale) const
{
  auto tau = compute(q);
  for(double& t : tau)
  {
    t *= scale;
  }
  return tau;
}

}  // namespace dm_openarm
