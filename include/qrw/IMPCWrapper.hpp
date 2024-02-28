#pragma once

#include "qrw/Params.hpp"
#include "qrw/MPCResult.hpp"
#include "qrw/IOCPAbstract.hpp"

namespace qrw {

struct IMPCWrapper {
  using StdVecVecN = std::vector<VectorN>;

  IMPCWrapper(Params const &params) : params_(params) {}
  virtual ~IMPCWrapper() = default;

  Params params_;

  int N_gait() const { return params_.N_gait; }
  uint window_size() const { return params_.window_size; }
  virtual void solve(uint k, const ConstVecRefN &x0, Motion base_vel_ref) = 0;
  virtual MPCResult get_latest_result() const = 0;
};

}  // namespace qrw
