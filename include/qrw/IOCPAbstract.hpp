#pragma once

#include "qrw/Params.hpp"
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/multibody/data.hpp>
#include <exception>

namespace qrw {

using Motion = pinocchio::MotionTpl<Scalar>;

class IOCPAbstract {
 public:
  IOCPAbstract(Params const& params);
  virtual ~IOCPAbstract() = default;

  virtual void push_node(uint k, const ConstVecRefN& x0, Motion base_vel_ref) = 0;
  virtual void solve(std::size_t k) = 0;

  Params params_;
  uint num_iters_;
  uint max_iter;
  uint init_max_iters;
  std::vector<VectorN> xs_init;
  std::vector<VectorN> us_init;

  bool warm_start_empty() const;
  void cycle_warm_start();

  inline void _check_ws_dim() const {
    auto N = (std::size_t)params_.N_gait;
    if ((xs_init.size() != N + 1) && (us_init.size() != N)) {
      throw std::runtime_error("Warm-start size wrong.");
    }
  }
};

}  // namespace qrw
