#include "qrw/IOCPAbstract.hpp"

namespace qrw {

IOCPAbstract::IOCPAbstract(Params const &params) : params_(params), num_iters_(), xs_init(), us_init() {
  max_iter = params_.save_guess ? 1000U : params_.ocp.max_iter;
  init_max_iters = params_.ocp.init_max_iters;
}

bool IOCPAbstract::warm_start_empty() const { return xs_init.empty() || us_init.empty(); }

void IOCPAbstract::cycle_warm_start() {
  xs_init[0] = xs_init.back();
  us_init[0] = us_init.back();
  std::rotate(xs_init.begin(), xs_init.begin() + 1, xs_init.end());
  std::rotate(us_init.begin(), us_init.begin() + 1, us_init.end());
}

}  // namespace qrw
