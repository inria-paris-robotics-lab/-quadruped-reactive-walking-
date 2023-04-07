#pragma once

#include <vector>
#include "qrw/Types.h"

static constexpr uint NUM_GAIT_COLS = 4;

struct MPCResult {
  MatrixNi gait;
  std::vector<VectorN> xs;
  std::vector<VectorN> us;
  std::vector<MatrixN> Ks;
  double solving_duration = 0.0;
  uint num_iters = 0;
  bool new_result = false;

  MPCResult(uint Ngait, uint nx, uint nu, uint ndx, uint window_size)
      : gait(Ngait + 1, NUM_GAIT_COLS),
        xs(Ngait + 1, VectorN::Zero(nx)),
        us(Ngait, VectorN::Zero(nu)),
        Ks(window_size, MatrixN::Zero(nu, ndx)) {
    gait.setZero();
  }

  MPCResult(uint Ngait, uint nx, uint nu, uint ndx)
      : MPCResult(Ngait, nx, nu, ndx, Ngait) {}
};
