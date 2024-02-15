/// @file
#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>

namespace YAML {

template <typename Scalar, int N, int M, int Options>
struct convert<Eigen::Matrix<Scalar, N, M, Options>> {
  static constexpr int DYN = Eigen::Dynamic;
  using MatrixType = Eigen::Matrix<Scalar, N, M, Options>;
  using Index = Eigen::Index;

  static constexpr Index getRows(const MatrixType &m) { return N == DYN ? m.rows() : N; }

  static constexpr Index getCols(const MatrixType &m) { return M == DYN ? m.cols() : M; }

  static constexpr Index getSize(const MatrixType &m) { return getRows(m) * getCols(m); }

  static bool decode(const Node &node, MatrixType &rhs) {
    auto rhs_as_vec = node.as<std::vector<Scalar>>();
    auto n = (Index)rhs_as_vec.size();

    if (MatrixType::IsVectorAtCompileTime) {
      rhs.resize(N == 1 ? 1 : n, M == 1 ? 1 : n);
    }

    if (n != getSize(rhs)) {
      std::stringstream ss;
      ss << "parsed matrix has the wrong size.";
      ss << " Expected " << getSize(rhs) << ", got " << rhs_as_vec.size() << ".";
      throw YAML::ParserException(node.Mark(), ss.str());
    }
    rhs = Eigen::Map<MatrixType>(rhs_as_vec.data(), getRows(rhs), getCols(rhs));
    return true;
  }
};

}  // namespace YAML
