#ifndef TYPES_H_INCLUDED
#define TYPES_H_INCLUDED

#include <Eigen/Core>
#include <Eigen/Dense>

using Scalar = double;

using Eigen::Index;

using Vector1 = Eigen::Matrix<Scalar, 1, 1>;
using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
using Vector3i = Eigen::Matrix<int, 3, 1>;
using Vector4 = Eigen::Matrix<Scalar, 4, 1>;
using Eigen::Vector4i;
using Vector5 = Eigen::Matrix<Scalar, 5, 1>;
using Vector6 = Eigen::Matrix<Scalar, 6, 1>;
using Vector7 = Eigen::Matrix<Scalar, 7, 1>;
using Vector8 = Eigen::Matrix<Scalar, 8, 1>;
using Vector11 = Eigen::Matrix<Scalar, 11, 1>;
using Vector12 = Eigen::Matrix<Scalar, 12, 1>;
using Vector18 = Eigen::Matrix<Scalar, 18, 1>;
using Vector19 = Eigen::Matrix<Scalar, 19, 1>;
using Vector24 = Eigen::Matrix<Scalar, 24, 1>;
using VectorN = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
using ConstVecRefN = Eigen::Ref<const VectorN>;
using VectorNi = Eigen::Matrix<int, Eigen::Dynamic, 1>;

using RowVector4 = Eigen::Matrix<Scalar, 1, 4>;
using RowMatrix6N = Eigen::Matrix<Scalar, 6, Eigen::Dynamic, Eigen::RowMajor>;

using Matrix2 = Eigen::Matrix<Scalar, 2, 2>;
using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;
using Matrix6 = Eigen::Matrix<Scalar, 6, 6>;
using Matrix12 = Eigen::Matrix<Scalar, 12, 12>;

using Matrix34 = Eigen::Matrix<Scalar, 3, 4>;
using Matrix43 = Eigen::Matrix<Scalar, 4, 3>;
using Matrix64 = Eigen::Matrix<Scalar, 6, 4>;
using Matrix74 = Eigen::Matrix<Scalar, 7, 4>;
using MatrixN4 = Eigen::Matrix<Scalar, Eigen::Dynamic, 4>;
using MatrixN4i = Eigen::Matrix<int, Eigen::Dynamic, 4>;
using Matrix3N = Eigen::Matrix<Scalar, 3, Eigen::Dynamic>;
using Matrix6N = Eigen::Matrix<Scalar, 6, Eigen::Dynamic>;
using Matrix12N = Eigen::Matrix<Scalar, 12, Eigen::Dynamic>;
using MatrixN = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
using ConstMatRefN = Eigen::Ref<const MatrixN>;
using MatrixNi = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>;

#endif  // TYPES_H_INCLUDED
