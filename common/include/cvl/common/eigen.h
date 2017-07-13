/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/05/05
  * \date last modified: 2017/05/05
  */
#pragma once
#ifndef CVL_COMMON_EIGEN_H_
#define CVL_COMMON_EIGEN_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ht
{
  using Eigen::Dynamic;

  // Matrix
  using Eigen::MatrixBase;
  using Eigen::Matrix;

  template<typename _Scalar>
  using Matrix3 = Matrix<_Scalar, 3, 3>;

  typedef Matrix3<double> Matrix3d;

  // Vector
  template<typename _Scalar, short _Dim>
  using Vector = Eigen::Matrix<_Scalar, _Dim, 1>;

  template<typename _Scalar>
  using Vector3 = Vector<_Scalar, 3>;

  using Eigen::Vector3f;
  using Eigen::Vector3d;

  // Quaternion
  using Eigen::QuaternionBase;
  using Eigen::Quaternion;
  using Eigen::Quaternionf;
  using Eigen::Quaterniond;

  // AngleAxis
  using Eigen::AngleAxis;
}

#endif //CVL_COMMON_EIGEN_H_
