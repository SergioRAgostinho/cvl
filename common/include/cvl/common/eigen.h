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

  template<typename _Scalar, int _Rows, int _Cols>
  using Matrix = Eigen::Matrix<_Scalar, _Rows, _Cols, Eigen::RowMajor>;

  template<typename _Scalar>
  using Matrix3 = Matrix<_Scalar, 3, 3>;

  template<typename _Scalar>
  using Matrix4 = Matrix<_Scalar, 4, 4>;

  typedef Matrix3<double> Matrix3d;

  template<typename _Scalar>
  using Matrix43 = Matrix<_Scalar, 4, 3>;

  typedef Matrix43<double> Matrix43d;

  // Vector
  template<typename _Scalar, short _Dim>
  using Vector = Eigen::Matrix<_Scalar, _Dim, 1>;

  template<typename _Scalar>
  using Vector3 = Vector<_Scalar, 3>;

  template<typename _Scalar>
  using Vector4 = Vector<_Scalar, 4>;

  using Eigen::Vector3f;
  using Eigen::Vector3d;
  using Eigen::Vector4f;
  using Eigen::Vector4d;

  // Quaternion
  using Eigen::QuaternionBase;
  using Eigen::Quaternion;
  using Eigen::Quaternionf;
  using Eigen::Quaterniond;

  // AngleAxis
  using Eigen::AngleAxis;
}

#endif //CVL_COMMON_EIGEN_H_
