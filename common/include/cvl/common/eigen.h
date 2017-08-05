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
  using Eigen::RowMajor;
  using Eigen::ColMajor;

  // Matrix
  using Eigen::MatrixBase;

  template<typename _Scalar, int _Rows, int _Cols, int _Options = RowMajor>
  using Matrix = Eigen::Matrix<_Scalar, _Rows, _Cols, _Options>;

  template<typename _Scalar>
  using Matrix3 = Matrix<_Scalar, 3, 3>;

  template<typename _Scalar>
  using Matrix4 = Matrix<_Scalar, 4, 4>;

  typedef Matrix3<float> Matrix3f;
  typedef Matrix3<double> Matrix3d;
  typedef Matrix4<float> Matrix4f;
  typedef Matrix4<double> Matrix4d;

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

  typedef Vector3<float> Vector3f;
  typedef Vector4<float> Vector4f;
  typedef Vector3<double> Vector3d;
  typedef Vector4<double> Vector4d;

  // RowVector
  template<typename _Scalar, short _Dim>
  using RowVector = Eigen::Matrix<_Scalar, 1, _Dim>;

  template<typename _Scalar>
  using RowVector3 = RowVector<_Scalar, 3>;

  template<typename _Scalar>
  using RowVector4 = RowVector<_Scalar, 4>;

  typedef RowVector3<float> RowVector3f;
  typedef RowVector4<float> RowVector4f;
  typedef RowVector3<double> RowVector3d;
  typedef RowVector4<double> RowVector4d;


  // Array
  using Eigen::Array;

  // Quaternion
  using Eigen::QuaternionBase;
  using Eigen::Quaternion;
  using Eigen::Quaternionf;
  using Eigen::Quaterniond;

  // AngleAxis
  using Eigen::AngleAxis;
  using Eigen::AngleAxisf;
  using Eigen::AngleAxisd;
}

#endif //CVL_COMMON_EIGEN_H_
