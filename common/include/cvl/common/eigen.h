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

  // Dense
  using Eigen::DenseBase;

  // Matrix
  using Eigen::MatrixBase;

  template<typename _Scalar, int _Rows, int _Cols, int _Options = RowMajor>
  using Matrix = Eigen::Matrix<_Scalar, _Rows, _Cols, _Options>;

  template<typename _Scalar, int _Options = RowMajor>
  using MatrixX = Matrix<_Scalar, Dynamic, Dynamic, _Options>;

  typedef MatrixX<uint8_t> MatrixXb;
  typedef MatrixX<int> MatrixXi;
  typedef MatrixX<unsigned int> MatrixXu;
  typedef MatrixX<float> MatrixXf;
  typedef MatrixX<double> MatrixXd;

  template<typename _Scalar, int _Options = RowMajor>
  using Matrix2 = Matrix<_Scalar, 2, 2, _Options>;

  template<typename _Scalar, int _Options = RowMajor>
  using Matrix3 = Matrix<_Scalar, 3, 3, _Options>;

  template<typename _Scalar, int _Options = RowMajor>
  using Matrix4 = Matrix<_Scalar, 4, 4, _Options>;

  typedef Matrix2<uint8_t> Matrix2b;
  typedef Matrix2<int> Matrix2i;
  typedef Matrix2<unsigned int> Matrix2u;
  typedef Matrix2<float> Matrix2f;
  typedef Matrix2<double> Matrix2d;

  typedef Matrix3<uint8_t> Matrix3b;
  typedef Matrix3<int> Matrix3i;
  typedef Matrix3<unsigned int> Matrix3u;
  typedef Matrix3<float> Matrix3f;
  typedef Matrix3<double> Matrix3d;

  typedef Matrix4<uint8_t> Matrix4b;
  typedef Matrix4<int> Matrix4i;
  typedef Matrix4<unsigned int> Matrix4u;
  typedef Matrix4<float> Matrix4f;
  typedef Matrix4<double> Matrix4d;

  template<typename _Scalar, int _Options = RowMajor>
  using Matrix43 = Matrix<_Scalar, 4, 3, _Options>;

  typedef Matrix43<uint8_t> Matrix43b;
  typedef Matrix43<int> Matrix43i;
  typedef Matrix43<unsigned int> Matrix43u;
  typedef Matrix43<float> Matrix43f;
  typedef Matrix43<double> Matrix43d;

  // Vector
  template<typename _Scalar, short _Dim>
  using Vector = Eigen::Matrix<_Scalar, _Dim, 1>;

  template<typename _Scalar>
  using Vector2 = Vector<_Scalar, 2>;

  template<typename _Scalar>
  using Vector3 = Vector<_Scalar, 3>;

  template<typename _Scalar>
  using Vector4 = Vector<_Scalar, 4>;

  template<typename _Scalar>
  using VectorX = Vector<_Scalar, Dynamic>;

  typedef Vector2<uint8_t> Vector2b;
  typedef Vector2<int> Vector2i;
  typedef Vector2<unsigned int> Vector2u;
  typedef Vector2<float> Vector2f;
  typedef Vector2<double> Vector2d;

  typedef Vector3<uint8_t> Vector3b;
  typedef Vector3<int> Vector3i;
  typedef Vector3<unsigned int> Vector3u;
  typedef Vector3<float> Vector3f;
  typedef Vector3<double> Vector3d;

  typedef Vector4<uint8_t> Vector4b;
  typedef Vector4<int> Vector4i;
  typedef Vector4<unsigned int> Vector4u;
  typedef Vector4<float> Vector4f;
  typedef Vector4<double> Vector4d;

  typedef VectorX<uint8_t> VectorXb;
  typedef VectorX<int> VectorXi;
  typedef VectorX<unsigned int> VectorXu;
  typedef VectorX<float> VectorXf;
  typedef VectorX<double> VectorXd;

  // RowVector
  template<typename _Scalar, short _Dim>
  using RowVector = Eigen::Matrix<_Scalar, 1, _Dim>;

  template<typename _Scalar>
  using RowVector2 = RowVector<_Scalar, 2>;

  template<typename _Scalar>
  using RowVector3 = RowVector<_Scalar, 3>;

  template<typename _Scalar>
  using RowVector4 = RowVector<_Scalar, 4>;

  template<typename _Scalar>
  using RowVectorX = RowVector<_Scalar, Dynamic>;

  typedef RowVector2<uint8_t> RowVector2b;
  typedef RowVector2<int> RowVector2i;
  typedef RowVector2<unsigned int> RowVector2u;
  typedef RowVector2<float> RowVector2f;
  typedef RowVector2<double> RowVector2d;

  typedef RowVector3<uint8_t> RowVector3b;
  typedef RowVector3<int> RowVector3i;
  typedef RowVector3<unsigned int> RowVector3u;
  typedef RowVector3<float> RowVector3f;
  typedef RowVector3<double> RowVector3d;

  typedef RowVector4<uint8_t> RowVector4b;
  typedef RowVector4<int> RowVector4i;
  typedef RowVector4<unsigned int> RowVector4u;
  typedef RowVector4<float> RowVector4f;
  typedef RowVector4<double> RowVector4d;

  typedef RowVectorX<uint8_t> RowVectorXb;
  typedef RowVectorX<int> RowVectorXi;
  typedef RowVectorX<unsigned int> RowVectorXu;
  typedef RowVectorX<float> RowVectorXf;
  typedef RowVectorX<double> RowVectorXd;

  // Array
  template<typename _Scalar, int _Rows, int _Cols, int _Options = RowMajor>
  using Array = Eigen::Array<_Scalar, _Rows, _Cols, _Options>;

  template<typename _Scalar, int _Options = RowMajor>
  using ArrayXX = Array<_Scalar, Dynamic, Dynamic, _Options>;

  typedef ArrayXX<uint8_t> ArrayXXb;
  typedef ArrayXX<int> ArrayXXi;
  typedef ArrayXX<unsigned int> ArrayXXu;
  typedef ArrayXX<float> ArrayXXf;
  typedef ArrayXX<double> ArrayXXd;

  template<typename _Scalar, int _Options = RowMajor>
  using Array22 = Array<_Scalar, 2, 2, _Options>;

  template<typename _Scalar, int _Options = RowMajor>
  using Array33 = Array<_Scalar, 3, 3, _Options>;

  template<typename _Scalar, int _Options = RowMajor>
  using Array44 = Array<_Scalar, 4, 4, _Options>;

  typedef Array22<uint8_t> Array22b;
  typedef Array22<int> Array22i;
  typedef Array22<unsigned int> Array22u;
  typedef Array22<float> Array22f;
  typedef Array22<double> Array22d;

  typedef Array33<uint8_t> Array33b;
  typedef Array33<int> Array33i;
  typedef Array33<unsigned int> Array33u;
  typedef Array33<float> Array33f;
  typedef Array33<double> Array33d;

  typedef Array44<uint8_t> Array44b;
  typedef Array44<int> Array44i;
  typedef Array44<unsigned int> Array44u;
  typedef Array44<float> Array44f;
  typedef Array44<double> Array44d;

  template<typename _Scalar, int _Options = 0>
  using Array2 = Eigen::Array<_Scalar, 2, 1, _Options>;

  template<typename _Scalar, int _Options = 0>
  using Array3 = Array<_Scalar, 3, 1, _Options>;

  template<typename _Scalar, int _Options = 0>
  using Array4 = Array<_Scalar, 4, 1, _Options>;

  template<typename _Scalar, int _Options = 0>
  using ArrayX = Array<_Scalar, Dynamic, 1, _Options>;

  typedef Array2<uint8_t> Array2b;
  typedef Array2<int> Array2i;
  typedef Array2<unsigned int> Array2u;
  typedef Array2<float> Array2f;
  typedef Array2<double> Array2d;

  typedef Array3<uint8_t> Array3b;
  typedef Array3<int> Array3i;
  typedef Array3<unsigned int> Array3u;
  typedef Array3<float> Array3f;
  typedef Array3<double> Array3d;

  typedef Array4<uint8_t> Array4b;
  typedef Array4<int> Array4i;
  typedef Array4<unsigned int> Array4u;
  typedef Array4<float> Array4f;
  typedef Array4<double> Array4d;

  typedef ArrayX<uint8_t> ArrayXb;
  typedef ArrayX<int> ArrayXi;
  typedef ArrayX<unsigned int> ArrayXu;
  typedef ArrayX<float> ArrayXf;
  typedef ArrayX<double> ArrayXd;

  // Quaternion
  using Eigen::QuaternionBase;
  using Eigen::Quaternion;
  using Eigen::Quaternionf;
  using Eigen::Quaterniond;
}

#endif //CVL_COMMON_EIGEN_H_
