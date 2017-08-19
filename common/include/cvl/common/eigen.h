/**
  * \author Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com
  * \date created: 2017/05/05
  * \date last modified: 2017/08/19
  * \file eigen.h
  * \brief Defines a number of useful Eigen aliases
  */
#pragma once
#ifndef CVL_COMMON_EIGEN_H_
#define CVL_COMMON_EIGEN_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ht
{
  /** \addtogroup common
   *  @{
   */

  using Eigen::Dynamic;
  using Eigen::RowMajor;
  using Eigen::ColMajor;

  // Dense
  using Eigen::DenseBase;

  // Matrix
  using Eigen::MatrixBase;

  /** \brief Exposes the Eigen::Matrix type, defaulting to RowMajor ordering */
  template<typename _Scalar, int _Rows, int _Cols, int _Options = RowMajor>
  using Matrix = Eigen::Matrix<_Scalar, _Rows, _Cols, _Options>;

  /** \brief Dynamic size Matrix alias. Defaults to RowMajor ordering. */
  template<typename _Scalar, int _Options = RowMajor>
  using MatrixX = Matrix<_Scalar, Dynamic, Dynamic, _Options>;

  /** \brief An instantiation of a Dynamic sized Matrix with a byte sized scalar. */
  typedef MatrixX<uint8_t> MatrixXb;

  /** \brief An instantiation of a Dynamic sized Matrix with an integer scalar. */
  typedef MatrixX<int> MatrixXi;

  /** \brief An instantiation of a Dynamic sized Matrix with an unsigned integer scalar. */
  typedef MatrixX<unsigned int> MatrixXu;

  /** \brief An instantiation of a Dynamic sized Matrix with a float scalar. */
  typedef MatrixX<float> MatrixXf;

  /** \brief An instantiation of a Dynamic sized Matrix with a double scalar. */
  typedef MatrixX<double> MatrixXd;

  /** \brief A 2x2 size Matrix alias. Defaults to RowMajor ordering. */
  template<typename _Scalar, int _Options = RowMajor>
  using Matrix2 = Matrix<_Scalar, 2, 2, _Options>;

  /** \brief A 3x3 size Matrix alias. Defaults to RowMajor ordering. */
  template<typename _Scalar, int _Options = RowMajor>
  using Matrix3 = Matrix<_Scalar, 3, 3, _Options>;

  /** \brief A 4x4 size Matrix alias. Defaults to RowMajor ordering. */
  template<typename _Scalar, int _Options = RowMajor>
  using Matrix4 = Matrix<_Scalar, 4, 4, _Options>;

  /** \brief An instantiation of a 2x2 sized Matrix with a byte sized scalar. */
  typedef Matrix2<uint8_t> Matrix2b;

  /** \brief An instantiation of a 2x2 sized Matrix with an integer scalar. */
  typedef Matrix2<int> Matrix2i;

  /** \brief An instantiation of a 2x2 sized Matrix with an unsigned integer scalar. */
  typedef Matrix2<unsigned int> Matrix2u;

  /** \brief An instantiation of a 2x2 sized Matrix with a float scalar. */
  typedef Matrix2<float> Matrix2f;

  /** \brief An instantiation of a 2x2 sized Matrix with a double scalar. */
  typedef Matrix2<double> Matrix2d;

  /** \brief An instantiation of a 3x3 sized Matrix with a byte sized scalar. */
  typedef Matrix3<uint8_t> Matrix3b;

  /** \brief An instantiation of a 3x3 sized Matrix with an integer scalar. */
  typedef Matrix3<int> Matrix3i;

  /** \brief An instantiation of a 3x3 sized Matrix with an unsigned integer scalar. */
  typedef Matrix3<unsigned int> Matrix3u;

  /** \brief An instantiation of a 3x3 sized Matrix with a float scalar. */
  typedef Matrix3<float> Matrix3f;

  /** \brief An instantiation of a 3x3 sized Matrix with a double scalar. */
  typedef Matrix3<double> Matrix3d;

  /** \brief An instantiation of a 4x4 sized Matrix with a byte sized scalar. */
  typedef Matrix4<uint8_t> Matrix4b;

  /** \brief An instantiation of a 4x4 sized Matrix with an integer scalar. */
  typedef Matrix4<int> Matrix4i;

  /** \brief An instantiation of a 4x4 sized Matrix with an unsigned integer scalar. */
  typedef Matrix4<unsigned int> Matrix4u;

  /** \brief An instantiation of a 4x4 sized Matrix with a float scalar. */
  typedef Matrix4<float> Matrix4f;

  /** \brief An instantiation of a 4x4 sized Matrix with a double scalar. */
  typedef Matrix4<double> Matrix4d;

  /** \brief A 4x3 size Matrix alias. Defaults to RowMajor ordering. */
  template<typename _Scalar, int _Options = RowMajor>
  using Matrix43 = Matrix<_Scalar, 4, 3, _Options>;

  /** \brief An instantiation of a 4x3 sized Matrix with a byte sized scalar. */
  typedef Matrix43<uint8_t> Matrix43b;

  /** \brief An instantiation of a 4x3 sized Matrix with an integer scalar. */
  typedef Matrix43<int> Matrix43i;

  /** \brief An instantiation of a 4x3 sized Matrix with an unsigned integer scalar. */
  typedef Matrix43<unsigned int> Matrix43u;

  /** \brief An instantiation of a 4x3 sized Matrix with a float scalar. */
  typedef Matrix43<float> Matrix43f;

  /** \brief An instantiation of a 4x3 sized Matrix with a double scalar. */
  typedef Matrix43<double> Matrix43d;

  // Vector
  /** \brief Exposes the Eigen::Vector type (Column vector). */
  template<typename _Scalar, short _Dim>
  using Vector = Eigen::Matrix<_Scalar, _Dim, 1>;

  /** \brief A 2 element Vector alias. */
  template<typename _Scalar>
  using Vector2 = Vector<_Scalar, 2>;

  /** \brief A 3 element Vector alias. */
  template<typename _Scalar>
  using Vector3 = Vector<_Scalar, 3>;

  /** \brief A 4 element Vector alias. */
  template<typename _Scalar>
  using Vector4 = Vector<_Scalar, 4>;

  /** \brief A Dynamic size element Vector alias. */
  template<typename _Scalar>
  using VectorX = Vector<_Scalar, Dynamic>;

  /** \brief An instantiation of a 2 element Vector with a byte sized scalar. */
  typedef Vector2<uint8_t> Vector2b;

  /** \brief An instantiation of a 2 element Vector with an integer sized scalar. */
  typedef Vector2<int> Vector2i;

  /** \brief An instantiation of a 2 element Vector with an unsigned integer sized scalar. */
  typedef Vector2<unsigned int> Vector2u;

  /** \brief An instantiation of a 2 element Vector with a float sized scalar. */
  typedef Vector2<float> Vector2f;

  /** \brief An instantiation of a 2 element Vector with a double sized scalar. */
  typedef Vector2<double> Vector2d;

  /** \brief An instantiation of a 3 element Vector with a byte sized scalar. */
  typedef Vector3<uint8_t> Vector3b;

  /** \brief An instantiation of a 3 element Vector with an integer scalar. */
  typedef Vector3<int> Vector3i;

  /** \brief An instantiation of a 3 element Vector with an unsigned integer scalar. */
  typedef Vector3<unsigned int> Vector3u;

  /** \brief An instantiation of a 3 element Vector with a float scalar. */
  typedef Vector3<float> Vector3f;

  /** \brief An instantiation of a 3 element Vector with a double scalar. */
  typedef Vector3<double> Vector3d;

  /** \brief An instantiation of a 4 element Vector with a byte sized scalar. */
  typedef Vector4<uint8_t> Vector4b;

  /** \brief An instantiation of a 4 element Vector with an integer scalar. */
  typedef Vector4<int> Vector4i;

  /** \brief An instantiation of a 4 element Vector with an unsigned integer scalar. */
  typedef Vector4<unsigned int> Vector4u;

  /** \brief An instantiation of a 4 element Vector with a float scalar. */
  typedef Vector4<float> Vector4f;

  /** \brief An instantiation of a 4 element Vector with a double scalar. */
  typedef Vector4<double> Vector4d;

  /** \brief An instantiation of a Dynamic size Vector with a byte sized scalar. */
  typedef VectorX<uint8_t> VectorXb;

  /** \brief An instantiation of a Dynamic size Vector with an integer scalar. */
  typedef VectorX<int> VectorXi;

  /** \brief An instantiation of a Dynamic size Vector with an unsigned integer scalar. */
  typedef VectorX<unsigned int> VectorXu;

  /** \brief An instantiation of a Dynamic size Vector with a float scalar. */
  typedef VectorX<float> VectorXf;

  /** \brief An instantiation of a Dynamic size Vector with a double scalar. */
  typedef VectorX<double> VectorXd;

  // RowVector
  /** \brief Exposes the Eigen::RowVector type (Row vector). */
  template<typename _Scalar, short _Dim>
  using RowVector = Eigen::Matrix<_Scalar, 1, _Dim>;

  /** \brief A 2 element RowVector alias. */
  template<typename _Scalar>
  using RowVector2 = RowVector<_Scalar, 2>;

  /** \brief A 3 element RowVector alias. */
  template<typename _Scalar>
  using RowVector3 = RowVector<_Scalar, 3>;

  /** \brief A 4 element RowVector alias. */
  template<typename _Scalar>
  using RowVector4 = RowVector<_Scalar, 4>;

  /** \brief A Dynamic size RowVector alias. */
  template<typename _Scalar>
  using RowVectorX = RowVector<_Scalar, Dynamic>;

  /** \brief An instantiation of a 2 element RowVector with a byte sized scalar. */
  typedef RowVector2<uint8_t> RowVector2b;

  /** \brief An instantiation of a 2 element RowVector with an integer scalar. */
  typedef RowVector2<int> RowVector2i;

  /** \brief An instantiation of a 2 element RowVector with an unsigned integer scalar. */
  typedef RowVector2<unsigned int> RowVector2u;

  /** \brief An instantiation of a 2 element RowVector with a float scalar. */
  typedef RowVector2<float> RowVector2f;

  /** \brief An instantiation of a 2 element RowVector with a double scalar. */
  typedef RowVector2<double> RowVector2d;

  /** \brief An instantiation of a 3 element RowVector with a byte sized scalar. */
  typedef RowVector3<uint8_t> RowVector3b;

  /** \brief An instantiation of a 3 element RowVector with an integer scalar. */
  typedef RowVector3<int> RowVector3i;

  /** \brief An instantiation of a 3 element RowVector with an unsigned integer scalar. */
  typedef RowVector3<unsigned int> RowVector3u;

  /** \brief An instantiation of a 3 element RowVector with a float scalar. */
  typedef RowVector3<float> RowVector3f;

  /** \brief An instantiation of a 3 element RowVector with a double scalar. */
  typedef RowVector3<double> RowVector3d;

  /** \brief An instantiation of a 4 element RowVector with a byte sized scalar. */
  typedef RowVector4<uint8_t> RowVector4b;

  /** \brief An instantiation of a 4 element RowVector with an integer scalar. */
  typedef RowVector4<int> RowVector4i;

  /** \brief An instantiation of a 4 element RowVector with an unsigned integer scalar. */
  typedef RowVector4<unsigned int> RowVector4u;

  /** \brief An instantiation of a 4 element RowVector with a float scalar. */
  typedef RowVector4<float> RowVector4f;

  /** \brief An instantiation of a 4 element RowVector with a double scalar. */
  typedef RowVector4<double> RowVector4d;

  /** \brief An instantiation of a Dynamic sized RowVector with a byte sized scalar. */
  typedef RowVectorX<uint8_t> RowVectorXb;

  /** \brief An instantiation of a Dynamic sized RowVector with an integer scalar. */
  typedef RowVectorX<int> RowVectorXi;

  /** \brief An instantiation of a Dynamic sized RowVector with an unsigned integer scalar. */
  typedef RowVectorX<unsigned int> RowVectorXu;

  /** \brief An instantiation of a Dynamic sized RowVector with a float scalar. */
  typedef RowVectorX<float> RowVectorXf;

  /** \brief An instantiation of a Dynamic sized RowVector with a double scalar. */
  typedef RowVectorX<double> RowVectorXd;

  // Array
  /** \brief Exposes the Eigen::Array type, defaulting to RowMajor ordering */
  template<typename _Scalar, int _Rows, int _Cols, int _Options = RowMajor>
  using Array = Eigen::Array<_Scalar, _Rows, _Cols, _Options>;

  /** \brief Bi-dimensional Dynamic size Array alias. Defaults to RowMajor ordering. */
  template<typename _Scalar, int _Options = RowMajor>
  using ArrayXX = Array<_Scalar, Dynamic, Dynamic, _Options>;

  /** \brief An instantiation of a bi-dimensional Dynamic sized Matrix with a byte sized scalar. */
  typedef ArrayXX<uint8_t> ArrayXXb;

  /** \brief An instantiation of a bi-dimensional Dynamic sized Matrix with an integer scalar. */
  typedef ArrayXX<int> ArrayXXi;

  /** \brief An instantiation of a bi-dimensional Dynamic sized Matrix with an unsigned integer scalar. */
  typedef ArrayXX<unsigned int> ArrayXXu;

  /** \brief An instantiation of a bi-dimensional Dynamic sized Matrix with a float scalar. */
  typedef ArrayXX<float> ArrayXXf;

  /** \brief An instantiation of a bi-dimensional Dynamic sized Matrix with a double scalar. */
  typedef ArrayXX<double> ArrayXXd;

  /** \brief A 2x2 size Array alias. Defaults to RowMajor ordering. */
  template<typename _Scalar, int _Options = RowMajor>
  using Array22 = Array<_Scalar, 2, 2, _Options>;

  /** \brief A 3x3 size Array alias. Defaults to RowMajor ordering. */
  template<typename _Scalar, int _Options = RowMajor>
  using Array33 = Array<_Scalar, 3, 3, _Options>;

  /** \brief A 4x4 size Array alias. Defaults to RowMajor ordering. */
  template<typename _Scalar, int _Options = RowMajor>
  using Array44 = Array<_Scalar, 4, 4, _Options>;

  /** \brief An instantiation of a 2x2 sized Array with a byte sized scalar. */
  typedef Array22<uint8_t> Array22b;

  /** \brief An instantiation of a 2x2 sized Array with an integer scalar. */
  typedef Array22<int> Array22i;

  /** \brief An instantiation of a 2x2 sized Array with an unsigned integer scalar. */
  typedef Array22<unsigned int> Array22u;

  /** \brief An instantiation of a 2x2 sized Array with a float scalar. */
  typedef Array22<float> Array22f;

  /** \brief An instantiation of a 2x2 sized Array with a double scalar. */
  typedef Array22<double> Array22d;

  /** \brief An instantiation of a 3x3 sized Array with a byte sized scalar. */
  typedef Array33<uint8_t> Array33b;

  /** \brief An instantiation of a 3x3 sized Array with an integer scalar. */
  typedef Array33<int> Array33i;

  /** \brief An instantiation of a 3x3 sized Array with an unsigned integer scalar. */
  typedef Array33<unsigned int> Array33u;

  /** \brief An instantiation of a 3x3 sized Array with a float scalar. */
  typedef Array33<float> Array33f;

  /** \brief An instantiation of a 3x3 sized Array with a double scalar. */
  typedef Array33<double> Array33d;

  /** \brief An instantiation of a 4x4 sized Array with a byte sized scalar. */
  typedef Array44<uint8_t> Array44b;

  /** \brief An instantiation of a 4x4 sized Array with an integer scalar. */
  typedef Array44<int> Array44i;

  /** \brief An instantiation of a 4x4 sized Array with an unsigned integer scalar. */
  typedef Array44<unsigned int> Array44u;

  /** \brief An instantiation of a 4x4 sized Array with a float scalar. */
  typedef Array44<float> Array44f;

  /** \brief An instantiation of a 4x4 sized Array with a double scalar. */
  typedef Array44<double> Array44d;

  /** \brief A 2 element Array alias. */
  template<typename _Scalar, int _Options = 0>
  using Array2 = Eigen::Array<_Scalar, 2, 1, _Options>;

  /** \brief A 3 element Array alias. */
  template<typename _Scalar, int _Options = 0>
  using Array3 = Array<_Scalar, 3, 1, _Options>;

  /** \brief A 4 element Array alias. */
  template<typename _Scalar, int _Options = 0>
  using Array4 = Array<_Scalar, 4, 1, _Options>;

  /** \brief A Dynamic sized Array alias. */
  template<typename _Scalar, int _Options = 0>
  using ArrayX = Array<_Scalar, Dynamic, 1, _Options>;

  /** \brief An instantiation of a 2 element Array with a byte sized scalar. */
  typedef Array2<uint8_t> Array2b;

  /** \brief An instantiation of a 2 element Array with an integer scalar. */
  typedef Array2<int> Array2i;

  /** \brief An instantiation of a 2 element Array with an unsigned integer scalar. */
  typedef Array2<unsigned int> Array2u;

  /** \brief An instantiation of a 2 element Array with a float scalar. */
  typedef Array2<float> Array2f;

  /** \brief An instantiation of a 2 element Array with a double scalar. */
  typedef Array2<double> Array2d;

  /** \brief An instantiation of a 3 element Array with a byte sized scalar. */
  typedef Array3<uint8_t> Array3b;

  /** \brief An instantiation of a 3 element Array with an integer scalar. */
  typedef Array3<int> Array3i;

  /** \brief An instantiation of a 3 element Array with an unsigned integer scalar. */
  typedef Array3<unsigned int> Array3u;

  /** \brief An instantiation of a 3 element Array with a float scalar. */
  typedef Array3<float> Array3f;

  /** \brief An instantiation of a 3 element Array with a double scalar. */
  typedef Array3<double> Array3d;

  /** \brief An instantiation of a 4 element Array with a byte sized scalar. */
  typedef Array4<uint8_t> Array4b;

  /** \brief An instantiation of a 4 element Array with an integer scalar. */
  typedef Array4<int> Array4i;

  /** \brief An instantiation of a 4 element Array with an unsigned integer scalar. */
  typedef Array4<unsigned int> Array4u;

  /** \brief An instantiation of a 4 element Array with a float scalar. */
  typedef Array4<float> Array4f;

  /** \brief An instantiation of a 4 element Array with a double scalar. */
  typedef Array4<double> Array4d;

  /** \brief An instantiation of a Dynamic sized Array with a byte sized scalar. */
  typedef ArrayX<uint8_t> ArrayXb;

  /** \brief An instantiation of a Dynamic sized Array with an integer scalar. */
  typedef ArrayX<int> ArrayXi;

  /** \brief An instantiation of a Dynamic sized Array with an unsigned integer scalar. */
  typedef ArrayX<unsigned int> ArrayXu;

  /** \brief An instantiation of a Dynamic sized Array with a float scalar. */
  typedef ArrayX<float> ArrayXf;

  /** \brief An instantiation of a Dynamic sized Array with a double scalar. */
  typedef ArrayX<double> ArrayXd;

  // Quaternion
  using Eigen::QuaternionBase;
  using Eigen::Quaternion;
  using Eigen::Quaternionf;
  using Eigen::Quaterniond;

  /** @}*/
}


#endif //CVL_COMMON_EIGEN_H_
