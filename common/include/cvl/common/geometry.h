/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/05/09
  * \date last modified: 2017/08/01
  */
#pragma once
#ifndef CVL_COMMON_GEOMETRY_H_
#define CVL_COMMON_GEOMETRY_H_

#include <cvl/common/eigen.h>

namespace ht
{
  /** \brief Compose to transformations specified by their rvec and
    * tvec vectors
    */
  void compose (Vector4f& rout,
                Vector4f& tout,
                const Vector4f& rvec1,
                const Vector4f& tvec1,
                const Vector4f& rvec2,
                const Vector4f& tvec2);

  /** \brief Estimate the regid transformation from set of points A to B
    * \param[out] rvec - angleaxis vector with the rotation between the two
    * point sets
    * \param[out] tvec - translation vector with between the two point sets
    * \param[in] pts_b - set of points B, size Nx3
    * \param[in] pts_a - set of points A, size Nx3
    */
  void rigid (Vector4f& rvec,
              Vector4f& tvec,
              const Matrix<float, Dynamic, 3, ColMajor>& pts_b,
              const Matrix<float, Dynamic, 3, ColMajor>& pts_a);

  /** \brief Generates a Rodrigues representation from a quaternion */
  template<typename _Derived>
  inline Vector3<typename QuaternionBase<_Derived>::Scalar>
  rodrigues (const QuaternionBase<_Derived>& q)
  {
    using Scalar = typename QuaternionBase<_Derived>::Scalar;
    const AngleAxis<Scalar> aa(q);
    return Vector3<Scalar> (aa.angle () * aa.axis ());
  }

  /** \brief Generates a Rodrigues representation from a quaternion */
  template<typename _Derived>
  inline Vector4<typename QuaternionBase<_Derived>::Scalar>
  angle_axis (const QuaternionBase<_Derived>& q)
  {
    using Scalar = typename QuaternionBase<_Derived>::Scalar;
    const AngleAxis<Scalar> aa(q);
    const Vector3<Scalar>& axis = aa.axis ();
    return Vector4<Scalar> (aa.angle (), axis[0], axis[1], axis[2]);
  }

  /** \brief Constructs an angle axis vector from Euler angles
    *
    * \note The rotation resultant rotation has the following order
    * R = Rx * Ry * Rz
    * \param[in] rx - rotation angle along x (in radians)
    * \param[in] ry - rotation angle along y (in radians)
    * \param[in] rz - rotation angle along z (in radians)
    * \return Returns an axis angle vector.
    */
  template<typename _Float>
  Vector4<_Float> euler_2_angle_axis (const _Float rx,
                                      const _Float ry,
                                      const _Float rz);

  /** \brief Rotate a vector */
  template<typename _Derived>
  inline Vector3<typename MatrixBase<_Derived>::Scalar>
  rotate (const MatrixBase<_Derived>& rvec,
          const MatrixBase<_Derived>& vec)
  {
    using Scalar = typename MatrixBase<_Derived>::Scalar;
    const Scalar norm = rvec.norm ();
    return AngleAxis<Scalar> (norm, (1/norm)* rvec).toRotationMatrix ()
              * vec;
  }

  /** \brief Rotate a vector */
  template<typename _Scalar, typename _Der_vec>
  inline Vector4<typename MatrixBase<_Der_vec>::Scalar>
  rotate4 ( const Vector4<_Scalar>& aa,
            const MatrixBase<_Der_vec>& vec)
  {
    using Scalar = typename MatrixBase<_Der_vec>::Scalar;
    static_assert ( std::is_same<_Scalar, Scalar>::value,
                    "Scalar types must be similar");
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(_Der_vec, 4);
    return Eigen::Transform<Scalar, 3, Eigen::Affine, Eigen::RowMajor> (
              AngleAxis<Scalar> (aa[0], aa.tail (3)).toRotationMatrix ()
            ) * vec;
  }
}

#include <cvl/common/impl/geometry.hpp>

#endif //CVL_COMMON_GEOMETRY_H_
