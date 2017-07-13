/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/05/09
  * \date last modified: 2017/05/11
  */
#pragma once
#ifndef CVL_COMMON_GEOMETRY_H_
#define CVL_COMMON_GEOMETRY_H_

#include <cvl/common/eigen.h>

namespace ht
{

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
              AngleAxis<Scalar> (aa[0], aa.tail(3)).toRotationMatrix ()
            ) * vec;
  }
}

#include <cvl/common/impl/geometry.hpp>

#endif //CVL_COMMON_GEOMETRY_H_
