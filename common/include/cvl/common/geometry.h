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
}

#include <cvl/common/impl/geometry.hpp>

#endif //CVL_COMMON_GEOMETRY_H_
