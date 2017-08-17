/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/05/09
  * \date last modified: 2017/05/09
  */
#pragma once
#ifndef CVL_COMMON_CAMERA_H_
#define CVL_COMMON_CAMERA_H_

#include <cvl/common/eigen.h>

namespace ht
{
  template<typename _Float>
  struct Camera
  {
    Matrix43<_Float> k; //vectorization
    Vector4<_Float> rvec; //axis angle
    Vector4<_Float> tvec;
    unsigned int width;
    unsigned int height;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  // Convenient typedefs
  using Cameraf = Camera<float>;
  using Camerad = Camera<double>;
}
#endif //CVL_COMMON_CAMERA_H_
