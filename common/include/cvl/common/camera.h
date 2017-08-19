/**
  * \author Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com
  * \date created: 2017/05/09
  * \date last modified: 2017/08/19
  * \file camera.h
  * \brief Holds all important type definitions pertaining cameras
  */
#pragma once
#ifndef CVL_COMMON_CAMERA_H_
#define CVL_COMMON_CAMERA_H_

#include <cvl/common/eigen.h>

namespace ht
{
  /** \addtogroup common
   *  @{
   */

  /** \struct Camera
    * \brief Packs together all relevant information related to camera:
    * intrinsics matrix, pose, and image size.
    *
    * Templated on floating point precision type.
    */
  template<typename _Float>
  struct Camera
  {
    /** \brief Intrinsics matrix
      *
      * The extra size is to allow for vectorization optimizations
      * if required
      */
    Matrix43<_Float> k; //vectorization

    /** \brief Axis-angle encoding of a rotation from world to camera
      * coordinates
      */
    Vector4<_Float> rvec;

    /** \brief Word's frame of reference origin in Camera's coordinates.
      *
      * Packed as 4 component vector to allow vectorization optimizations.
      * The last component is set to 0
      */
    Vector4<_Float> tvec;

    /** \brief Width of the image acquired by the camera */
    unsigned int width;

    /** \brief Height of the image acquired by the camera */
    unsigned int height;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  // Convenient typedefs
  /** \brief Alias for the float instantiation of the Camera type. */
  using Cameraf = Camera<float>;

  /** \brief Alias for the double instantiation of the Camera type. */
  using Camerad = Camera<double>;

  /** @}*/
}

#endif //CVL_COMMON_CAMERA_H_
