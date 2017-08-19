/**
  * \author Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com
  * \date created: 2017/07/30
  * \date last modified: 2017/08/19
  * \file opencv.h
  * \brief Visualization utilities for visualizing stuff in OpenCV
  */
#pragma once
#ifndef CVL_VISUALIZATION_OPENCV_H_
#define CVL_VISUALIZATION_OPENCV_H_

#include <cvl/common/mesh.h>
#include <cvl/common/eigen.h>
#include <opencv2/core.hpp>

namespace ht
{
  /** \addtogroup visualization
   *  @{
   */

  /** \brief Draw an edge mesh model into an opencv image matrix
    * \param[in,out] img - the input/output image matrix
    * \param[in] mesh - an edge mesh model
    * \param[in] k - camera intrinsics matrix
    * \param[in] rvec - an axis angle vector with World to Camera rotation
    * \param[in] tvec - an translation vector with World origin in Camera FoR
    */
	void draw ( cv::Mat& img,
              const EdgeMesh& mesh,
              const Matrix3f& k = Matrix3f::Identity (),
              const Vector4f& rvec = Vector4f (0.f, 0.f, 1.f, 0.f),
              const Vector4f& tvec = Vector4f (0.f, 0.f, 0.f, 0.f));

  /** @}*/
}

#endif //CVL_VISUALIZATION_OPENCV_H_
