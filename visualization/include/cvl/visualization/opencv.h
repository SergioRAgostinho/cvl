/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/07/30
  * \date last modified: 2017/07/30
  */
#pragma once
#ifndef CVL_VISUALIZATION_OPENCV_H_
#define CVL_VISUALIZATION_OPENCV_H_

#include <cvl/common/mesh.h>
#include <cvl/common/eigen.h>
#include <opencv2/core.hpp>

namespace ht
{
  /** \brief Draw an edge mesh model into an opencv matrix
    */
	void draw ( cv::Mat& img,
              const EdgeMesh& mesh,
              const Matrix3f& k = Matrix3f::Identity (),
              const Vector4f& rvec = Vector4f (0.f, 0.f, 0.f, 1.f),
              const Vector4f& tvec = Vector4f (0.f, 0.f, 0.f, 0.f));
}
#endif //CVL_VISUALIZATION_OPENCV_H_
