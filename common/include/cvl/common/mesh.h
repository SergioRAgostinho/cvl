/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/05/31
  * \date last modified: 2017/05/31
  */
#pragma once
#ifndef CVL_COMMON_CAMERA_H_
#define CVL_COMMON_CAMERA_H_

namespace ht
{
  struct TriMesh
  {
    /** \brief vertices */
    float* v;

    /** \brief Number of vertices */
    size_t n_v;

    float* n;

    size_t n_n;

    size_t* f;

    size_t n_f;
  };
}
#endif //CVL_COMMON_CAMERA_H_
