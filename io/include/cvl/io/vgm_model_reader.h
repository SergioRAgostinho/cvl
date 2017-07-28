/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/07/27
  * \date last modified: 2017/07/27
  */
#pragma once
#ifndef CVL_IO_VGM_MODEL_READER_H_
#define CVL_IO_VGM_MODEL_READER_H_

#include <cvl/common/mesh.h>

namespace ht
{
  /** \brief Parses the model mat files from the Vgm dataset and returns
    * a new TriMesh object with all the data inside.
    * \param[in] path - path to the mat file.
    */
  bool vgm_model_reader (TriMesh& mesh, const char* const path);
}
#endif //CVL_IO_VGM_MODEL_READER_H_
