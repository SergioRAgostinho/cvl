/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/07/27
  * \date last modified: 2017/07/27
  */
#include <cvl/io/vgm_model_reader.h>
#include <mat.h>
#include <algorithm>

bool
ht::vgm_model_reader (ht::TriMesh& mesh, const char* const path)
{
  // Open mat file
  MATFile* const file = matOpen (path, "r");
  if (!file)
  {
    std::cerr <<  '[' << __func__ << "] Unable to open file." << std::endl;
    return false;
  }

  mxArray* const model = matGetVariable (file, "model");
  if (!model)
  {
    std::cerr << '[' << __func__ << "] Could not model inside mat file"
              << std::endl;
    matClose (file);
    return false;
  }

  mxArray* const verts = mxGetField (model, 0, "vertices");
  mxArray* const faces = mxGetField (model, 0, "triangles");
  if (!model || !faces)
  {
    std::cerr << '[' << __func__ << "] Could not parse the structure fields" << std::endl;
    mxDestroyArray (model);
    matClose (file);
    return false;
  }

  const size_t s_v = mxGetNumberOfElements (verts);
  const size_t s_f = mxGetNumberOfElements (faces);

  // make sure data makes sense
  assert (!(s_v % 3));
  assert (!(s_f % 3));
  mesh.v.resize (s_v);
  mesh.f.resize (s_f);

  // Get access to data
  const double* const data_v = (double*) mxGetData (verts);
  const double* const data_f = (double*) mxGetData (faces);

  // Perform copy
  std::copy (data_v, data_v + s_v, mesh.v.begin ());
  std::copy (data_f, data_f + s_f, mesh.f.begin ());

  mxDestroyArray (model);
  matClose (file);
  return true;
}
