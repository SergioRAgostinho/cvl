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
    std::cerr << '[' << __func__ << "] Could find not model inside mat file"
              << std::endl;
    matClose (file);
    return false;
  }

  mxArray* const verts = mxGetField (model, 0, "vertices");
  mxArray* const faces = mxGetField (model, 0, "triangles");
  mxArray* const origin = mxGetField (model, 0, "origin");
  if (!model || !faces || !origin)
  {
    std::cerr << '[' << __func__ << "] Could not parse the structure fields" << std::endl;
    mxDestroyArray (model);
    matClose (file);
    return false;
  }

  const size_t s_v = mxGetNumberOfElements (verts);
  const size_t s_f = mxGetNumberOfElements (faces);
  #ifndef NDEBUG
    const size_t s_o = mxGetNumberOfElements (origin);
  #endif

  // make sure data makes sense
  assert (!(s_v % 3));
  assert (!(s_f % 3));
  assert (!(s_o % 3));
  const size_t s_v_n = s_v / 3;
  const size_t s_f_n = s_f / 3;
  mesh = TriMesh (s_v_n, s_f_n, 0);

  // Get access to data
  const double* const data_v = (double*) mxGetData (verts);
  const double* const data_f = (double*) mxGetData (faces);
  const double* const data_o = (double*) mxGetData (origin);

  // Perform copy
  Eigen::Map<Matrix<float, Dynamic, 3>> v_map (mesh.vertices ()->data (), s_v_n, 3);
  v_map = Eigen::Map<const Matrix<double, Dynamic, 3>> (data_v, s_v_n, 3).cast<float> ().rowwise ()
          - Eigen::Map<const RowVector3d> (data_o).cast<float> ();

  Eigen::Map<ArrayX<size_t>> f_map (mesh.faces ()->data (), s_f);
  f_map = Eigen::Map<const ArrayX<double>> (data_f, s_f).cast<size_t> () - 1u;

  mxDestroyArray (model);
  matClose (file);
  return true;
}
