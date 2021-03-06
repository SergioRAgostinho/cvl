/*
* \author Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com
* \date created: 2017/08/11
* \date last modified: 2017/08/12
*/
template<typename _Mesh> _Mesh
ht::DuplicateVertexRemoval<_Mesh>::filter ()
{
  // Map input data
  std::vector<float>& v = *input_->vertices ();
  const size_t n_v = input_->sizeVertices ();
  Eigen::Map<Matrix<float, Dynamic, 3>> mat (v.data (), n_v, 3);

  // If input is dirty, the tree needs to be regenerated
  if (dirty_)
  {
    tree_.setInput (std::make_shared<Matrix<float, Dynamic, 3>> (mat));
    tree_.generateTreeIndex ();
    dirty_ = false;
  }

  // Populate index map
  std::vector<size_t> index_map (n_v);
  for (size_t i = 0; i < n_v; ++i)
    index_map[i] = i;

  // Query each point
  std::vector<Matrix<float, Dynamic, 3>::Index> idx (2);
  std::vector<Matrix<float, Dynamic, 3>::Scalar> dist (2);
  std::vector<size_t> sub (n_v);
  const float th = threshold_ * threshold_;
  size_t unique_pts = n_v;

  for (size_t i = 0; i < n_v; ++i)
  {
    const Eigen::Map<Vector3f> pt = input_->vertex (i);
    tree_.query (idx, dist, pt, 2);

    size_t& i1 = index_map[i];
    size_t& i2 = index_map[idx[1]];

    if (dist[1] > th || (input_->vertex (i1) - input_->vertex (i2)).squaredNorm () > th)
      continue;

    if (i1 < i2)
    {
      if (i2 == size_t (idx[1]))
        --unique_pts;
      i2 = i1;
    }
    else
    {
      if (i1 == i)
        --unique_pts;
      i1 = i2;
    }
  }

  // Build final mesh
  MeshT mesh (unique_pts, input_->sizeFaces (), 0);
  std::vector<size_t> red_idx (n_v);

  // populate vertices and faces and reduce index vector
  for (size_t i = 0, j = 0, k = 0; i < n_v; ++i)
  {
    sub[i] = j;
    size_t& idx = index_map[i];

    red_idx[i] = idx - sub[idx];
    if (idx != i)
      ++j;
    else
      mesh.vertex (k++) = input_->vertex (i);
  }

  // std::cout << "idx:";
  // for (auto idx : index_map)
  //   std::cout << ' ' << idx;
  // std::cout << std::endl;

  // std::cout << "sub:";
  // for (auto idx : sub)
  //   std::cout << ' ' << idx;
  // std::cout << std::endl;


  // std::cout << "Red idx:";
  // for (auto idx : red_idx)
  //   std::cout << ' ' << idx;
  // std::cout << std::endl;


  // populate faces
  for (size_t i = 0; i < mesh.faces ()->size (); ++i)
    (*mesh.faces ())[i] = red_idx[(*input_->faces ())[i]];

  return mesh;
}
