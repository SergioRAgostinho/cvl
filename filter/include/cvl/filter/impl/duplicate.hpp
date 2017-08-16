/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/08/11
  * \date last modified: 2017/08/12
  */
template<typename _Mesh> _Mesh
ht::DuplicateVertexRemoval<_Mesh>::filter ()
{
  // Map input data
  std::vector<float>& v = input_->v;
  assert (!(v.size () % 3));
  const size_t n_v = v.size () / 3;
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
    const Eigen::Map<Vector3f> pt (&v[3*i]);
    tree_.query (idx, dist, pt, 2);

    auto sq_dist = [] (const float* const p1, const float* const p2)
    {
      return (Eigen::Map<const Vector3f> (p1)
                - Eigen::Map<const Vector3f> (p2)).squaredNorm ();
    };

    size_t& i1 = index_map[i];
    size_t& i2 = index_map[idx[1]];

    if (dist[1] > th || sq_dist (&v[3*i1], &v[3*i2]) > th)
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
  MeshT mesh;
  mesh.v.resize (3 * unique_pts);
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
      std::copy (&v[3*i], &v[3*i] + 3, &mesh.v[3*k++]);
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
  mesh.f.resize (input_->f.size ());
  for (size_t i = 0; i < mesh.f.size (); ++i)
    mesh.f[i] = red_idx[input_->f[i]];

  return mesh;
}
