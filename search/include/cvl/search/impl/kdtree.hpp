/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/08/10
  * \date last modified: 2017/08/10
  */

////////////////////////////////////////
//              Input
////////////////////////////////////////

template<typename _EigenMatrix>
ht::KDTree<_EigenMatrix>::KDTree (const int leaf_max_size)
  : leaf_max_size_ (leaf_max_size)
{}

template<typename _EigenMatrix> void
ht::KDTree<_EigenMatrix>::generateTreeIndex ()
{
  tree_ = std::make_unique<Tree> (input_->cols (),
                                  *input_,
                                  leaf_max_size_);
  tree_->index->buildIndex ();
}

template<typename _EigenMatrix> int
ht::KDTree<_EigenMatrix>::getLeafMaxSize () const
{
  return leaf_max_size_;
}

template<typename _EigenMatrix> template<typename _Derived> void
ht::KDTree<_EigenMatrix>::query ( std::vector<typename ht::KDTree<_EigenMatrix>::Index>& idxs,
                                  std::vector<typename ht::KDTree<_EigenMatrix>::Scalar>& dists,
                                  const Eigen::DenseBase<_Derived>& pt,
                                  const size_t num_neighbors,
                                  const bool sorted)
{
  // Make sure query point size is right
  EIGEN_STATIC_ASSERT_VECTOR_ONLY (Eigen::DenseBase<_Derived>);
  assert (pt.size () == input_->cols ());

  // Prepare output
  idxs.resize (num_neighbors);
  dists.resize (num_neighbors);
  nanoflann::KNNResultSet<Scalar, Index> set (num_neighbors);
  set.init (idxs.data (), dists.data ());

  //perform the query
  tree_->index->findNeighbors (set, &pt[0], nanoflann::SearchParams (32, 0, sorted));
}

template<typename _EigenMatrix> void
ht::KDTree<_EigenMatrix>::setInput (const ht::KDTree<_EigenMatrix>::InputConstPtr& input)
{
  input_ = input;
}

template<typename _EigenMatrix> void
ht::KDTree<_EigenMatrix>::setLeafMaxSize (const int leaf_max_size)
{
  leaf_max_size_ = leaf_max_size;
}