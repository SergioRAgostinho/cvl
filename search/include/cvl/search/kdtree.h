/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/08/07
  * \date last modified: 2017/08/17
  */
#pragma once
#ifndef CVL_SEARCH_KDTREE_H_
#define CVL_SEARCH_KDTREE_H_

#include <cvl/common/eigen.h>
#include <nanoflann.hpp>

namespace ht
{
  template<typename _EigenMatrix>
  class KDTree
  {
    public:

      /////////////////////////////////
      //            Types
      /////////////////////////////////

      /** \brief Index type returned in the queries */
      typedef typename _EigenMatrix::Index Index;

      /** \brief Scalar type of the data */
      typedef typename _EigenMatrix::Scalar Scalar;

      /** \brief Input type */
      typedef _EigenMatrix Input;

      /** \brief Shared pointer to input type */
      typedef std::shared_ptr<Input> InputPtr;

      /** \brief Shared pointer to const input type */
      typedef std::shared_ptr<const Input> InputConstPtr;

      /////////////////////////////////
      //            Methods
      /////////////////////////////////

      KDTree (const int leaf_max_size = 20)
        : leaf_max_size_ (leaf_max_size)
      {}

      /** \brief Generates the tree index for provided data set */
      void generateTreeIndex ()
      {
        tree_ = std::make_unique<Tree> (*input_, leaf_max_size_);
        tree_->index->buildIndex ();
      }

      /** \brief Return the maximum leaf size */
      int getLeafMaxSize () const { return leaf_max_size_; }

      /** \brief Query the tree for a given points
        * \param[out] idxs - indexes (row number) of the closest points
        * \param[out] dists - squared distances to closest points
        * \param[in] pt - query points
        * \param[in] num_neighbors - number of neighbors to search for
        * \param[in] sorted - if the results should come sorted based on
        * the distance to the query point
        */
      template<typename _Derived>
      void query (std::vector<Index>& idxs,
                  std::vector<Scalar>& dists,
                  const Eigen::DenseBase<_Derived>& pt,
                  const size_t num_neighbors = 1,
                  const bool sorted = true);

      /** \brief Set the new data set to perform queries on
        * \param[in] input - a shared pointer to and NxDim Eigen matrix,
        * N being the number of samples and Dim the dimension of
        * each sample
        */
      void setInput (const InputConstPtr& input) { input_ = input; }

      /** \brief Sets the maximum leaf size
        * \parama[in] leaf_max_size - the maximum leaf size
        */
      void setLeafMaxSize (const int leaf_max_size) { leaf_max_size_ = leaf_max_size; }

    protected:

      /////////////////////////////////
      //            Types
      /////////////////////////////////

      // typedef nanoflann::KDTreeEigenMatrixAdaptor<_EigenMatrix, _EigenMatrix::ColsAtCompileTime> Tree;
      typedef nanoflann::KDTreeEigenMatrixAdaptor<_EigenMatrix> Tree;

      /////////////////////////////////
      //            Members
      /////////////////////////////////

      /** \brief The internal KDtree from nanoflann */
      std::unique_ptr<Tree> tree_;

      /** \brief Inout data */
      InputConstPtr input_;

      /** \brief Maximum size of leaf */
      int leaf_max_size_;
  };
}

// Implementation file
#include <cvl/search/impl/kdtree.hpp>

#endif //CVL_SEARCH_KDTREE_H_
