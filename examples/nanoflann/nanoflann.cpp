/**
  * \author Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com
  * \date created: 2017/08/07
  * \date last modified: 2017/08/07
  */
#include <cvl/search/kdtree.h>
#include <nanoflann.hpp>
#include <Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<float,
                      Eigen::Dynamic,
                      Eigen::Dynamic,
                      Eigen::RowMajor> MatrixXf;
typedef nanoflann::KDTreeEigenMatrixAdaptor<MatrixXf>  KDTree;

int
main ()
{
  //populate points
  float pts[] =
  {
    0.f, 0.f, 0.f,
    1.f, 1.f, 1.f,
    0.f, 0.f, 5.f,
    10.f, 0.f, 0.f
  };
  Eigen::Map<MatrixXf> map (pts, 4, 3);

  //Generate tree index
  // KDTree tree (3, map, 10);
  KDTree tree (map, 10);
  tree.index->buildIndex ();

  // Prepare a Knn search
  const size_t n_results = 1;
  size_t index;
  float dist;
  nanoflann::KNNResultSet<float> set (n_results);
  set.init (&index, &dist);

  //query
  float pt[3] = {2.f, 2.f, 2.f};
  tree.index->findNeighbors (set, pt, nanoflann::SearchParams ());

  // Display output
  std::cout << "Nanoflann query point: " << pt[0] << ' ' << pt[1] << ' ' << pt[2]
            << "\nClosest index: " << index
            << "\nDistance: " << std::sqrt(dist)
            << std::endl;

  // same test with search class
  ht::KDTree<MatrixXf> cvl_tree;
  cvl_tree.setInput (std::make_shared<MatrixXf> (map));
  cvl_tree.generateTreeIndex ();

  std::vector<MatrixXf::Index> cvl_idx;
  std::vector<MatrixXf::Scalar> cvl_dist;
  const Eigen::Map<ht::Vector3f> cvl_pt (pt);

  // query
  cvl_tree.query (cvl_idx,
                  cvl_dist,
                  cvl_pt,
                  2);
  // Display output
  std::cout << "CVL query point: " << cvl_pt[0] << ' ' << cvl_pt[1] << ' ' << cvl_pt[2]
            << "\nClosest index: " << cvl_idx[0] << ' ' << cvl_idx[1]
            << "\nDistance: " << std::sqrt(cvl_dist[0]) << ' ' << std::sqrt(cvl_dist[1])
            << std::endl;
  return 0;
}