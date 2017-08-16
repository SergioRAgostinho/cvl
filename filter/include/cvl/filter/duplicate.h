/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/08/11
  * \date last modified: 2017/08/12
  */
#pragma once
#ifndef CVL_FILTER_DUPLICATE_H_
#define CVL_FILTER_DUPLICATE_H_

#include <cvl/search/kdtree.h>
#include <cvl/common/mesh.h>

namespace ht
{
  /** \brief Class responsible for filtering duplicate vertexes */
  template<typename _Mesh>
  class DuplicateVertexRemoval
  {
    public:

      /** \brief Type of mesh being filtered */
      typedef _Mesh MeshT;

      /** \brief Shared pointer to the filtered mesh type */
      typedef std::shared_ptr<_Mesh> MeshPtrT;

      /** \brief Default Ctor/Threshold Ctor
        * \param[in] threshold - specifies the maximum distance within two
        * vertexes are considered to be the same
        */
      DuplicateVertexRemoval (const float threshold = 1.f)
        : dirty_ (true)
        , threshold_ (threshold)
      {}

      /** \brief Generates a new filtered mesh
        * \return Returns the new filtered mesh object.
        */
      MeshT filter ();

      /** \brief A getter for the threshold value
        * \return Returns the threshold value
        */
      float getThreshold () const { return threshold_; }

      /** \brief Sets the input mesh
        * \param[in] mesh - a shared pointer to the input mesh
        */
      void setInput (const MeshPtrT& mesh)
      {
        input_ = mesh;
        dirty_ = true;
      };

      /** \brief Threshold setter
        * \param[in] threshold - the new threshold value
        */
      void setThreshold (const float threshold) { threshold_ = threshold; }

    protected:

      /** \brief The input mesh to be filtered */
      MeshPtrT input_;

      /** \brief A flag indicating if the kdtree needs to be rebuilt */
      bool dirty_;

      /** \brief The filtering threshold storing the maximum distance in which
        * two points are considered similar
        */
      float threshold_;

      /** \brief An auxiliary KDTree used to search for the nearest neighbors */
      KDTree<Matrix<float, Dynamic, 3>> tree_;
  };
}

#include <cvl/filter/impl/duplicate.hpp>

#endif //CVL_FILTER_DUPLICATE_H_
