/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/05/31
  * \date last modified: 2017/07/27
  */
#pragma once
#ifndef CVL_COMMON_MESH_H_
#define CVL_COMMON_MESH_H_

#include <cvl/common/eigen.h>
#include <vector>
#include <iostream>

namespace ht
{
  struct Mesh
  {
    /** \brief vertices */
    std::vector<float> v;

    /** \brief faces */
    std::vector<size_t> f;

    /** \brief normals */
    std::vector<float> n;
  };

  struct TriMesh : public Mesh
  {
    //////////////////////////
    //       Methods
    //////////////////////////

    /** \brief Return the face normal provided it's index
      * \param[in] idx - index
      * \return Returns a normalized vector with the normal
      */
    Vector3f faceNormal (const size_t idx) const;
  };

  struct EdgeMesh
  {
    //////////////////////////
    //       Members
    //////////////////////////

    /** \brief vertices */
    std::vector<float> v;

    /** \brief edges */
    std::vector<size_t> e;

    //////////////////////////
    //       Methods
    //////////////////////////

    /** \brief Default construtor */
    EdgeMesh () = default;

    /** \brief Constructor from TriMesh
      * \param[in] tri - a triangular mesh from which the EdgeMEsh
      * is generated
      * \param[in] filter_angles - the abs of the angle cosine.
      * Any cosine above this value will be filtered. Accepted
      * range ]0,1];
      */
    EdgeMesh (const TriMesh& tri, const float filter_angle = 1.f);

    /** \brief Assignment from TriMesh
      * \param[in] - tri - a triangular mesh from which the EdgeMEsh
      * is generated
      * \return Returns itself (EdgeMesh).
      */
    EdgeMesh& operator= (const TriMesh& tri);

    /** \brief Populates an EdgeMesh from a filtered TriMesh
      * \param[in] tri - a triangular mesh from which the EdgeMEsh
      * is generated
      * \param[in] filter_angles - the abs of the angle cosine.
      * Any cosine above this value will be filtered. Accepted
      * range ]0,1];
      * \return Returns itself (EdgeMesh).
      */
    EdgeMesh& filter (const TriMesh& tri, const float filter_angle);
  };
}

//////////////////////////////////////////
//          Print utilities
//////////////////////////////////////////

/** \brief Stream output operator for a Mesh type
  * \param[in,out] os - output stream
  * \param[in] m - the mesh to be printed
  * \return Returns the modified output stream.
  */
std::ostream& operator<< (std::ostream& os, const ht::Mesh& m);

/** \brief Stream output operator for a TriMesh type
  * \param[in,out] os - output stream
  * \param[in] m - the trimesh to be printed
  * \return Returns the modified output stream.
  */
std::ostream& operator<< (std::ostream& os, const ht::TriMesh& m);

/** \brief Stream output operator for a EdgeMesh type
  * \param[in,out] os - output stream
  * \param[in] m - the edgemesh to be printed
  * \return Returns the modified output stream.
  */
std::ostream& operator<< (std::ostream& os, const ht::EdgeMesh& m);

#endif //CVL_COMMON_MESH_H_
