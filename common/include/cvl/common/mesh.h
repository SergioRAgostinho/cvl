/**
  * \author Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com
  * \date created: 2017/05/31
  * \date last modified: 2017/08/19
  * \file mesh.h
  * \brief All type definitions and utilities regarding mesh primitives
  */
#pragma once
#ifndef CVL_COMMON_MESH_H_
#define CVL_COMMON_MESH_H_

#include <cvl/common/eigen.h>
#include <vector>
#include <iostream>

/** \addtogroup common
 *  @{
 */

namespace ht
{
  // Forward declarations
  class MeshBase;
  class Mesh;
  class TriMesh;
  class EdgeMesh;
}

//////////////////////////////////////////
//          Print utilities
//////////////////////////////////////////

/** \brief Stream output operator for a MeshBase type
  * \param[in,out] os - output stream
  * \param[in] m - the mesh to be printed
  * \return Returns the modified output stream.
  */
std::ostream& operator<< (std::ostream& os, const ht::MeshBase& m);

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

namespace ht
{
  /** \addtogroup common
    *  @{
    */

  /** \brief A container for holding a shared buffer of 3D vertices */
  class MeshBase
  {
    friend std::ostream& ::operator<< (std::ostream& os, const MeshBase& m);

    public:

      /** \brief A vector of floats */
      typedef std::vector<float> VertV;

      /** \brief A shared pointer to a vector of floats */
      typedef std::shared_ptr<VertV> VertVPtr;

      /** \brief A shared pointer to a const vector of floats */
      typedef std::shared_ptr<const VertV> VertVConstPtr;

      /** \brief Default ctor. Initializing an empty buffer */
      MeshBase ()
        : v_ (std::make_shared<VertV> ())
      {}

      /** \brief Initialize size ctor
        * \param[in] n_v - number or vertices
        */
      MeshBase (const size_t n_v)
        : v_ (std::make_shared<VertV> (3 * n_v))
      {}

      /** \brief Initialize buffer copy ctor
        * \param[in] v - the vector buffer to be copied
        */
      MeshBase (const VertV& v)
        : v_ (std::make_shared<VertV> (v))
      {}

      /** \brief Initialize shared ownership ctor
        * \param[in] v - the shared pointer to the vertices' buffer
        */
      MeshBase (const VertVPtr& v)
        : v_ (v)
      {}

      /** \brief Performs a deep copy
        * \return A new MeshBase object with a deep copy of all buffers
        */
      MeshBase clone () const
      {
        return MeshBase (*v_);
      }

      /** \brief Returns an Eigen::Map to the vertex requested
        * \param[in] idx - index of the vertex requested
        * \return A new const Eigen::Map pointing to the vertex requested
        */
      Eigen::Map<const Vector3f> vertex (const size_t idx) const
      {
        return Eigen::Map<const Vector3f> (v_->data () + 3*idx);
      }

      /** \brief Returns an Eigen::Map to the vertex requested
        * \param[in] idx - index of the vertex requested
        * \return A new Eigen::Map pointing to the vertex requested
        */
      Eigen::Map<Vector3f> vertex (const size_t idx)
      {
        return Eigen::Map<Vector3f> (v_->data () + 3*idx);
      }

      /** \brief Provides access to the underlying vertex buffer
        * \return A reference to the underlying buffer.
        */
      const VertVPtr& vertices () const { return v_; }

      /** \brief Returns the number of vertices stored */
      size_t sizeVertices () const
      {
        assert (!(v_->size () % 3));
        return v_->size () / size_t (3);
      }

    protected:

      /** \brief Vertex buffer shared ptr */
      VertVPtr v_;
  };

  /** \brief A mesh primitive which supports variable sized polygons */
  class Mesh : public MeshBase
  {
    friend std::ostream& ::operator<< (std::ostream& os, const Mesh& m);

    public:

      /** \brief Alias to a vector of indexes */
      typedef std::vector<size_t> FaceV;

      /** \brief Alias to a shared pointer of a vector of indexes */
      typedef std::shared_ptr<FaceV> FaceVPtr;

      /** \brief Alias to a shared pointer of a const vector of indexes */
      typedef std::shared_ptr<const FaceV> FaceVConstPtr;

      /** \brief Alias to a vector of floats representing
        * a the normals
        */
      typedef std::vector<float> NormV;

      /** \brief Alias to a shared pointer of the normals vector */
      typedef std::shared_ptr<NormV> NormVPtr;

      /** \brief Alias to a shared pointer of the const normals vector */
      typedef std::shared_ptr<const NormV> NormVConstPtr;

      /** \brief Default ctor
        *
        * Initializes all the shared pointers with empty buffers
        */
      Mesh ()
        : f_ (std::make_shared<FaceV> ())
        , n_ (std::make_shared<NormV> ())
      {}

      /** \brief Size initialization ctor
        *
        * Allocates new buffers to accommodate for the specified number
        * of vertices, faces and normals
        * \param[in] n_v - desired number of vertices
        * \param[in] n_f - desired size of the face buffer
        * \param[in] n_n - desired number of normals
        */
      Mesh (const size_t n_v,
            const size_t n_f,
            const size_t n_n)
        : MeshBase (n_v)
        , f_ (std::make_shared<FaceV> (n_f))
        , n_ (std::make_shared<NormV> (3 * n_n))
      {}

      /** \brief Buffer copy ctor
        * \param[in] v - the vector buffer to be copied
        * \param[in] f - the face buffer to be copied
        * \param[in] n - the normal buffer to be copied
        */
      Mesh (const VertV& v,
            const FaceV& f,
            const NormV& n)
        : MeshBase (v)
        , f_ (std::make_shared<FaceV> (f))
        , n_ (std::make_shared<NormV> (n))
      {}

      /** \brief Shared ownership ctor
        * \param[in] v - the shared pointer to the vertex buffer
        * \param[in] f - the shared pointer to the face buffer
        * \param[in] n - the shared pointer to the normal buffer
        */
      Mesh (const VertVPtr& v,
            const FaceVPtr& f,
            const NormVPtr& n)
        : MeshBase (v)
        , f_ (f)
        , n_ (n)
      {}

      /** \brief Performs a deep object copy
        * \return A new Mesh object with a deep copy of all buffers
        */
      Mesh clone () const
      {
        return Mesh (*v_, *f_, *n_);
      }

      /** \brief Returns the vertex index specified by the face index
        * \param[in] idx - index of the face buffer
        * \return The vertex index specified by the face buffer
        */
      size_t face (const size_t idx) const { return (*f_)[idx]; }

      /** \brief Returns a reference to an element of the face buffer
        * \param[in] idx - index of the face buffer
        * \return A reference to an element of the face buffer
        */
      size_t& face (const size_t idx) { return (*f_)[idx]; }

      /** \brief Provides access to the underlying face buffer
        * \return A reference to the underlying face buffer.
        */
      const FaceVPtr& faces () const { return f_; }

      /** \brief Returns an Eigen::Map to the normal requested
        * \param[in] idx - index of the normal requested
        * \return A new const Eigen::Map pointing to the normal requested
        */
      Eigen::Map<const Vector3f> normal (const size_t idx) const
      {
        return Eigen::Map<const Vector3f> (n_->data () + 3*idx);
      }

      /** \brief Returns an Eigen::Map to the normal requested
        * \param[in] idx - index of the normal requested
        * \return A new Eigen::Map pointing to the normal requested
        */
      Eigen::Map<Vector3f> normal (const size_t idx)
      {
        return Eigen::Map<Vector3f> (n_->data () + 3*idx);
      }

      /** \brief Provides access to the underlying normal buffer
        * \return A reference to the underlying normal buffer.
        */
      const NormVPtr& normals () const { return n_; }

      /** \brief Returns the size of the face buffer */
      size_t sizeFaces () const
      {
        return f_->size ();
      }

      /** \brief Returns the number of normals stored */
      size_t sizeNormals () const
      {
        assert (!(n_->size () % 3));
        return n_->size () / size_t (3);
      }

    protected:

      /** \brief Shared pointer to the face buffer */
      FaceVPtr f_;

      /** \brief Shared pointer to the normal buffer */
      NormVPtr n_;
  };

  /** \brief A triangular mesh primitive */
  class TriMesh : public Mesh
  {
    friend std::ostream& ::operator<< (std::ostream& os, const TriMesh& m);

    public:
      //////////////////////////
      //       Methods
      //////////////////////////

      /** \brief Default ctor
        *
        * Initializes empty buffers
        */
      TriMesh () = default;

      /** \brief Size initialization ctor
        *
        * Allocates new buffers to accommodate for the specified number
        * of vertices, faces and normals
        * \param[in] n_v - desired number of vertices
        * \param[in] n_f - desired number of faces
        * \param[in] n_n - desired number of normals
        */
      TriMesh ( const size_t n_v,
                const size_t n_f,
                const size_t n_n)
        : Mesh (n_v, 3 * n_f, n_n)
      {}

      /** \brief Returns an Eigen::Map to the face requested
        * \param[in] idx - index of the face requested
        * \return A new const Eigen::Map pointing to the face requested
        */
      Eigen::Map<const Vector3<size_t>> face (const size_t idx) const
      {
        return Eigen::Map<const Vector3<size_t>> (f_->data () + 3*idx);
      }

      /** \brief Returns an Eigen::Map to the face requested
        * \param[in] idx - index of the face requested
        * \return A const Eigen::Map pointing to the face requested
        */
      Eigen::Map<Vector3<size_t>> face (const size_t idx)
      {
        return Eigen::Map<Vector3<size_t>> (f_->data () + 3*idx);
      }

      /** \brief Returns the face normal provided its index
        * \param[in] idx - the face index
        * \return Returns a normalized vector with the normal
        */
      Vector3f faceNormal (const size_t idx) const;

      /** \brief Returns the number of faces stored */
      size_t sizeFaces () const
      {
        assert (!(f_->size () % 3));
        return f_->size () / size_t (3);
      }
  };

  /** \brief A edge mesh (wireframe) primitive */
  class EdgeMesh : public MeshBase
  {
    friend std::ostream& ::operator<< (std::ostream& os, const EdgeMesh& m);

    public:

      /** \brief Alias to a vector of indexes representing
        * the edges
        */
      typedef std::vector<size_t> EdgeV;

      /** \brief Alias to a shared pointer of the edges vector */
      typedef std::shared_ptr<EdgeV> EdgeVPtr;

      /** \brief Alias to a shared pointer of the const edges vector */
      typedef std::shared_ptr<const EdgeV> EdgeVConstPtr;

      //////////////////////////
      //       Methods
      //////////////////////////

      /** \brief Default construtor
        *
        * Initializes empty buffers
        */
      EdgeMesh ()
        : e_ (std::make_shared<EdgeV> ())
      {}

      /** \brief Constructor from TriMesh
        * \param[in] tri - a triangular mesh from which the EdgeMesh
        * is generated
        * \param[in] filter_angle - the abs of the angle cosine.
        * Any cosine above this value will be filtered. Accepted
        * range ]0,1];
        */
      EdgeMesh (const TriMesh& tri, const float filter_angle = 1.f);

      /** \brief Assignment from TriMesh
        * \param[in] tri - a triangular mesh from which the EdgeMEsh
        * is generated
        * \return Returns itself (EdgeMesh).
        */
      EdgeMesh& operator= (const TriMesh& tri);

      /** \brief Returns an Eigen::Map to the edges requested
        * \param[in] idx - index of the edge requested
        * \return A new const Eigen::Map pointing to the edge requested
        */
      Eigen::Map<const Vector2<size_t>> edge (const size_t idx) const
      {
        return Eigen::Map<const Vector2<size_t>> (e_->data () + 2*idx);
      }

      /** \brief Returns an Eigen::Map to the edges requested
        * \param[in] idx - index of the edge requested
        * \return A new Eigen::Map pointing to the edge requested
        */
      Eigen::Map<Vector2<size_t>> edge (const size_t idx)
      {
        return Eigen::Map<Vector2<size_t>> (e_->data () + 2*idx);
      }

      /** \brief Provides access to the underlying edge buffer
        * \return A reference to the underlying edge buffer.
        */
      const EdgeVPtr& edges () const { return e_; }

      /** \brief Populates an EdgeMesh from a filtered TriMesh
        * \param[in] tri - a triangular mesh from which the EdgeMEsh
        * is generated
        * \param[in] filter_angle - the abs of the angle cosine.
        * Any cosine above this value will be filtered. Accepted
        * range ]0,1];
        * \return Returns itself (EdgeMesh).
        */
      EdgeMesh& filter (const TriMesh& tri, const float filter_angle);

      /** \brief Returns the number of edges stored */
      size_t sizeEdges () const
      {
        assert (!(e_->size () % 2));
        return e_->size () / size_t (2);
      }

    protected:
      //////////////////////////
      //       Members
      //////////////////////////

      /** \brief Shared pointer to the edges buffer */
      EdgeVPtr e_;
  };

  /** @}*/
}

/** @}*/

#endif //CVL_COMMON_MESH_H_
