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
  class MeshBase
  {
    friend std::ostream& ::operator<< (std::ostream& os, const MeshBase& m);

    public:

      typedef std::vector<float> VertV;
      typedef std::shared_ptr<VertV> VertVPtr;
      typedef std::shared_ptr<const VertV> VertVConstPtr;

      MeshBase ()
        : v_ (std::make_shared<VertV> ())
      {}

      MeshBase (const size_t n_v)
        : v_ (std::make_shared<VertV> (n_v))
      {}

      MeshBase (const VertV& v)
        : v_ (std::make_shared<VertV> (v))
      {}

      MeshBase (const VertVPtr& v)
        : v_ (v)
      {}

      MeshBase clone () const
      {
        return MeshBase (*v_);
      }

      Eigen::Map<const Vector3f> vertex (const size_t idx) const
      {
        return Eigen::Map<const Vector3f> (v_->data () + 3*idx);
      }

      Eigen::Map<Vector3f> vertex (const size_t idx)
      {
        return Eigen::Map<Vector3f> (v_->data () + 3*idx);
      }

      const VertVPtr& vertices () const { return v_; }

    protected:
      /** \brief vertices */
      VertVPtr v_;
  };

  class Mesh : public MeshBase
  {
    friend std::ostream& ::operator<< (std::ostream& os, const Mesh& m);

    public:

      typedef std::vector<size_t> FaceV;
      typedef std::shared_ptr<FaceV> FaceVPtr;
      typedef std::shared_ptr<const FaceV> FaceVConstPtr;

      typedef std::vector<float> NormV;
      typedef std::shared_ptr<NormV> NormVPtr;
      typedef std::shared_ptr<const NormV> NormVConstPtr;

      Mesh ()
        : f_ (std::make_shared<FaceV> ())
        , n_ (std::make_shared<NormV> ())
      {}

      Mesh (const size_t n_v,
            const size_t n_f,
            const size_t n_n)
        : MeshBase (n_v)
        , f_ (std::make_shared<FaceV> (n_f))
        , n_ (std::make_shared<NormV> (n_n))
      {}

      Mesh (const VertV& v,
            const FaceV& f,
            const NormV& n)
        : MeshBase (v)
        , f_ (std::make_shared<FaceV> (f))
        , n_ (std::make_shared<NormV> (n))
      {}

      Mesh (const VertVPtr& v,
            const FaceVPtr& f,
            const NormVPtr& n)
        : MeshBase (v)
        , f_ (f)
        , n_ (n)
      {}

      Mesh clone () const
      {
        return Mesh (*v_, *f_, *n_);
      }

      size_t face (const size_t idx) const { return (*f_)[idx]; }

      size_t& face (const size_t idx) { return (*f_)[idx]; }

      const FaceVPtr& faces () const { return f_; }

      Eigen::Map<const Vector3f> normal (const size_t idx) const
      {
        return Eigen::Map<const Vector3f> (n_->data () + 3*idx);
      }

      Eigen::Map<Vector3f> normal (const size_t idx)
      {
        return Eigen::Map<Vector3f> (n_->data () + 3*idx);
      }

      const NormVPtr& normals () const { return n_; }

    protected:

      /** \brief faces */
      FaceVPtr f_;

      /** \brief normals */
      NormVPtr n_;
  };

  class TriMesh : public Mesh
  {
    friend std::ostream& ::operator<< (std::ostream& os, const TriMesh& m);

    public:
      //////////////////////////
      //       Methods
      //////////////////////////

      TriMesh () = default;

      TriMesh ( const size_t n_v,
                const size_t n_f,
                const size_t n_n)
        : Mesh (n_v, n_f, n_n)
      {}

      Eigen::Map<const Vector3<size_t>> face (const size_t idx) const
      {
        return Eigen::Map<const Vector3<size_t>> (f_->data () + 3*idx);
      }

      Eigen::Map<Vector3<size_t>> face (const size_t idx)
      {
        return Eigen::Map<Vector3<size_t>> (f_->data () + 3*idx);
      }

      /** \brief Return the face normal provided it's index
        * \param[in] idx - index
        * \return Returns a normalized vector with the normal
        */
      Vector3f faceNormal (const size_t idx) const;
  };

  class EdgeMesh : public MeshBase
  {
    friend std::ostream& ::operator<< (std::ostream& os, const EdgeMesh& m);

    public:

      typedef std::vector<size_t> EdgeV;
      typedef std::shared_ptr<EdgeV> EdgeVPtr;
      typedef std::shared_ptr<const EdgeV> EdgeVConstPtr;

      //////////////////////////
      //       Methods
      //////////////////////////

      /** \brief Default construtor */
      EdgeMesh ()
        : e_ (std::make_shared<EdgeV> ())
      {}

      /** \brief Constructor from TriMesh
        * \param[in] tri - a triangular mesh from which the EdgeMesh
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

      Eigen::Map<const Vector2<size_t>> edge (const size_t idx) const
      {
        return Eigen::Map<const Vector2<size_t>> (e_->data () + 2*idx);
      }

      Eigen::Map<Vector2<size_t>> edge (const size_t idx)
      {
        return Eigen::Map<Vector2<size_t>> (e_->data () + 2*idx);
      }

      const EdgeVPtr& edges () const { return e_; }

      /** \brief Populates an EdgeMesh from a filtered TriMesh
        * \param[in] tri - a triangular mesh from which the EdgeMEsh
        * is generated
        * \param[in] filter_angles - the abs of the angle cosine.
        * Any cosine above this value will be filtered. Accepted
        * range ]0,1];
        * \return Returns itself (EdgeMesh).
        */
      EdgeMesh& filter (const TriMesh& tri, const float filter_angle);

    protected:
      //////////////////////////
      //       Members
      //////////////////////////

      /** \brief edges */
      EdgeVPtr e_;
  };
}


#endif //CVL_COMMON_MESH_H_
