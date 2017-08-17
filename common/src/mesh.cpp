/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/07/27
  * \date last modified: 2017/08/07
  */
#include <cvl/common/mesh.h>
#include <set>
#include <map>
#include <cassert>

////////////////////////////////////
//            TriMesh
////////////////////////////////////

ht::Vector3f
ht::TriMesh::faceNormal (const size_t idx) const
{
  const Eigen::Map<const Vector3f> p0 (&(*v_)[3*(*f_)[idx]]);
  const Eigen::Map<const Vector3f> p1 (&(*v_)[3*(*f_)[idx + 1]]);
  const Eigen::Map<const Vector3f> p2 (&(*v_)[3*(*f_)[idx + 2]]);

  Vector3f normal = (p1 - p0).cross (p2 - p1);
  normal.normalize ();
  return normal;
}

////////////////////////////////////
//            EdgeMesh
////////////////////////////////////

ht::EdgeMesh::EdgeMesh (const ht::TriMesh& tri,
                        const float filter_angle)
  : MeshBase (tri.vertices ())
{
  const std::vector<size_t>& f = *tri.faces ();

  assert (filter_angle >= 0.f);
  assert (!(f.size () % 3));
  std::map<std::pair<size_t, size_t>,std::vector<size_t>> edges;

  // Populate the map
  for (size_t i = 0; i < f.size (); i += 3)
  {
    for (size_t j = 0; j < 3; ++j)
    {
      const size_t min = std::min (f[i + (j % 3)], f[i + ((j + 1) % 3)]);
      const size_t max = std::max (f[i + (j % 3)], f[i + ((j + 1) % 3)]);

      // Might happen due to vertex filtering
      if (min == max)
        continue;

      const std::pair<size_t, size_t> key (min, max);
      std::vector<size_t>& faces = edges[key];
      faces.push_back (i);

      // Check if needs to filtered
      if (filter_angle == 1.f || faces.size () < 2)
        continue;

      //Filter
      const Vector3f v0 = tri.faceNormal (faces[0]);
      const Vector3f v1 = tri.faceNormal (faces[1]);
      if (std::abs (v0.dot (v1)) > filter_angle)
        edges.erase (key);
    }
  }

  // resize
  e_ = std::make_shared<EdgeV> (edges.size () * 2);
  size_t i = 0;
  for (const auto& edge : edges)
  {
    (*e_)[i++] = edge.first.first;
    (*e_)[i++] = edge.first.second;
  }
}

ht::EdgeMesh&
ht::EdgeMesh::operator= (const ht::TriMesh& tri)
{
  EdgeMesh m (tri);
  std::swap (*this, m);
  return *this;
}

ht::EdgeMesh&
ht::EdgeMesh::filter (const ht::TriMesh& tri,
                      const float filter_angle)
{
  EdgeMesh m (tri, filter_angle);
  std::swap (*this, m);
  return *this;
}

std::ostream&
operator<< (std::ostream& os, const ht::MeshBase& m)
{
  // Vertices
  const size_t s_v = m.v_->size () / 3;
  os << "vert: (" << s_v << ")\n";
  for (size_t i = 0; i < s_v; ++i)
    os << ' ' << (*m.v_)[3*i] << ' ' << (*m.v_)[3*i + 1] << ' ' << (*m.v_)[3*i + 2] << '\n';
  return os;
}

std::ostream&
operator<< (std::ostream& os, const ht::Mesh& m)
{
  // Vertices
  os << *static_cast<const ht::MeshBase*> (&m);

  // Faces
  const size_t s_f = m.f_->size ();
  os << "faces: (" << s_f << ")\n";
  for (size_t i = 0; i < s_f; ++i)
    os << ' ' << (*m.f_)[i] << '\n';

  // Normals
  const size_t s_n = m.n_->size () / 3;
  os << "normals: (" << s_n << ")\n";
  for (size_t i = 0; i < s_n; ++i)
    os << ' ' << (*m.n_)[3*i] << ' ' << (*m.n_)[3*i + 1] << ' ' << (*m.n_)[3*i + 2] << '\n';
	return os;
}

std::ostream&
operator<< (std::ostream& os, const ht::TriMesh& m)
{
  // Vertices
  os << *static_cast<const ht::MeshBase*> (&m);

  // Faces
  const size_t s_f = m.f_->size () / 3;
  os << "faces: (" << s_f << ")\n";
  for (size_t i = 0; i < s_f; ++i)
    os << ' ' << (*m.f_)[3*i] << ' ' << (*m.f_)[3*i + 1] << ' ' << (*m.f_)[3*i + 2] << '\n';

  // Normals
  const size_t s_n = m.n_->size () / 3;
  os << "normals: (" << s_n << ")\n";
  for (size_t i = 0; i < s_n; ++i)
    os << ' ' << (*m.n_)[3*i] << ' ' << (*m.n_)[3*i + 1] << ' ' << (*m.n_)[3*i + 2] << '\n';
  return os;
}

std::ostream&
operator<< (std::ostream& os, const ht::EdgeMesh& m)
{
  // Vertices
  os << *static_cast<const ht::MeshBase*> (&m);

  // Edges
  const size_t s_e = m.e_->size () / 2;
  os << "edges: (" << s_e << ")\n";
  for (size_t i = 0; i < s_e; ++i)
    os << ' ' << (*m.e_)[2*i] << ' ' << (*m.e_)[2*i + 1] << '\n';
  return os;
}
