/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/07/27
  * \date last modified: 2017/07/27
  */
#include <cvl/common/mesh.h>

std::ostream&
operator<< (std::ostream& os, const ht::Mesh& m)
{
  // Vertices
	const size_t s_v = m.v.size () / 3;
	os << "vert: (" << s_v << ")\n";
	for (size_t i = 0; i < s_v; ++i)
		os << ' ' << m.v[3*i] << ' ' << m.v[3*i + 1] << ' ' << m.v[3*i + 2] << '\n';

  // Faces
  const size_t s_f = m.f.size ();
  os << "faces: (" << s_f << ")\n";
  for (size_t i = 0; i < s_f; ++i)
    os << ' ' << m.f[i] << '\n';

  // Normals
  const size_t s_n = m.n.size () / 3;
  os << "normals: (" << s_n << ")\n";
  for (size_t i = 0; i < s_n; ++i)
    os << ' ' << m.n[3*i] << ' ' << m.n[3*i + 1] << ' ' << m.n[3*i + 2] << '\n';
	return os;
}

std::ostream&
operator<< (std::ostream& os, const ht::TriMesh& m)
{
  // Vertices
  const size_t s_v = m.v.size () / 3;
  os << "vert: (" << s_v << ")\n";
  for (size_t i = 0; i < s_v; ++i)
    os << ' ' << m.v[3*i] << ' ' << m.v[3*i + 1] << ' ' << m.v[3*i + 2] << '\n';

  // Faces
  const size_t s_f = m.f.size () / 3;
  os << "faces: (" << s_f << ")\n";
  for (size_t i = 0; i < s_f; ++i)
    os << ' ' << m.f[3*i] << ' ' << m.f[3*i + 1] << ' ' << m.f[3*i + 2] << '\n';

  // Normals
  const size_t s_n = m.n.size () / 3;
  os << "normals: (" << s_n << ")\n";
  for (size_t i = 0; i < s_n; ++i)
    os << ' ' << m.n[3*i] << ' ' << m.n[3*i + 1] << ' ' << m.n[3*i + 2] << '\n';
  return os;
}
