/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/05/09
  * \date last modified: 2017/07/28
  */
template<typename _Float> ht::Vector4<_Float>
ht::euler_2_angle_axis (const _Float rx, const _Float ry, const _Float rz)
{
  const AngleAxis<_Float> out (AngleAxis<_Float> (rx, Vector3<_Float>::UnitX ())
                                  * AngleAxis<_Float> (ry, Vector3<_Float>::UnitY ())
                                  * AngleAxis<_Float> (rz, Vector3<_Float>::UnitZ ()));
  const Vector3<_Float>& axis = out.axis ();
  return Vector4<_Float> (out.angle (), axis[0], axis[1], axis[0]);
}
