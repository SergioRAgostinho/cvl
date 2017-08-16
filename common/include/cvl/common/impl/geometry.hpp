/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/05/09
  * \date last modified: 2017/07/28
  */

/////////////////////////////////////////////
//              AxisAngle
/////////////////////////////////////////////

template<typename _Float> ht::AxisAngle<_Float>
operator* ( const ht::AxisAngle<_Float>& lhs,
            const ht::AxisAngle<_Float>& rhs)
{
  using Eigen::AngleAxis;
  const AngleAxis<_Float> out
  (
    AngleAxis<_Float> (lhs.angle (), lhs.axis ())
      * AngleAxis<_Float> (rhs.angle (), rhs.axis ())
  );
  return ht::AxisAngle<_Float> (out);
}

template<typename _Float> ht::Pose<_Float>
operator* ( const ht::Pose<_Float>& lhs,
            const ht::Pose<_Float>& rhs)
{
  return ht::Pose<_Float> ( lhs.rotation () * rhs.rotation (),
                            lhs.rotation () * rhs.translation ()
                              + lhs.translation ());
}

/////////////////////////////////////////////
//           GP Functions
/////////////////////////////////////////////


template<typename _Derived>void
ht::compose ( ht::AxisAngle<typename ht::MatrixBase<_Derived>::Scalar>& rout,
              ht::MatrixBase<_Derived>& tout,
              const ht::AxisAngle<typename MatrixBase<_Derived>::Scalar>& rvec1,
              const ht::MatrixBase<_Derived>& tvec1,
              const ht::AxisAngle<typename MatrixBase<_Derived>::Scalar>& rvec2,
              const ht::MatrixBase<_Derived>& tvec2)
{
  rout = rvec1 * rvec2;
  tout = rvec1 * tvec2 + tvec1;
}

template<typename _DerivedO, typename _DerivedI> void
ht::rigid ( ht::MatrixBase<_DerivedO>& rvec,
            ht::MatrixBase<_DerivedO>& tvec,
            const ht::MatrixBase<_DerivedI>& pts_b,
            const ht::MatrixBase<_DerivedI>& pts_a)
{
  using ScalarI = typename MatrixBase<_DerivedI>::Scalar;
  using ScalarO = typename MatrixBase<_DerivedO>::Scalar;
  static_assert (std::is_same<ScalarI, ScalarO>::value, "Scalar types must be similar");

  // Extract centroids
  const RowVector3<ScalarI> c_a = pts_a.colwise ().mean ();
  const RowVector3<ScalarI> c_b = pts_b.colwise ().mean ();

  // Normalize data
  const Matrix<ScalarI, Dynamic, 3, ColMajor> pts_a_n = pts_a.rowwise () - c_a;
  const Matrix<ScalarI, Dynamic, 3, ColMajor> pts_b_n = pts_b.rowwise () - c_b;

  // SVD
  Eigen::JacobiSVD<Matrix3<ScalarI>> svd (pts_b_n.transpose () * pts_a_n,
                                          Eigen::ComputeFullU | Eigen::ComputeFullV);
  Matrix3<ScalarI> R = svd.matrixU () * svd.matrixV ().transpose ();

  // Perform reflection if necessary
  if (R.determinant () < 0)
  {
    Matrix3<ScalarI> eye = Matrix3f::Identity ();
    eye (2,2) *= -1;
    R.noalias () = svd.matrixU () * eye * svd.matrixV ().transpose ();
  }
  const Eigen::AngleAxis<ScalarO> aa (R);
  rvec.head (3) = aa.axis ();
  rvec[3] = aa.angle ();

  // Translation
  tvec.head (3) = c_b - c_a * R.transpose ();
  tvec[3] = 0.f;
}
