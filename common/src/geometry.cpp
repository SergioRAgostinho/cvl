/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/08/01
  * \date last modified: 2017/08/01
  */
#include <cvl/common/geometry.h>

using namespace ht;

void
ht::compose ( Vector4f& rout,
              Vector4f& tout,
              const Vector4f& rvec1,
              const Vector4f& tvec1,
              const Vector4f& rvec2,
              const Vector4f& tvec2)
{
  const AngleAxisf out (AngleAxisf (rvec1[0], rvec1.tail<3> ())
                          * AngleAxisf (rvec2[0], rvec2.tail<3> ()));
  tout = rotate4 (rvec1, tvec2) + tvec1;
  rout[0] = out.angle ();
  rout.tail<3> () = out.axis ();
}

void
ht::rigid ( Vector4f& rvec,
            Vector4f& tvec,
            const ht::Matrix<float, ht::Dynamic, 3, ht::ColMajor>& pts_b,
            const ht::Matrix<float, ht::Dynamic, 3, ht::ColMajor>& pts_a)
{
  // Extract centroids
  const RowVector3f c_a = pts_a.colwise ().mean ();
  const RowVector3f c_b = pts_b.colwise ().mean ();

  // Normalize data
  const Matrix<float, Dynamic, 3, ColMajor> pts_a_n = pts_a.rowwise () - c_a;
  const Matrix<float, Dynamic, 3, ColMajor> pts_b_n = pts_b.rowwise () - c_b;

  // SVD
  Eigen::JacobiSVD<Matrix3f> svd (pts_b_n.transpose () * pts_a_n,
                                    Eigen::ComputeFullU | Eigen::ComputeFullV);
  Matrix3f R = svd.matrixU () * svd.matrixV ().transpose ();

  // Perform reflection if necessary
  if (R.determinant () < 0)
  {
    Matrix3f eye = Matrix3f::Identity ();
    eye (2,2) *= -1;
    R.noalias () = svd.matrixU () * eye * svd.matrixV ().transpose ();
  }
  const AngleAxisf aa (R);
  rvec[0] = aa.angle ();
  rvec.tail<3> () = aa.axis ();

  // Translation
  tvec.head<3> () = c_b - c_a * R.transpose ();
  tvec[3] = 0.f;
}