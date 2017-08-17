/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/07/30
  * \date last modified: 2017/07/30
  */
#include <cvl/visualization/opencv.h>
#include <opencv2/imgproc.hpp>

void
ht::draw (cv::Mat& img,
          const EdgeMesh& mesh,
          const Matrix3f& k,
          const Vector4f& rvec,
          const Vector4f& tvec)
{
  assert (!(mesh.vertices ()->size () % 3));
  const size_t s_v = mesh.vertices ()->size () / 3;

  const Eigen::Map<const Matrix<float, 3, Dynamic, ColMajor>> mesh_p (mesh.vertices ()->data (), 3, s_v);
  const Matrix3f rot (Eigen::AngleAxisf (rvec[3], rvec.head<3> ()));

  const Array<float, 3, Dynamic, ColMajor> pts = k * ((rot * mesh_p).colwise () + tvec.head<3> ());
  const Array<int, 2, Dynamic, ColMajor> uvs = (pts.topRows<2> ().rowwise () * (1.f / pts.bottomRows<1> ())).cast<int> ();

  // iterate over all lines and draw them
  assert (!(mesh.edges()->size () % 2));
  for (size_t i = 0; i < mesh.edges ()->size ();)
  {
    const cv::Point* const pt1 = reinterpret_cast<const cv::Point*> (uvs.data () + 2 * (*mesh.edges ())[i++]);
    const cv::Point* const pt2 = reinterpret_cast<const cv::Point*> (uvs.data () + 2 * (*mesh.edges ())[i++]);
    cv::line (img, *pt1, *pt2, cv::Scalar (255, 255, 255), 1, cv::LINE_AA);
  }
}
