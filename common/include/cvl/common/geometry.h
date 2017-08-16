/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/05/09
  * \date last modified: 2017/08/01
  */
#pragma once
#ifndef CVL_COMMON_GEOMETRY_H_
#define CVL_COMMON_GEOMETRY_H_

#include <cvl/common/eigen.h>

namespace ht
{
  /** Abstract an angle axis rotation */
  template<typename _Float>
  class AxisAngle
  {
    public:

      AxisAngle ()
        : data_ {0, 0, 1, 0}
      {}

      AxisAngle ( const _Float vx,
                  const _Float vy,
                  const _Float vz,
                  const _Float angle)
        : data_ {vx, vy, vz, angle}
      {}

      /** \brief Constructs an AxisAngle from Euler angles
        *
        * \note The rotation resultant rotation has the following order
        * R = Rx * Ry * Rz
        * \param[in] rx - rotation angle along x (in radians)
        * \param[in] ry - rotation angle along y (in radians)
        * \param[in] rz - rotation angle along z (in radians)
        */
      AxisAngle ( const _Float rx,
                  const _Float ry,
                  const _Float rz)
      {
        using Eigen::AngleAxis;
        const AngleAxis<_Float> aa (AngleAxis<_Float> (rx, Vector3<_Float>::UnitX ())
                                      * AngleAxis<_Float> (ry, Vector3<_Float>::UnitY ())
                                      * AngleAxis<_Float> (rz, Vector3<_Float>::UnitZ ()));
        axis () = aa.axis ();
        angle () = aa.angle ();
      }

      AxisAngle (const _Float (&data)[4])
        : data_ (data)
      {}

      template<typename _Derived>
      AxisAngle (const DenseBase<_Derived>& vec)
      {
        EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE (Vector4<_Float>, DenseBase<_Derived>);
        Eigen::Map<Vector4<_Float>> m (data_);
        m = vec;
      }

      AxisAngle (const Eigen::AngleAxis<_Float>& aa)
      {
        data_[3] = aa.angle ();
        axis () = aa.axis ();
      }

      template<typename _Derived>
      AxisAngle (const QuaternionBase<_Derived>& quat)
      {
        using Scalar = typename QuaternionBase<_Derived>::Scalar;
        static_assert ( std::is_same<_Float, Scalar>::value,
          "Scalar types must be similar");
        const Eigen::AngleAxis<Scalar> aa(quat);
        axis () = aa.axis ();
        angle () = aa.angle ();
      }

      template<typename _Derived> Vector4<typename MatrixBase<_Derived>::Scalar>
      operator* (const MatrixBase<_Derived>& vec) const
      {
        using Scalar = typename MatrixBase<_Derived>::Scalar;
        static_assert ( std::is_same<_Float, Scalar>::value,
                        "Scalar types must be similar");
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(MatrixBase<_Derived>, 4);
        return Eigen::Transform<Scalar, 3, Eigen::Affine, Eigen::RowMajor> (
              Eigen::AngleAxis<Scalar> (angle (), axis ()).toRotationMatrix ()
            ) * vec;
      }

      _Float& angle () { return data_[3]; }

      _Float angle () const { return data_[3]; }

      Eigen::Map<Vector3<_Float>>
      axis () { return Eigen::Map<Vector3<_Float>> (data_); }

      Eigen::Map<const Vector3<_Float>>
      axis () const { return Eigen::Map<const Vector3<_Float>> (data_); }

      Eigen::Map<Vector4<_Float>>
      vector () { return Eigen::Map<Vector4<_Float>> (data_); }

      Eigen::Map<const Vector4<_Float>>
      vector () const { return Eigen::Map<const Vector4<_Float>> (data_); }

    protected:

      /** \brief holds the actual values */
      _Float data_[4];
  };

  using AxisAnglef = AxisAngle<float>;
  using AxisAngled = AxisAngle<double>;


  template<typename _Float>
  class Pose
  {
    public:

      Pose ()
        : rvec_ ()
        , tvec_ {0, 0, 0, 0}
      {}

      Pose (const _Float (&rvec)[4],
            const _Float (&tvec)[4])
        : rvec_ (rvec)
        , tvec_ (tvec)
      {}

      template<typename _Derived>
      Pose (const DenseBase<_Derived>& rvec,
            const DenseBase<_Derived>& tvec)
        : rvec_ (rvec)
      {
        using Scalar = typename DenseBase<_Derived>::Scalar;
        static_assert ( std::is_same<_Float, Scalar>::value, "Scalar types must be similar");
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DenseBase<_Derived>, 4);

        translation () = tvec;
      }

      template<typename _Derived>
      Pose (const AxisAngle<_Float>& rvec,
            const DenseBase<_Derived>& tvec)
        : rvec_ (rvec)
      {
        using Scalar = typename DenseBase<_Derived>::Scalar;
        static_assert ( std::is_same<_Float, Scalar>::value, "Scalar types must be similar");
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DenseBase<_Derived>, 4);

        translation () = tvec;
      }

      AxisAngle<_Float>& rotation () { return rvec_; }

      const AxisAngle<_Float>& rotation () const { return rvec_; }

      Eigen::Map<Vector4<_Float>>
      translation () { return Eigen::Map<Vector4<_Float>> (tvec_); }

      Eigen::Map<const Vector4<_Float>>
      translation () const { return Eigen::Map<const Vector4<_Float>> (tvec_); }

    protected:

      AxisAngle<_Float> rvec_;

      _Float tvec_[4];
  };

  using Posef = Pose<float>;
  using Posed = Pose<double>;

  /** \brief Compose to transformations specified by their rvec and
    * tvec vectors
    */
  template<typename _Derived>
  void compose (AxisAngle<typename MatrixBase<_Derived>::Scalar>& rout,
                MatrixBase<_Derived>& tout,
                const AxisAngle<typename MatrixBase<_Derived>::Scalar>& rvec1,
                const MatrixBase<_Derived>& tvec1,
                const AxisAngle<typename MatrixBase<_Derived>::Scalar>& rvec2,
                const MatrixBase<_Derived>& tvec2);

  template<typename _Derived>
  void compose (MatrixBase<_Derived>& rout,
                MatrixBase<_Derived>& tout,
                const MatrixBase<_Derived>& rvec1,
                const MatrixBase<_Derived>& tvec1,
                const MatrixBase<_Derived>& rvec2,
                const MatrixBase<_Derived>& tvec2)
  {
    AxisAngle<typename MatrixBase<_Derived>::Scalar> aa_out;
    compose (aa_out, tout, rvec1, tvec1, rvec2, tvec2);
    rout = aa_out.vector ();
  }

  /** \brief Estimate the regid transformation from set of points A to B
    * \param[out] rvec - angleaxis vector with the rotation between the two
    * point sets
    * \param[out] tvec - translation vector with between the two point sets
    * \param[in] pts_b - set of points B, size Nx3
    * \param[in] pts_a - set of points A, size Nx3
    */
  template<typename _DerivedO, typename _DerivedI>
  void rigid (MatrixBase<_DerivedO>& rvec,
              MatrixBase<_DerivedO>& tvec,
              const MatrixBase<_DerivedI>& pts_b,
              const MatrixBase<_DerivedI>& pts_a);

  template<typename _Derived>
  void rigid (AxisAngle<typename MatrixBase<_Derived>::Scalar>& rvec,
              MatrixBase<_Derived>& tvec,
              const MatrixBase<_Derived>& pts_b,
              const MatrixBase<_Derived>& pts_a)
  {
    rigid (rvec.vector (), tvec, pts_b, pts_a);
  }

  template<typename _Derived> Pose<typename MatrixBase<_Derived>::Scalar>
  rigid ( const MatrixBase<_Derived>& pts_b,
          const MatrixBase<_Derived>& pts_a)
  {
    using Scalar = typename ht::MatrixBase<_Derived>::Scalar;
    Vector4<Scalar> rvec, tvec;
    rigid (rvec, tvec, pts_b, pts_a);
    return Pose<Scalar> (rvec, tvec);
  }

  /** \brief Generates a Rodrigues representation from a quaternion */
  template<typename _Derived>
  inline Vector3<typename QuaternionBase<_Derived>::Scalar>
  rodrigues (const QuaternionBase<_Derived>& q)
  {
    using Scalar = typename QuaternionBase<_Derived>::Scalar;
    const Eigen::AngleAxis<Scalar> aa(q);
    return Vector3<Scalar> (aa.angle () * aa.axis ());
  }

  /** \brief Rotate a vector */
  template<typename _Derived>
  inline Vector3<typename MatrixBase<_Derived>::Scalar>
  rotate (const MatrixBase<_Derived>& rodrigues,
          const MatrixBase<_Derived>& vec)
  {
    using Scalar = typename MatrixBase<_Derived>::Scalar;
    const Scalar norm = rodrigues.norm ();
    return Eigen::AngleAxis<Scalar> (norm, (1/norm)* rodrigues).toRotationMatrix ()
                                  * vec;
  }
}

// Operators
template<typename _Float> ht::AxisAngle<_Float>
operator* ( const ht::AxisAngle<_Float>& lhs,
            const ht::AxisAngle<_Float>& rhs);

template<typename _Float> ht::Pose<_Float>
operator* ( const ht::Pose<_Float>& lhs,
            const ht::Pose<_Float>& rhs);

#include <cvl/common/impl/geometry.hpp>

#endif //CVL_COMMON_GEOMETRY_H_
