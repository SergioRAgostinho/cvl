/**
  * \author Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com
  * \date created: 2017/05/09
  * \date last modified: 2017/08/19
  * \file geometry.h
  * \brief Defines a number of useful geometric types and functions
  * like rotations and pose holding types and operations
  */
#pragma once
#ifndef CVL_COMMON_GEOMETRY_H_
#define CVL_COMMON_GEOMETRY_H_

#include <cvl/common/eigen.h>


namespace ht
{
  /** \addtogroup common
   *  @{
   */

  /** \class AxisAngle
    * \brief Abstracts an axis angle rotation
    */
  template<typename _Float>
  class AxisAngle
  {
    public:

      /** \brief Default ctor. Initialized an identity rotation */
      AxisAngle ()
        : data_ {0, 0, 1, 0}
      {}

      /** \brief Component-wise Ctor
        *
        * Allows setting all component individually
        * \note vx, vy, and vz are assumed to form a normalized
        * vector
        * \param[in] vx - x component of the axis
        * \param[in] vy - y component of the axis
        * \param[in] vz - y component of the axis
        * \param[in] angle - angle in radians
        */
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

      /** \brief Initialize the object from an array
        * \note the first three component of the vector are assumed to
        * define a normalized 3D vector, and the last and angle in radians.
        * \param[in] data - 4 component array
        */
      AxisAngle (const _Float (&data)[4])
        : data_ (data)
      {}

      /** \brief Initialize an Eigen dense object of size 4
        * \param[in] vec - an Eigen dense object
        * \note the first three component of the vector are assumed to
        * define a normalized 3D vector, and the last and angle in radians.
        */
      template<typename _Derived>
      AxisAngle (const DenseBase<_Derived>& vec)
      {
        EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE (Vector4<_Float>, DenseBase<_Derived>);
        Eigen::Map<Vector4<_Float>> m (data_);
        m = vec;
      }

      /** \brief Initialize an Eigen::AngleAxis object
        * \param[in] aa - An Eigen AngleAxis object
        */
      AxisAngle (const Eigen::AngleAxis<_Float>& aa)
      {
        data_[3] = aa.angle ();
        axis () = aa.axis ();
      }

      /** \brief Initialize an Eigen::Quaternion object
        * \param[in] quat -  an Eigen Quaternion object
        */
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

      /** \brief Implements a rotation * vector product
        * \param[in] vec - the vector to be rotated
        * \return The rotated vector
        */
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

      /** \brief Provides access to the angle component
        * \return A reference to the angle element
        */
      _Float& angle () { return data_[3]; }

      /** \brief Provides a const access to the angle component
        * \return The value of the angle element
        */
      _Float angle () const { return data_[3]; }

      /** \brief Provides an Eigen::Map to the axis vector
        * \return An Eigen::Map to the axis vector
        */
      Eigen::Map<Vector3<_Float>>
      axis () { return Eigen::Map<Vector3<_Float>> (data_); }

      /** \brief Provides a const version of an Eigen::Map to the axis vector
        * \return A const Eigen::Map to the axis vector
        */
      Eigen::Map<const Vector3<_Float>>
      axis () const { return Eigen::Map<const Vector3<_Float>> (data_); }

      /** \brief Provides an Eigen::Map to the data container
        * \return An Eigen::Map to the data container
        */
      Eigen::Map<Vector4<_Float>>
      vector () { return Eigen::Map<Vector4<_Float>> (data_); }

      /** \brief Provides a const version of an Eigen::Map to the data container
        * \return A const Eigen::Map to the data container
        */
      Eigen::Map<const Vector4<_Float>>
      vector () const { return Eigen::Map<const Vector4<_Float>> (data_); }

    protected:

      /** \brief Data container holding the actual values
        *
        * The first three elements are a normalized axis and the fourth
        * the angle in radians.
        */
      _Float data_[4];
  };

  /** \brief A floating point float instantiation of AxisAngle type */
  using AxisAnglef = AxisAngle<float>;

  /** \brief A floating point double instantiation of AxisAngle type */
  using AxisAngled = AxisAngle<double>;

  /** \class Pose
    * \brief Abstracts pose information, aggregating a rotation and
    * translation component.
    */
  template<typename _Float>
  class Pose
  {
    public:

      /** \brief Default Ctor
        *
        * Initializes to and "identity" rotation and a zero translation.
        */
      Pose ()
        : tvec_ {0, 0, 0, 0}
      {}

      /** \brief Copy array ctor
        *
        * Initializes directly the rotation and translation arrays with
        * copies.
        * \param[in] rvec - an axis angle array. The first three components of the rotation vector are assumed
        * to be the normalized vector, and the fourth the angle.
        * \param[in] tvec - the translation vector array. The fourth
        * component of the translation vector is assumed to be 0.
        */
      Pose (const _Float (&rvec)[4],
            const _Float (&tvec)[4])
        : rvec_ (rvec)
        , tvec_ (tvec)
      {}

      /** \brief Copy Eigen dense object ctor
        *
        * Initializes directly the rotation and translation arrays with
        * copies.
        * \param[in] rvec - an axis angle dense object. The first three components of the rotation vector are assumed
        * to be the normalized vector, and the fourth the angle.
        * \param[in] tvec - the translation vector dense object. The fourth
        * component of the translation vector is assumed to be 0.
        */
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

      /** \brief Copy AxisAngle and Eigen dense object ctor
        *
        * Initializes directly the rotation and translation arrays with
        * copies.
        * \param[in] rvec - an AxisAngle object.
        * \param[in] tvec - the translation vector dense object. The fourth
        * component of the translation vector is assumed to be 0.
        */
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

      /** \brief Exposes a reference to the AxisAngle rotation
        * \return A reference to the AxisAngle rotation
        */
      AxisAngle<_Float>& rotation () { return rvec_; }

      /** \brief Exposes a reference to the AxisAngle rotation
        * \return A const reference to the AxisAngle rotation
        */
      const AxisAngle<_Float>& rotation () const { return rvec_; }

      /** \brief Exposes an Eigen::Map to the translation array
        * \return An Eigen::Map to the translation array
        */
      Eigen::Map<Vector4<_Float>>
      translation () { return Eigen::Map<Vector4<_Float>> (tvec_); }

      /** \brief Exposes an Eigen::Map to the translation array
        * \return An const Eigen::Map to the translation array
        */
      Eigen::Map<const Vector4<_Float>>
      translation () const { return Eigen::Map<const Vector4<_Float>> (tvec_); }

    protected:

      /** \brief The rotation AxisAngle component */
      AxisAngle<_Float> rvec_;

      /** \brief The translation component */
      _Float tvec_[4];
  };

  /** \brief A float instantiation of the Pose class. */
  using Posef = Pose<float>;

  /** \brief A double instantiation of the Pose class. */
  using Posed = Pose<double>;

  /** \brief Compose two transformations specified by their rotation and
    * translation components.
    * \param[out] rout - an AxisAngle with the composed output rotation
    * \param[out] tout - a MatrixBase object with the composed output translation
    * \param[in] rvec1 - an AxisAngle with the left rotation component
    * \param[in] tvec1 - a MatrixBase object with the left translation vector
    * \param[in] rvec2 - an AxisAngle with the right rotation component
    * \param[in] tvec2 - a MatrixBase object with the right translation vector
    */
  template<typename _Derived>
  void compose (AxisAngle<typename MatrixBase<_Derived>::Scalar>& rout,
                MatrixBase<_Derived>& tout,
                const AxisAngle<typename MatrixBase<_Derived>::Scalar>& rvec1,
                const MatrixBase<_Derived>& tvec1,
                const AxisAngle<typename MatrixBase<_Derived>::Scalar>& rvec2,
                const MatrixBase<_Derived>& tvec2);

  /** \brief Compose two transformations specified by their rotation and
    * translation components.
    * \param[out] rout - a MatrixBase with the composed output rotation vector
    * \param[out] tout - a MatrixBase object with the composed output translation
    * \param[in] rvec1 - a MatrixBase with the left rotation component vector
    * \param[in] tvec1 - a MatrixBase object with the left translation vector
    * \param[in] rvec2 - a MatrixBase with the right rotation component vector
    * \param[in] tvec2 - a MatrixBase object with the right translation vector
    */
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

  /** \brief Estimate the rigid transformation from set of points A to B
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

  /** \brief Estimate the rigid transformation from set of points A to B
    * \param[out] rvec - AxisAngle object with the rotation between the two
    * point sets
    * \param[out] tvec - translation vector with between the two point sets
    * \param[in] pts_b - set of points B, size Nx3
    * \param[in] pts_a - set of points A, size Nx3
    */
  template<typename _Derived>
  void rigid (AxisAngle<typename MatrixBase<_Derived>::Scalar>& rvec,
              MatrixBase<_Derived>& tvec,
              const MatrixBase<_Derived>& pts_b,
              const MatrixBase<_Derived>& pts_a)
  {
    rigid (rvec.vector (), tvec, pts_b, pts_a);
  }

  /** \brief Estimate the rigid transformation from set of points A to B
    * \param[in] pts_b - set of points B, size Nx3
    * \param[in] pts_a - set of points A, size Nx3
    * \return A Pose object with the transformation from set of points A to B
    */
  template<typename _Derived> Pose<typename MatrixBase<_Derived>::Scalar>
  rigid ( const MatrixBase<_Derived>& pts_b,
          const MatrixBase<_Derived>& pts_a)
  {
    using Scalar = typename ht::MatrixBase<_Derived>::Scalar;
    Vector4<Scalar> rvec, tvec;
    rigid (rvec, tvec, pts_b, pts_a);
    return Pose<Scalar> (rvec, tvec);
  }

  /** \brief Generates a Rodrigues vector from a quaternion
    * \param[in] q - a Quaternion rotation
    * \return A Rodrigues vector
    */
  template<typename _Derived>
  inline Vector3<typename QuaternionBase<_Derived>::Scalar>
  rodrigues (const QuaternionBase<_Derived>& q)
  {
    using Scalar = typename QuaternionBase<_Derived>::Scalar;
    const Eigen::AngleAxis<Scalar> aa(q);
    return Vector3<Scalar> (aa.angle () * aa.axis ());
  }

  /** \brief Rotates a 3 element Vector provided a Rodrigues vector
    * \param[in] rodrigues - a Rodrigues rotation vector
    * \param[in] vec - the three dimensional vector to be rotated
    * \return The rotated 3 element Vector
    */
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

  /** @}*/
}

/** \addtogroup common
   *  @{
   */

// Operators
/** \brief The product between two rotations
  * \param[in] lhs - the left hand side AxisAngle object
  * \param[in] rhs - the right hand side AxisAngle object
  * \return The composed rotation AxisAngle
  */
template<typename _Float> ht::AxisAngle<_Float>
operator* ( const ht::AxisAngle<_Float>& lhs,
            const ht::AxisAngle<_Float>& rhs);

/** \brief The product between two transformations/poses
  * \param[in] lhs - the left hand side Pose object
  * \param[in] rhs - the right hand side Pose object
  * \return The composed pose/transformation Pose object
  */
template<typename _Float> ht::Pose<_Float>
operator* ( const ht::Pose<_Float>& lhs,
            const ht::Pose<_Float>& rhs);

/** @}*/

#include <cvl/common/impl/geometry.hpp>

#endif //CVL_COMMON_GEOMETRY_H_
