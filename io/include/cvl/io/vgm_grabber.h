/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/05/05
  * \date last modified: 2017/05/05
  */
#pragma once
#ifndef CVL_IO_VGM_GRABBER_H_
#define CVL_IO_VGM_GRABBER_H_

#include <cvl/io/grabber.h>
#include <cvl/common/camera.h>
#include <cvl/common/eigen.h>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <fstream>
#include <string>

namespace ht
{
  class VgmGrabber : public Grabber
  {
    public:

      //////////////////////////////////////////////////////
      //                  Types
      //////////////////////////////////////////////////////

      /** \brief Signature for the callback function of the vgm data set
        * \param[in] size_t - frame number
        * \param[in] cv::Mat - an image
        * \param[in] Vector4f - rotation (angle axis) from groundtruth
        * \param[in] Vector4f - translation from groundtruth
        */
      typedef void (cb_vgm_img_motion) (const size_t,
                                        const cv::Mat&,
                                        const Vector4f&,
                                        const Vector4f&);

      /** \brief Callback signature which returns the reference points
        * trajectories in world coordinates
        * \param[in] size_t - the frame number
        * \param[in] Matrix<float, 3, Dynamic, ColMajor>& - a matrix with the
        * the reference points coordinates
        */
      typedef void (cb_vgm_ref_points) (const size_t,
                                        const Matrix<float, 3, Dynamic, ColMajor>&);

      //////////////////////////////////////////////////////
      //                  Methods
      //////////////////////////////////////////////////////

      VgmGrabber (const char* const path);

      const Camera<double>& getCamera () const { return cam_; }

      const Matrix<float, 3, Dynamic, ColMajor>&
      getReferencePoints () const { return refs_; }

      bool start () override;

      void stop () override;

      /** \brief Triggers the reading of data */
      void trigger ();
      
    protected:

      //////////////////////////////////////////////////////
      //                  Types
      //////////////////////////////////////////////////////

      enum CbFlags
      {
        HAS_IMAGE = 0b1u,
        HAS_SEGMENTS = 0b10u,
        HAS_TRAJECTORIES = 0b100u
      };

      //////////////////////////////////////////////////////
      //                  Members
      //////////////////////////////////////////////////////

      /** \brief Store the root folder path for the data set
        * \note Assumes the data set naming conventions were follwed
        */
      std::string path_;

      /** \brief Name of the data set
        * \note Assumes the data set naming conventions were followed
        */
      std::string name_;

      /** \brief Camera information extracted from the dataset
        *
        * \note The camera pose corresponds to the transformation from
        * the world coordinates to camera coordinates. Suitable to 
        * perform visualization
        */
      Camera<double> cam_;

      /** \brief The image capuring device */
      cv::VideoCapture vc_;

      /** \brief The file handler responsible for reading the segments
        * section of the grountruth data */
      std::ifstream f_s_;

      /** \brief The file handler responsible for reading the trajectories
        * section of the grountruth data */
      std::ifstream f_t_;

      /** \brief Stores the stream position to start reading
        * the segments from
        */
      size_t streampos_s_;

      /** \brief Stores the stream position to start reading
        * the trajecories from.
        */
      size_t streampos_t_;

      /** \brief Stores the reference points coords. It's a dynamic
        * size matrix so it doesn't require the Eigen new operator
        * overload
        */
      Matrix<float, 3, Dynamic, ColMajor> refs_;

      /** \brief Callback bit flag used to determine which types
        * of data fetching are required
        */
      uint8_t cb_flags_;

      //////////////////////////////////////////////////////
      //                  Methods
      //////////////////////////////////////////////////////

      /** \brief Called when the all callbacks are to be notified of
        *  new data
        * \param[in] id - the data frame number
        * \param[in] frame - an image frame
        * \param[in] rvec - angleaxis rotation of the object in the
        * VICON reference frame
        * \param[in] tvec - translation of the obj in the VICON
        * reference frame.
        */
      void cbVgmImgMotion ( const size_t id,
                            const cv::Mat& frame,
                            const Vector4f& rvec,
                            const Vector4f& tvec) const;

      /** \brief A callback returning the reference points trajectory
        *  on each frame
        * \param[in] id - the data frame serial identifier
        * \param[in] pts - the reference points trajectory coordinates
        * in the VICON reference frame.
        */
      void cbVgmRefPoints ( const size_t id,
                            const Matrix<float, 3, Dynamic, ColMajor>& pts) const;

      size_t findStreamPos (const char* const id) const;

      /** \brief Initializes the callbacks flags based on the
        * registered signatures
        */
      void initCbFlags ();

      static std::set<Mode> initSupportedModes ();

      static std::set<const char*> initSupportedSigs ();

      /** \brief Parse camera information */
      bool parseCamera (const std::string& path);

      /** \brief Parses the provided configuration file. */
      void parseConfigurationFile ();

      /** \brief Responsible for stripping the provided path from any
        * trailling forward slashes and infer the name of the data set
        */
      void parsePathAndName (const std::string& path);

      /** \brief Methods invoked on async mode to continuously fetch
        * data from the data source
        */
      void run ();

    private:

      /** \brief Extracts the sequence origin from a string
        * \param[in] data - the data string
        */
      void parseOriginPoint (const std::string& data);

      /** \brief Extracts the reference points from a string
        * \param[in] data - the data string
        */
      void parseReferencePoints (const std::string& data);


  };
}

#endif //CVL_IO_VGM_GRABBER_H_
