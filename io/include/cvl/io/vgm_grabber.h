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
        * \param[in] size - frame number
        * \param[in] cv::Mat - an image
        * \param[in] Vector4f - rotation (angle axis) from groundtruth
        * \param[in] Vector4f - translation from groundtruth
        */
      typedef void (cb_vgm) ( const size_t,
                              const cv::Mat&,
                              const Vector4f&,
                              const Vector4f&);

      //////////////////////////////////////////////////////
      //                  Methods
      //////////////////////////////////////////////////////

      VgmGrabber (const char* const path);

      const Camera<double>& getCamera () const { return cam_; }

      bool start () override;

      void stop () override;

      /** \brief Triggers the reading of data */
      void trigger ();
      
    protected:

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

      //////////////////////////////////////////////////////
      //                  Methods
      //////////////////////////////////////////////////////

      /** \brief Called when the all callbacks are to be notified of
        *  new data
        */
      void cbVgm (const size_t id,
                  const cv::Mat& frame,
                  const Vector4f& rvec,
                  const Vector4f& tvec) const;

      size_t findStreamPos (const char* const id) const;

      static std::set<Mode> initSupportedModes ();

      static std::set<const char*> initSupportedSigs ();

      /** \brief Parse camera information */
      bool parseCamera (const std::string& path);

      /** \brief Responsible for stripping the provided path from any
        * trailling forward slashes and infer the name of the data set
        */
      void parsePathAndName (const std::string& path);

      /** \brief Methods invoked on async mode to continuously fetch
        * data from the data source
        */
      void run ();
  };
}

#endif //CVL_IO_VGM_GRABBER_H_
