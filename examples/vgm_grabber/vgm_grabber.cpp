/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/05/05
  * \date last modified: 2017/05/05
  */
#include <cvl/io/vgm_grabber.h>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>

using ht::VgmGrabber;
using ht::Vector4f;
using ht::Camera;

static cv::Mat frame_;

void
cb (size_t n,
    const cv::Mat& img,
    const Vector4f& rvec,
    const Vector4f& tvec)
{
  frame_ = img;
  std::cout << "n: " << n
            << " rvec: " << rvec.transpose ()
            << " tvec: " << tvec.transpose ()
            << std::endl;
}

int
main (const int argc, const char** const argv)
{
  // check if path to root folder was provided
  if (argc < 2)
  {
    std::cerr << "Insuficient number of arguments" << std::endl;
    return -1;
  }

  VgmGrabber grabber (argv[1]);
  const Camera<double>& cam = grabber.getCamera ();

  std::cout << "K:\n" << cam.k
            << "\nrvec: " << cam.rvec.transpose ()
            << "\ntvec: " << cam.tvec.transpose ()
            << "\nwidth: " << cam.width
            << " height: " << cam.height << std::endl;

  // Register callback
  const std::function<VgmGrabber::cb_vgm> f = std::bind ( cb,
                                                          std::placeholders::_1,
                                                          std::placeholders::_2,
                                                          std::placeholders::_3,
                                                          std::placeholders::_4);
  if (!grabber.registerCb (f))
  {
    std::cerr << "Failed to register the callback" << std::endl;
    return -1;
  }

  // Set mode to trigger
  grabber.setMode (ht::Grabber::Mode::TRIGGER);
  if (!grabber.start ())
  {
    std::cerr << "Grabber failed to start." << std::endl;
    return -1;
  }

  const char* const window = "Sequence Video";
  cv::namedWindow ( window,
                    cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_NORMAL);
  while (true)
  {
    grabber.trigger ();
    if (!grabber.isRunning ())
      break;

    cv::imshow (window, frame_);
    const int key = cv::waitKey (1);
    if (key == 27) //esc
      break;
  }
  return 0;
}