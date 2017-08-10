/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/05/05
  * \date last modified: 2017/08/07
  */
#include <cvl/visualization/opencv.h>
#include <cvl/io/vgm_grabber.h>
#include <cvl/io/vgm_model_reader.h>
#include <cvl/common/mesh.h>
#include <cvl/common/geometry.h>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>

using ht::VgmGrabber;
using ht::Matrix3f;
using ht::Vector4f;
using ht::Camera;

static cv::Mat frame_;
static Vector4f rvec_;
static Vector4f tvec_;

void
cb_img_motion ( size_t,
                const cv::Mat& img,
                const Vector4f& rvec,
                const Vector4f& tvec)
{
  frame_ = img;
  rvec_ = rvec;
  tvec_ = tvec;
}

std::string
extract_model_path (const std::string& path)
{
  size_t upper = path.size ();
  size_t lower = path.find_last_of ("/\\");

  while (!path.substr (lower, upper - lower - 1).size ())
  {
    upper = lower;
    lower = path.find_last_of ("/\\", lower - 1);
  }

  // Store result in class
  const std::string dir = path.substr (0, lower);
  const std::string name = path.substr (lower + 1, upper - lower - 5);
  return dir + "/../model/" + name + ".mat";
}

bool
load_model (ht::EdgeMesh& edge, const char* const path)
{
  std::string model_path = extract_model_path (path);
  if (model_path.empty ())
  {
    std::cerr << "Could not parse path to model file." << std::endl;
    return false;
  }

  ht::TriMesh mesh;
  std::clog << "Loading model from: " << model_path << std::endl;
  if (!ht::vgm_model_reader (mesh, model_path.c_str ()))
  {
    std::cerr << "Failed to load the model." << std::endl;
    return false;
  }

  edge.filter (mesh, .8f);
  std::cout << "Model Info:\n" << edge << std::endl;
  return true;
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

  // Load the 3d Model
  ht::EdgeMesh mesh;
  if (!load_model (mesh, argv[1]))
    return -1;

  // Create the grabber and start things
  VgmGrabber grabber (argv[1]);
  const Camera<double>& cam = grabber.getCamera ();

  std::cout << "K:\n" << cam.k
            << "\nrvec: " << cam.rvec.transpose ()
            << "\ntvec: " << cam.tvec.transpose ()
            << "\nwidth: " << cam.width
            << " height: " << cam.height << std::endl;

  std::cout << "Reference points:\n"
            << grabber.getReferencePoints ()
            << std::endl;

  // Register callbacks
  const std::function<VgmGrabber::cb_vgm_img_motion> f_img_motion = std::bind ( cb_img_motion,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2,
                                                                                std::placeholders::_3,
                                                                                std::placeholders::_4);
  if (!grabber.registerCb (f_img_motion))
  {
    std::cerr << "Failed to register the image motion callback" << std::endl;
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
  cv::namedWindow ( window, cv::WINDOW_NORMAL
                              | cv::WINDOW_KEEPRATIO
                              | cv::WINDOW_GUI_NORMAL);
  while (true)
  {
    grabber.trigger ();
    if (!grabber.isRunning ())
      break;

    // std::cout << "rvec: " << rvec_.transpose ()
    //           << "\ntvec: " << tvec_.transpose () << std::endl;
    ht::draw (frame_, mesh, cam.k.topRows<3> ().cast<float> (), rvec_, tvec_);

    cv::imshow (window, frame_);
    const int key = cv::waitKey (1);
    if (key == 27) //esc
      break;
  }
  return 0;
}
