/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/05/05
  * \date last modified: 2017/05/05
  */
#include <cvl/io/vgm_grabber.h>
#include <iostream>
#include <string>

using namespace ht;

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
  return 0;
}