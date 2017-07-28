/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/05/05
  * \date last modified: 2017/05/09
  */
#include <cvl/io/vgm_grabber.h>
#include <cvl/io/utils.h>
#include <cvl/common/geometry.h>
#include <tinyxml2.h>
#include <thread>
#include <iostream>

///////////////////////////////////////////////
//                  Public
///////////////////////////////////////////////

ht::VgmGrabber::VgmGrabber (const char* const path)
  : Grabber (initSupportedSigs (), initSupportedModes ())
{
  parsePathAndName (path);
  parseCamera (path_ + '/' + name_ + '/' + name_ + ".xcp");
  streampos_s_ = findStreamPos ("Segments");
  streampos_t_ = findStreamPos ("Trajectories");
}

bool
ht::VgmGrabber::start ()
{
  // Open video capture
  vc_.open (path_ + '/' + name_ + "/frames/%08d.jpg",  cv::CAP_IMAGES);
  if (!vc_.isOpened ())
  {
    std::cerr << "ERROR: Failed to open video capture device" << std::endl;
    return false;
  }

  // Open text file and seek till after que provided sequence
  f_s_.open (path_ + '/' + name_ + '/' + name_ + ".csv");
  if (!f_s_.is_open ())
  {
    std::cerr << "ERROR: Failed to open groudtruth file" << std::endl;
    stop ();
    return false;
  }
  f_s_.seekg (streampos_s_);

  // Open text file and seek till after que provided trajectories
  f_t_.open (path_ + '/' + name_ + '/' + name_ + ".csv");
  if (!f_t_.is_open ())
  {
    std::cerr << "ERROR: Failed to open groudtruth file" << std::endl;
    stop ();
    return false;
  }
  f_t_.seekg (streampos_t_);

  // launch async thread if mode requires
  if (mode_ == Mode::ASYNC)
    thread_ = std::thread (&VgmGrabber::run, this);

  frame_nr_ = 0;
  running_ = true;
  return true;
}

void
ht::VgmGrabber::stop ()
{
  running_ = false;
  if (thread_.joinable ())
    thread_.join();

  if (vc_.isOpened ())
    vc_.release ();

  if (f_s_.is_open ())
    f_s_.close ();

  if (f_t_.is_open ())
    f_t_.close ();
}

void
ht::VgmGrabber::trigger ()
{
  if (!running_)
    return;

  std::string str;
  if (!getline_rn (f_s_, str))
  {
    running_ = false;
    return;
  }

  // ignore data from every odd iteration
  if (frame_nr_++ % 2)
    return;

  cv::Mat frame;
  vc_ >> frame;

  // Check if data is ok
  if (frame.empty ())
  {
    running_ = false;
    return;
  }

  Vector3f euler;
  Vector4f tvec (0.f, 0.f, 0.f, 0.f);
  size_t id;
  sscanf (str.c_str (),
          "%lu,%*u,%f,%f,%f,%f,%f,%f",
          &id,
          &euler[0], &euler[1], &euler[2],
          &tvec[0], &tvec[1], &tvec[2]);


  // notify
  const Vector4f rvec = euler_2_angle_axis (euler[0], euler[1], euler[2]);
  cbVgm (id, frame.clone (), rvec, tvec);
}

///////////////////////////////////////////////
//                  Protected
///////////////////////////////////////////////

void
ht::VgmGrabber::cbVgm ( const size_t id,
                        const cv::Mat& frame,
                        const Vector4f& rvec,
                        const Vector4f& tvec) const
{
  for (const FunctionBase* const f : registered_cbs_.at (typeid (cb_vgm).name ()))
    (*static_cast<const Function<cb_vgm>*> (f)) (id, frame, rvec, tvec);
}

size_t
ht::VgmGrabber::findStreamPos (const char* const id) const
{
  std::ifstream ifs (path_ + '/' + name_ + '/' + name_ + ".csv");
  std::string line;
  while (getline_rn (ifs, line))
  {
    if (!line.compare (id))
    {
      // discard four lines
      ifs.ignore (std::numeric_limits<std::streamsize>::max (), '\n');
      ifs.ignore (std::numeric_limits<std::streamsize>::max (), '\n');
      ifs.ignore (std::numeric_limits<std::streamsize>::max (), '\n');
      ifs.ignore (std::numeric_limits<std::streamsize>::max (), '\n');
      return ifs.tellg ();
    }
  }
  return ifs.tellg ();
}

std::set<ht::Grabber::Mode>
ht::VgmGrabber::initSupportedModes ()
{
  static const std::set<Mode> s { Mode::ASYNC, Mode::TRIGGER };
  return s; //copy elision
}

std::set<const char*>
ht::VgmGrabber::initSupportedSigs ()
{
  static const char* const cb_vgm_type = typeid (cb_vgm).name ();
  static const std::set<const char*> s { cb_vgm_type };
  return s; //copy elision
}

bool
ht::VgmGrabber::parseCamera (const std::string& path)
{
  using tinyxml2::XMLDocument;
  using tinyxml2::XMLElement;

  // Load the doc
  XMLDocument doc;
  doc.LoadFile (path.c_str ());
  if (doc.ErrorID ())
  {
    std::cerr << "ERROR: Could not load XML file" << std::endl;
    return false;
  }

  // traverse cameras till we find the one we want
  const XMLElement* el = doc.FirstChildElement ("Cameras")
                              ->LastChildElement ("Camera");
  while (el && strncmp (el->Attribute ("TYPE"), "DCAM", 4))
    el = el->PreviousSiblingElement ("Camera");
  if (!el)
  {
    std::cerr << "ERROR: Could not find correct camera." << std::endl;
    return false;
  }

  // parse image size
  const char* att = el->Attribute ("SENSOR_SIZE");
  if (2 != sscanf (att, "%5u %5u", &cam_.width, &cam_.height))
  {
    std::cerr << "ERROR: Could not parse image size" << std::endl;
    return false;
  }

  // Get a hold of the keyframe element with all the useful info
  el = el->LastChildElement ("KeyFrames")->FirstChildElement ("KeyFrame");
  if (!el)
  {
    std::cerr << "ERROR: Failed to find camera info." << std::endl;
    return false;
  }

  // Parse intrinsics matrix
  cam_.k = Matrix43d::Identity ();
  cam_.k(0, 0) = cam_.k(1, 1) = el->DoubleAttribute ("FOCAL_LENGTH");
  att = el->Attribute ("PRINCIPAL_POINT");
  int n = sscanf (att, "%20lf %20lf", &cam_.k(0, 2), &cam_.k(1, 2));
  if (n < 2)
  {
    std::cerr << "ERROR: Failed to parse camera's intrinsics."
              << std::endl;
    return false;
  }

  // Parse Extrinsics - original data is Cam to W 
  Quaterniond q;
  att = el->Attribute ("ORIENTATION");
  n = sscanf (att, "%20lf %20lf %20lf %20lf", &q.x (), &q.y (), &q.z (), &q.w ());

  Vector4d t (0, 0, 0, 0);
  att = el->Attribute ("POSITION");
  n += sscanf (att, "%20lf %20lf %20lf", &t[0], &t[1], &t[2]);
  if (n < 7)
  {
    std::cerr << "ERROR: Failed to parse the camera's extrinsics"
              << std::endl;
    return false;
  }
  
  // Perform final conversion to storage 
  cam_.rvec = angle_axis (q);
  cam_.rvec[0] *= -1;
  cam_.tvec = -rotate4 (cam_.rvec, t);
  return true;
}

void
ht::VgmGrabber::parsePathAndName (const std::string& path)
{
  size_t upper = path.size ();
  size_t lower = path.find_last_of ("/\\");

  while (!path.substr (lower, upper - lower - 1).size ())
  {
    upper = lower;
    lower = path.find_last_of ("/\\", lower - 1);
  }

  // Store result in class
  path_ = path.substr (0, lower);
  name_ = path.substr (lower + 1, upper - lower - 1);
}

void
ht::VgmGrabber::run ()
{
  while (running_)
    trigger ();
}