/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/05/04
  * \date last modified: 2017/05/04
  */
#include <cvl/io/grabber.h>

//////////////////////////////////////////////////////
//                      Public
//////////////////////////////////////////////////////

bool
ht::Grabber::setMode (const ht::Grabber::Mode mode)
{
  // do not allow changing the mode when grabber is running
  if (running_)
    return false;

  const auto it = supported_modes_.find (mode);
  if (it == supported_modes_.cend ())
    return false;

  mode_ = mode;
  return true;
}

void
ht::Grabber::unregisterAllCbs ()
{
  for (auto& sig : registered_cbs_)
    sig.second.clear ();
}