/**
  * \author Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com
  * \date created: 2017/05/04
  * \date last modified: 2017/05/04
  */

//////////////////////////////////////////////////////////////
//                      Public
//////////////////////////////////////////////////////////////

template<typename _Sig> bool
ht::Grabber::isSupported ()
{
  return supported_sigs_.find (typeid (_Sig).name ()) != supported_sigs_.end();
}

template<typename _Sig> bool
ht::Grabber::registerCb (const std::function<_Sig>& f)
{
  // check if signature is supported
  if (!isSupported<_Sig> ())
    return false;

  const char* const type = typeid (_Sig).name ();
  registered_cbs_[type].emplace_front (new Function<_Sig> (f));
  return true;
}