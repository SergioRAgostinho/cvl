/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/05/03
  * \date last modified: 2017/05/05
  */
#pragma once
#ifndef CVL_IO_GRABBER_H_
#define CVL_IO_GRABBER_H_

#include <cvl/common/function.h>
#include <forward_list>
#include <map>
#include <set>
#include <thread>

namespace ht
{
  class Grabber
  {
    public:

      ///////////////////////////////////////////////////////
      //                    Types
      ///////////////////////////////////////////////////////

      enum class Mode : uint8_t
      {
        ASYNC,
        TRIGGER
      };

      ///////////////////////////////////////////////////////
      //                    Methods
      ///////////////////////////////////////////////////////

      /** \brief Supported signatures ctor */
      Grabber ( const std::set<const char*>& supported_sigs,
                const std::set<Mode>& supported_modes = std::set<Mode> {Mode::ASYNC})
        : supported_sigs_ (supported_sigs)
        , supported_modes_ (supported_modes)
        , mode_ (Mode::ASYNC)
        , running_ (false)
      {}

      /** \brief Default dtor */
      ~Grabber () { unregisterAllCbs (); }

      /** \brief Returns currently active mode */
      inline Mode getMode () const { return mode_; }

      /** \brief Returns the supported modes by this grabber instance */
      inline const std::set<Mode>&
      getSupportedModes () const { return supported_modes_; }

      /** \brief Exposes a const ref to the suuported signatures */
      inline const std::set<const char*>&
      getSupportedSigs () const { return supported_sigs_; }

      /** \brief Getter for the running status flag */
      bool isRunning () const { return running_; }

      /** \brief Check if signature is supported */
      template<typename _Sig>
      bool isSupported ();

      /** \brief Invoke to register a callback of a certain type */
      template<typename _Sig>
      bool registerCb (const std::function<_Sig>& f);

      /** \brief Allows setting the mode of the grabber instance */
      bool setMode (const Mode mode);

      /** \brief Starts the grabber */
      virtual bool start () = 0;

      /** \brief Stops the grabber */
      virtual void stop () = 0;

      /** \brief Triggers the unregistering of all callbacks*/
      void unregisterAllCbs ();

    protected:

      ///////////////////////////////////////////////////////
      //                    Members
      ///////////////////////////////////////////////////////

      /** \brief A set with all the suported signatures for this
        * particular Calllback object. Designed to be filled on
        * construction        
        */
      const std::set<const char*> supported_sigs_;

      /** \brief registered callbacks */
      std::map<const char*, std::forward_list<FunctionBase*>> registered_cbs_;

      /** \brief Supported modes set. ASYNC is always supported */
      const std::set<Mode> supported_modes_;

      /** \brief Current working mode  */
      Mode mode_;

      /** \brief Indicates if the grabber is in the middle of a data
        * acquisition situation
        */ 
      bool running_;

      /** \brief Used to lanch the async execution */
      std::thread thread_;
  };
}

#include <cvl/io/impl/grabber.hpp>

#endif //CVL_IO_GRABBER_H_
