/**
  * \author Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com
  * \date created: 2017/05/03
  * \date last modified: 2017/08/19
  * \file grabber.h
  * \brief Provides the abstract Grabber implementation
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
  /** \addtogroup io
   *  @{
   */

  /** \brief An abstract implementation of a data source which feeds
    * data through a callback subscription mechanism.
    */
  class Grabber
  {
    public:

      ///////////////////////////////////////////////////////
      //                    Types
      ///////////////////////////////////////////////////////

      /** \brief The working modes of the Grabber */
      enum class Mode : uint8_t
      {
        /** \brief In ASYNC mode, data fetching occurs automatically
          * in a separate thread
          */
        ASYNC,

        /** \brief In TRIGGER mode, data fetching only occurs
          * when the Grabber::trigger method is invoked
          */
        TRIGGER
      };

      ///////////////////////////////////////////////////////
      //                    Methods
      ///////////////////////////////////////////////////////

      /** \brief Supported signatures and operating modes ctor
        *
        * Each grabber implementation needs to supply upon construction
        * the function callback signatures it supports as well as the
        * operating modes it can handle
        * \param[in] supported_sigs - the supported function callback signatures
        * \param[in] supported_modes - the supported operating modes
        */
      Grabber ( const std::set<const char*>& supported_sigs,
                const std::set<Mode>& supported_modes = std::set<Mode> {Mode::ASYNC})
        : supported_sigs_ (supported_sigs)
        , supported_modes_ (supported_modes)
        , mode_ (Mode::ASYNC)
        , running_ (false)
      {}

      /** \brief Returns currently active mode
        * \return The currently active mode
      */
      inline Mode getMode () const { return mode_; }

      /** \brief Returns the supported modes by this grabber instance
        * \return The set of supported modes
        */
      inline const std::set<Mode>&
      getSupportedModes () const { return supported_modes_; }

      /** \brief Provides access to this Grabber supported callback signatures
        * \return The set of supported callback signatures
        */
      inline const std::set<const char*>&
      getSupportedSigs () const { return supported_sigs_; }

      /** \brief Check if the grabber is running (has started)
        * \return True if the grabber is running, false otherwise
        */
      bool isRunning () const { return running_; }

      /** \brief Queries the object to check if the function signature
        * is supported as a callback
        * \return True if the signature is supported, false otherwise
        */
      template<typename _Sig>
      bool isSupported ();

      /** \brief Invoke to register a callback of a certain type
        * \param[in] f - the callable object to be registered
        * \return True if the callback is successfully registered,
        * false otherwise.
        */
      template<typename _Sig>
      bool registerCb (const std::function<_Sig>& f);

      /** \brief Allows setting the mode of the grabber instance
        * \param[in] mode - the new desired mode
        * \return True if the mode is successfully set, false otherwise
        */
      bool setMode (const Mode mode);

      /** \brief Starts the grabber */
      virtual bool start () = 0;

      /** \brief Stops the grabber */
      virtual void stop () = 0;

      /** \brief Trigger manual read from the grabber */
      virtual void trigger () {};

      /** \brief Triggers the unregistering of all callbacks*/
      void unregisterAllCbs ();

    protected:

      ///////////////////////////////////////////////////////
      //                    Members
      ///////////////////////////////////////////////////////

      /** \brief A set with all the supported signatures for this
        * particular Grabber object. Designed to be filled on
        * construction        
        */
      const std::set<const char*> supported_sigs_;

      /** \brief Map with all the registered callbacks, indexed
        * by their signatures
        */
      std::map<const char*, std::forward_list<std::unique_ptr<FunctionBase>>> registered_cbs_;

      /** \brief Supported modes set. ASYNC is always supported */
      const std::set<Mode> supported_modes_;

      /** \brief Holds the working mode  */
      Mode mode_;

      /** \brief Indicates if the grabber is in the middle of a data
        * acquisition situation
        */ 
      bool running_;

      /** \brief Thread object responsible for the async data acquisition */
      std::thread thread_;

      /** \brief (Data) frame number counter */
      size_t frame_nr_;
  };

  /** @}*/
}

#include <cvl/io/impl/grabber.hpp>

#endif //CVL_IO_GRABBER_H_
