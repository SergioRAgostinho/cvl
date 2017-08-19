/**
	* \author Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com
  * \date created: 2017/05/03
  * \date last modified: 2017/08/19
  * \file function.h
  * \brief Defines all function related types
  */
#pragma once
#ifndef CVL_COMMON_FUNCTION_H_
#define CVL_COMMON_FUNCTION_H_

#include <functional>

namespace ht
{
  /** \addtogroup common
   *  @{
   */

  /** \class FunctionBase
    * \brief A bare base class which just defines a virtual destructor
    *
    * Plays a superset role, allowing the have containers of
    * functions. The intended use is to allow storing functions of
    * different types under the same container
    */
  class FunctionBase
  {
    public:

      /** \brief Virtual default dtor */
      virtual ~FunctionBase() = default;
  };

  /** \class Function
    * \brief A wrapper class for any callable object.
    * \note Only the callable instantiation is usable.
    */
  template<typename>
  class Function;

  /** \brief A template specialization for Function which wraps any
    * callable object.
    * \note This is the only usable template specialization of Function.
    */
  template<typename _Ret, typename... _Args>
  class Function<_Ret (_Args...)> : public FunctionBase
  {
    public:

      /** \brief Ctor initilizing the std::function object
        * \param[in] f - another callable object
        */
      Function (const std::function<_Ret(_Args...)>& f)
        : f_ (f)
      {}

      /** \brief The callable operator
        * \param [in,out] args - any number or argument
        * \return Whatever the callable object returns.
        */
      inline _Ret operator() (_Args... args) const
      {
        return f_ (std::forward<_Args> (args)...);
      }
      
    protected:

      /** \brief The callable object */
      std::function<_Ret(_Args...)> f_;
  };

  /** @}*/
}

#endif //CVL_COMMON_FUNCTION_H_
