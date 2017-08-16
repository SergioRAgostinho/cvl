/**
	* \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/05/03
  * \date last modified: 2017/08/14
  */
#pragma once
#ifndef CVL_COMMON_FUNCTION_H_
#define CVL_COMMON_FUNCTION_H_

#include <functional>

namespace ht
{
  /** \brief Plays a superset role, allowing the have containers of
    * functions
    */
  class FunctionBase
  {
    public:

      virtual ~FunctionBase() = default;
  };

  /** \brief Mock class to allow constructing templates with function
    * signture types
    */
  template<typename>
  class Function;

  /** \brief Generic function wrappes which derives from a parent
    * class. The intended use is to allow storing functions of
    * different types under the same container
    */
  template<typename _Ret, typename... _Args>
  class Function<_Ret (_Args...)> : public FunctionBase
  {
    public:

      Function (const std::function<_Ret(_Args...)>& f)
        : f_ (f)
      {}

      inline _Ret operator() (_Args... args) const
      {
        return f_ (std::forward<_Args> (args)...);
      }
      
    protected:

      std::function<_Ret(_Args...)> f_;
  };
}

#endif //CVL_COMMON_FUNCTION_H_
