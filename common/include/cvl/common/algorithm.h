/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/05/11
  * \date last modified: 2017/05/11
  */
#pragma once
#ifndef CVL_COMMON_ALGORITHM_H_
#define CVL_COMMON_ALGORITHM_H_

namespace ht
{
  /** \brief Returns the size of a plain array */
	template<typename _T, size_t _N>
  inline constexpr size_t size (_T (&)[_N]) noexcept
  {
    return N;
  }
}
#endif //CVL_COMMON_ALGORITHM_H_
