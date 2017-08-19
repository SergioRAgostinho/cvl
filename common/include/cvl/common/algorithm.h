/**
  * \author Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com
  * \date created: 2017/05/11
  * \date last modified: 2017/08/19
  * \file algorithm.h
  * \brief General utility functions.
  */
#pragma once
#ifndef CVL_COMMON_ALGORITHM_H_
#define CVL_COMMON_ALGORITHM_H_


namespace ht
{
  /** \addtogroup common
   *  @{
   */

  /** \brief Returns the size of a plain array
    * \param[in] - an plain C-style array
    * \return Returns the number of elements in the array
    */
	template<typename _T, size_t _N>
  inline constexpr size_t size (_T (&)[_N]) noexcept
  {
    return N;
  }

  /** @}*/
}

#endif //CVL_COMMON_ALGORITHM_H_
