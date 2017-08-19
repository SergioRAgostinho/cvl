/**
  * \author Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com
  * \date created: 2017/07/28
  * \date last modified: 2017/08/19
  * \file utils.h
  * \brief Utility implementation data exchange.
  */
#pragma once
#ifndef CVL_IO_UTILS_H
#define CVL_IO_UTILS_H

#include <iostream>
#include <string>

namespace ht
{
  /** \addtogroup common
   *  @{
   */

  /** \brief Fetch lines from an input stream with the \\r\\n terminator
    * \param[in,out] is - the input stream
    * \param[out] s - the string holding the new line
    * \return The input stream
    */
  std::istream& getline_rn (std::istream& is, std::string& s);

  /** @}*/
}

#endif //CVL_IO_UTILS_H
