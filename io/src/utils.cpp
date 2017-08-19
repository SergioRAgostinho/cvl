/**
  * \author Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com
  * \date created: 2017/07/28
  * \date last modified: 2017/07/28
  */
#include <cvl/io/utils.h>

std::istream&
ht::getline_rn (std::istream& is, std::string& s)
{
  std::string line;
  char c;
  s.clear ();

  while (true)
  {
    if (!std::getline (is, line, '\r'))
      break;

    s += line;

    if (!is.get (c) || c == '\n')
      break;

    s += c;
  }
	return is;
}
