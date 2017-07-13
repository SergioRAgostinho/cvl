/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/05/11
  * \date last modified: 2017/05/11
  */
#include <thread>

void func () {}

int
main ()
{
  std::thread t (func);
  return 0;
}