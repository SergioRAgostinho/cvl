/**
	* \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/05/03
  * \date last modified: 2017/05/03
  */
#include <gtest/gtest.h>
#include <cvl/common/function.h>

using namespace ht;

static int
sum (const int lhs, const int rhs)
{
  return lhs + rhs;
}

static void
inc (int& op)
{
  ++op;
}

TEST(Function, FunctionConstructor) {
  // const like funtion
  Function<int(int, int)> f_const (sum);
  // wrap a function which modifies operands
  Function<void(int&)> f_non_const (inc);
}

TEST(Function, FunctorOperator) {

  int op1 = 1, op2 = 2;

  // const like funtion
  Function<int(int, int)> f_const (sum);
  // wrap a function which modifies operands
  Function<void(int&)> f_non_const (inc);

  // Const
  ASSERT_EQ (sum (op1, op2), f_const (op1, op2));
  ASSERT_EQ (sum (1, 2), f_const (1, 2));

  // Non const lvalue
  const int exp = op1 + 1;
  f_non_const (op1);
  ASSERT_EQ (exp, op1);
}
