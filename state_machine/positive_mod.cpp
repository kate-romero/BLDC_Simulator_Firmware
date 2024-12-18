#include "positive_mod.h"

int positiveMod(int dividend, int divisor)
{
  int posMod = dividend % divisor;
  if(posMod < 0)
  {
    posMod += divisor;
  }
  return posMod;
}