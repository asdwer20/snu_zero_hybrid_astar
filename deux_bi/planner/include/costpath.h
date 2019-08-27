// costpath.h

#ifndef COSTPATH
#define COSTPATH

#include <iostream>
#include <vector>
#include <ompl/geometric/PathGeometric.h>

namespace og = ompl::geometric;

class Costpath{
  public:
  og::PathGeometric path;
  double cost;
  // public:
  Costpath(const og::PathGeometric x, double y);
};
Costpath::Costpath(const og::PathGeometric x, double y) : path(x), cost(y) {
}
class Compare {
  public:
  bool operator() (Costpath x, Costpath y)
  {
    return x.cost > y.cost;
  }
};

#endif
