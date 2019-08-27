// binomial_heap.h

#ifndef BINOMIALHEAP
#define BINOMIALHEAP

#include <vector>
#include <iostream>
#include <list>

#include "costpath.h"

using namespace std;

struct Node
{
    int degree;
    Costpath data;
    Node *child, *sibling, *parent;
};


#endif
