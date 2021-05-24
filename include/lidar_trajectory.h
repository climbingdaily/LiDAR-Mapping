#pragma once
#include <iostream>
#include <set>
using namespace std;

struct Pos
{
  long long frame_id;
  Matrix4Type transform;
};
typedef std::vector<Pos> Trajectory;

class PosSetSortCriterion 
{
  public:
      bool operator()(const Pos &left, const Pos &right)
      {
	 if(left.frame_id == right.frame_id) 
           return false;
         else
           return left.frame_id < right.frame_id; 
      }
  
};

typedef std::set<Pos, PosSetSortCriterion> PosSet;