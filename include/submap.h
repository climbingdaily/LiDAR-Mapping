#pragma once
#include "tool_func.h"
#include "lidar_trajectory.h"

class SubMap
{
  public:
    int id;
    Trajectory traj;
    Pos center;
    void SaveTraj(std::string _path, long long _traj_id);
    double Distance(SubMap s);
    void GetCenter();

  private:
    G2oToolFunc g2o;

};

class SubMap2
{
  public:
    PosSet traj;
    void SaveTraj(std::string _path);
    void ShowTraj(DebugToolFunc* debug_tool);
    long long GetCenterPosId();
    
  private:
    G2oToolFunc g2o;
   
};