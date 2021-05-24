#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <fstream>
#include <string>
#include <iomanip>
#include "common.h"
#include "lidar_trajectory.h"
#include "tool_func.h"

using namespace std;

class MakeHorizonal
{
public:
  void ReadXYZfromVertex(PointCloud::Ptr cloud);
  void ReadXYZfromVertex(PointCloud::Ptr cloud, long long startId);
  void ReadXYZfromVertex(PointCloud::Ptr cloud, double startDistance, double endDistance);
  Matrix4Type calculateMatrix(double a, double b, double c);
  void SetInputFile(std::string _input_file);
  void SetTotalFrame(int total) { total_Frame = total; };
  void Run();
  void SetStartDistance(double v) { startDis_ = v; }
  void SetEndDistance(double v) { endDis_ = v; }
  void setFrameGap(int gap) { frameGap = gap; }
  MakeHorizonal()
  {
    frameGap = 3;
    startDis_ = 10.0;
    endDis_ = 20.0;
    horizon_matrix = Matrix4Type::Identity();
  }

private:
  int frameGap;
  double startDis_, endDis_;
  void RemoveEdges();
  int total_Frame;
  ifstream input_file;
  string input_path;
  G2oToolFunc g2o_tool;
  Matrix4Type horizon_matrix;
};