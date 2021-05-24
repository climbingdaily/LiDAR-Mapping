#pragma once
#include <pcl/common/transforms.h>
#include <pcl/registration/gicp.h>
#include "common.h"
#include "point_cloud_reader.h"
#include "map_manager.h"
//#include "parameter_reader.h"
#include "tool_func.h"

using namespace std;
//extern ParameterReader para;

class GicpEnd
{
  public:
    GicpEnd();
    ~GicpEnd();
    int Run();
    void SetShowMap(bool _shcl_flag);
    double PairAlign_Segal (const PointCloud::Ptr base, const PointCloud::Ptr data, Eigen::Matrix4f &pairTransform);
    double PairAlign_PCL (const PointCloud::Ptr base, const PointCloud::Ptr data, Matrix4Type &pairTransform);
    double UseNeighborsAlign(OctreePointCloudSearch::Ptr octree, PointCloud::Ptr cur_frame, MatrixType &pairTransform);
    void SetSubMapSize(int _smps);
    void setOutputFile(std::string _out_file);
    
  private:
    void AlignFrameWithMap(OctreePointCloudSearch::Ptr octree, PointCloud::Ptr cur_frame, MatrixType &pairTransform);
    bool subMapFull(PointCloudReader &reader);
    void saveTrack();
    PointCloudReader reader;
    Matrix4Type Global;
    G2oToolFunc g2o_tool;
    DebugToolFunc debug;
    bool usePCL;
    bool globalFirst;
    bool showCloud;
    ofstream gicp_log;
    ofstream vertex_file;
    int subMapSize;
};