#pragma once
#include "common.h"
#include "map_manager.h"
#include "point_cloud_reader.h"
#include <boost/concept_check.hpp>
#include <fstream>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <utility>
//#include "parameter_reader.h"
#include "build_map.h"
#include "gicp_end.h"
#include "submap.h"
#include "tool_func.h"

struct Constraint
{
   long long reference_frameid;
   long long aligned_frameid;
   Matrix4Type icp_delta_T;
   double icp_error;
};

class LoopDetector
{
 public:
   LoopDetector();
   ~LoopDetector();
   int Run();
   void ShowDetectedMap(bool _show_flag);
   void SetDetectMapRes(double _detect_map_res);
   void SetPcapFilePath(std::string _pcap_file_path);
   void SetCalibFilePath(std::string _cali_file_path);
   void SetFrontResultPath(std::string _front_path);
   void SetDetectDistance(double _dis);
   void SetMaxIcpError(double _icp_eror) { icp_error_threshold = _icp_eror; }
   void SetOutputFile(std::string _out_file);
   void SetShowDetectTraj(bool _show_traj) { showTraj = _show_traj; }

 private:
   void loadFrontResult();
   void DetectLoop();
   void loadTrajectory();
   void SaveTraj(SubMap &m);
   void caculateAndSaveNewEdges();
   void addConstraint(long long refer_, long long align_, Matrix4Type T, double error);
   void addToTrajectory(long long int frame_id, Matrix4Type T, Trajectory &origin_trajectory);

   void openFrontResult();
   void DetectLoop2();
   bool findPoseNeighbors(long long i, SubMap2 &_align_submap);

   BuildMapfromTrajectory buildMap;
   double detect_res;
   double icp_error_threshold;
   GicpEnd gicp;
   bool showCloud;
   bool showTraj;
   G2oToolFunc g2o;
   IOToolFunc io_tool;
   DebugToolFunc debug;
   std::vector<std::string> g2o_input;
   std::vector<Constraint> constraints;
   std::ofstream icp_delta_log;
   std::vector<Matrix4Type> centers;
   std::vector<SubMap> submaps;
   Matrix4Type cali_T;
   std::ifstream frontResult;
   Trajectory origin_trajectory;
   std::string pcapFile;
   std::string calibFile;

   Matrix4Type curPose;
   long long curFrameId;
   bool *loop_flag;
   int loam_step;
   std::string front_result_path;
   double detect_distance;
   std::ofstream result;
};
