#ifndef TOOL_FUNC_H
#define TOOL_FUNC_H
#include "common.h"
#include "lidar_trajectory.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>
#include <time.h>
#include <boost/graph/graph_concepts.hpp>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;
using namespace std;
using namespace Eigen;


class ToolFunc;
class TimeCounter;
class G2oToolFunc;
class DebugToolFunc;

struct Vertex{
    long long id;
    Vector3Type pos;
    QuaternionType qua;
};


class ToolFunc
{
  public:
      template <typename T>
      std::string toString(T other_type)
      {
	std::stringstream s;
	s << other_type;
	return s.str();
      }

      template <typename T>
      T toValue(std::string &s)
      {
	  std::stringstream ss;
	  T value;
	  ss << s;
	  ss >> value;
	  return value;
      }      
      

      void makeDir(std::string dir);
    
  protected:
      void makedir(std::string dir);
    
};


class TimeCounter:public ToolFunc
{
  public:
      TimeCounter();
      TimeCounter(std::string s);
      void setProcessName(std::string s);
      void begin();
      double getFinishTime();
      std::string end();
      std::string record(std::string processName);

  private:
      clock_t start, finish;
      std::string processName;
};



class IOToolFunc:public ToolFunc
{
  public:
      IOToolFunc();
      void setBackbagSystem(bool _ubgs);
      void setCalibMatrixPath(std::string _cmp);
      void setSubMapSavePath(std::string _smsp);
      void LoadCalibMatrix(Matrix4Type &cali_T);
      void SaveSubMap(int id, PointCloud::Ptr cloud);
      void SaveCloudByPcapName(std::string pcapName, PointCloud::Ptr cloud);
      void SetMatrixFilePath(std::string _matrix_path);
      void LoadMatrixFromFile(Matrix4Type &m);
      
  private:
      bool use_backbag_system;
      std::string matrix_path;
      std::string submap_save_path;
};



class G2oToolFunc:public ToolFunc
{
  public:
      double getDistance(Matrix4Type pos1, Matrix4Type pos2);
      void printInfoMatrix(ostream &g2o_file);
      long long vertex2Transform(ifstream &g2o_file,Matrix4Type &transform);
      Vector3Type getTranslation(Matrix4Type &transform);
      Matrix3Type getRotation(Matrix4Type &transform);
      QuaternionType rotation2Quaternion(Matrix3Type rotation);
      Matrix4Type rt2Transform(Matrix3Type &r, Vector3Type &t);
      long long int vertex2QT(ifstream& g2o_file, QuaternionType &q, Vector3Type &t);
      std::string informationMatrixInline();
      std::string transform2Vertex(long long id, Matrix4Type m);
      std::string transform2Edge(long long from, long long to, Matrix4Type m);
      std::string caculate_edge(long long from, long long to);
      std::string caculate_edge(long long from, long long to, Matrix4Type icp_delta);
      std::string caculate_edge(long long from, long long to, PosSet &track);
      std::string caculate_edge(long long from, long long to, PosSet &track, Matrix4Type icp_delta);
      
      long long startId;
      std::vector<Matrix4Type> track;
};


class DebugToolFunc:public ToolFunc
{
  public:
      std::string getEularAngle(Matrix4Type &m);
      void ShowCloud(const PointCloud::Ptr cloud);
      void ShowCloud(const pcl::PointCloud< PointType >::Ptr cloud, long long int id);
      void ShowCloudWithCenter(const PointCloud::Ptr cloud, Matrix4Type &center, int mapid);
      
      pcl::visualization::PCLVisualizer *p;

  private:
      G2oToolFunc g2o;
  
};

#endif
