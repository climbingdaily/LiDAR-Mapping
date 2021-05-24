#pragma once
#include <pcl/common/transforms.h>
#include "common.h"
#include "point_cloud_reader.h"
#include "map_manager.h"
#include "tool_func.h"
#include "lidar_trajectory.h"
#include "submap.h"

#define END_OF_PCAPFILE -1
#define END_OF_TRACKFILE -1
#define CONTINUE_READ 0


class BuildMap
{
  public:
    BuildMap();
    ~BuildMap();
    void setMapPointer(PointCloud::Ptr &map_input);
    void setMapResolution(double res);
    void setPcapFilePath(std::string _pcap_path);
    void setCalibPath(std::string _calib_path);
    void setShowCloud(bool _shcl_bmp);
    void setCalibrationMatrixPath(std::string _matrix_filename);
    void setCalibrationFlag(bool _calib_flag);//Buit map with 2 laser scanners
    void setHorizonFlag(bool _horizon_flag);
    void setHorizonMatrixPath(std::string _h_matrix_path);
    void setOutputPath(std::string _out_path);
    void setDistanceLimit(double _dis_limit);
    void setSaveMap(bool _save_flag) { save_mapdata = _save_flag; }
    bool setPrintFlag(bool _print_flag) { printFlag = _print_flag;}
    void setPointCloudReader2(std::string pcappath, std::string calipath2);
    void setExePath(std::string _path){exepath = _path;};
    
    PointCloud::Ptr getMapPointer();
    void getOctreePtr(OctreePtr &inTree){inTree = map_octree;};
    double getMapResolution();
    int Run();

  protected:
    virtual int init(){}
    virtual int readPose(){}
    int loadFrame();
    
    PointCloudReader reader,reader2;
    DebugToolFunc debug;
    IOToolFunc io_tool;
    
    OctreePtr map_octree;
    ifstream trackReader;
    ofstream build_map_log;
    double map_res;
    long long curFrameId;
    int frameOffset;
    int preID[2];
    
    PointCloud::Ptr map_data;
    PointCloud::Ptr curFrame;
    
    bool calib_flag;
    bool horizon_flag;
    bool showCloud;
    bool save_mapdata;
    bool printFlag;
    
    Matrix4Type curFramePose;
    Matrix4Type calib_matrix;
    Matrix4Type horizon_matrix;
    
    std::string output_cloud_path;
    std::string trackFileName;
    std::string input_pcap;
    std::string exepath;
    
    double errorTransform(Matrix4Type preFramePose, Matrix4Type curFramePose);
};


class BuildMapfromTrajectory : public BuildMap
{
  public:
    BuildMapfromTrajectory();
    ~BuildMapfromTrajectory();
    void SetTrajectory(Trajectory &t);
    void SetTrajectory(PosSet &ps, Trajectory &t);

  private:
    int init();
    int readPose();

    Trajectory::iterator traj_it;
    Trajectory::iterator traj_end;
};


class BuildMapfromG2oVertex : public BuildMap
{
  public:
    //BuildMapfromG2oVertex();
    //~BuildMapfromG2oVertex();
    void SetTrackFilename(std::string s);

  protected:
    int init();
    int readPose();

  private:
    G2oToolFunc g2o;
};


class BuildMapfromTMatrix:public BuildMap
{
  public:
    BuildMapfromTMatrix();
    ~BuildMapfromTMatrix();
    void SetTrackFilename(std::string s);

  protected:
    int init();
    int readPose();

  private:
    int loadPose(istream &in, Matrix4Type &m);
};


class BuildMapWithMultiLayers : public BuildMapfromTMatrix
{
  public:
    BuildMapWithMultiLayers();
    ~BuildMapWithMultiLayers();

    void SetMatrixFilename(std::string _matrix_filename);
    void SetBeginLayerId(long long _begin_id);
    int Run();


  private:
    IOToolFunc io_tool;
    std::string matrix_filename;
    Matrix4Type matrix;
    long long beginLayerId;
};


class BuildSubMap:public BuildMap
{

};