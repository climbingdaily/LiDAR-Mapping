#pragma once
#ifndef LOAM_MAPPING_H
#define LOAM_MAPPING_H
#include <math.h>
#include <fstream>
#include <stdio.h>
#include <sstream>
#include <time.h>
// #include <boost/graph/graph_concepts.hpp>
#include <opencv/cv.h>
#include <pcl/registration/gicp.h>
#include "pcapFileReader.h"
#include "common.h"
#include "myFunction.h"
#include "featureextraction.h"
#include "common.h"

class Mapping
{
  public:
    Mapping();
    void transformAssociateToMap();
    void transformUpdate();
    void pointAssociateToMap(PointType const *const pi, PointType *const po);
    void pointAssociateTobeMapped(PointType const *const pi, PointType *const po);
    void setPcapPath2(std::string pcapLoc);
    void setCaliPath2(std::string caliPath);
    void setIsMerg2Cloud(bool isMerge2Cloud);
    void setEndFrameNumber(int number);
    void setStartFrameNumber(int number);
    void setNscans(int scans);
    void setNscans2(int scans);
    void setSkipFrameNumber(int n) { skipFrameNumber = n; };
    void setTransformaiton(Eigen::Matrix4f transform) { transformation = transform; };
    void setInitTraj(std::string traj);
    void setVisualization(bool _isshow){isShowCloud = _isshow;};
    void getReader2(PointCloudReader _reader2) { _reader2 = reader2; };
    void setGPSFile(std::string p){ gpsfile= p;};
    void loadMatrixFile(std::string matrixPath);

    int run(std::string txtSaveLoc, std::string fileNamePcap, std::string caliPath);
 
  private:
    PointCloudReader reader2;
    std::string pcap2;
    std::string caliPath2;
    std::string gpsfile;
    int startFrameNumber; //default 0
    int endFrameNumber;   //default -1
    int skipFrameNumber;  //mapping every N frame, default is 3
    int Nscans;           //default 32
    int Nscans2;          //default 16
    bool isMerge2Cloud;   //default false
    bool isInitTraj;      //是否输入了初始轨迹
    bool isShowCloud;
    Vector6d initTraj[30000];
    double initTrajTime[30000];
    int initTrajCount;

    Eigen::Matrix4f transformation;
    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> GICP;
    float isOutdoor(PointCloudReader &reader, int minFlamePointSize);
};
#endif