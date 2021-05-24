#ifndef POINT_CLOUD_READER_H
#define POINT_CLOUD_READER_H

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "pcapFileReader.h"
#include "common.h"
#include "myFunction.h"
#include "multiscanregistration.h"

class PointCloudReader{
public:
    PointCloudReader();
    PointCloudReader(std::string fileNamePcap, std::string calibrationPath);

    void setPcapPath(std::string _filePath);
    // void setPcapPath(std::string pc){ fileNamePcap = pc; }
    // void setCalibrationPath(std::string ca){ calibrationPath = ca; }
    void setCalibrationPath(std::string ca);
    void SetVoxelGridFlag(bool _vg_flag = false){ voxel_grid_flag = _vg_flag; }
    bool GetVoxelGridFlag(){ return voxel_grid_flag;}
    void SetVoxelGridLeafsize(float _vg_lf = 0.05) { voxel_grid_leafsize = _vg_lf; }
    void SetDistanceControl(float _dis_ctrl = 20.0) { distanceControl = _dis_ctrl; }
    void SetFirstFrameSparseThreshold(int _first_st = 100){ sparseThreshold = _first_st; }
    std::string getPcdById(int frameid){return files[frameid];};
    long long getTotalFrame()
    { 
        if (!IsReadSuc()) return 0; 
        if(isPcap) return reader.totalFrame(); 
        return files.size();}
    bool IsReadSuc(){ return isReadSuc;}
    void PointCloudFilter(pcl::PointCloud<PointType>::Ptr cloudin, pcl::PointCloud<PointType>::Ptr cloudout, float Leafsize);

    bool OpenPcap();
    int readPointCloud(PointCloud::Ptr cloud, bool record = true);
    int LoadFirstFrame(PointCloud::Ptr frame);
    void VoxelGrid(PointCloud::Ptr cloud);
    void setScansGap(int scanGap);
    void setFrameGap(int frameGap);
    void setGPSFile(std::string p){gpsFile = p;};
    double getTimpStamp(int frameid);
    double startTimeStamp = 0;

    long long frameNumber;
    std::string fileNamePcap;
    std::string calibrationPath;
private:
    MultiScanRegistration cloudFromeScan;
    pcapReader reader;
    pcl::VoxelGrid<PointType> grid;
    veloFrame frame;
    bool isReadSuc;
    bool isPcap;
    bool voxel_grid_flag;
    float voxel_grid_leafsize;
    float distanceControl;
    int sparseThreshold;
    int scanGap;
    int frameGap;
    std::vector<std::string> files; //用来存pcd的文件名  
    std::vector<double> timeStamps; //pcd的时间戳
    std::string filePath; //文件地址
    std::string gpsFile; //保存GPS信息的文件地址
};

 
 //定义回调参数结构体 
// struct callback_args{
// 	// structure used to pass arguments to the callback function
// 	PointCloud::Ptr  pointcloud;
// 	pcl::visualization::PCLVisualizer::Ptr viewerPtr;
// };
#endif
