#ifndef FEATUREEXTRACTION_H
#define FEATUREEXTRACTION_H
#include <cmath>
#include <vector>
#include "common.h"
#include <opencv/cv.h>
#include <opencv/cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointCloud<PointType> PointCloud;

class FeatureExtraction
{
   
public:
   FeatureExtraction()
   {
      laserCloud.reset(new pcl::PointCloud<PointType>);
      scanPeriod = 0.1;
      systemDelay = 20;
      systemInitCount = 0;
      curvThredhold = 30;//0.1/30
      N_SCANS = 16 ;
   }
   void setInputCloud(pcl::PointCloud<PointType> &laserCloudIn);
   void getCornerSharp(pcl::PointCloud<PointType> &cornerPointsSharp);
   void getCornerPointsLessSharp(pcl::PointCloud<PointType> &cornerPointsLessSharp);
   void getSurfPointsFlat(pcl::PointCloud<PointType> &surfPointsFlat);
   void getSurfPointsLessFlat(pcl::PointCloud<PointType> &surfPointsLessFlat);
   void getFullres(pcl::PointCloud<PointType> &laserCloud);
   void setNscans(int n);//set the scan number of each frame

private:
   double scanPeriod;   
   float curvThredhold;
   int systemDelay;
   int systemInitCount;
   // bool systemInited = false;
   int N_SCANS;
   
   float cloudCurvature[80000];
   int cloudSortInd[80000];
   int cloudNeighborPicked[80000];
   int cloudLabel[80000];
   pcl::PointCloud<PointType> laserCloudIn;
   pcl::PointCloud<PointType> cornerPointsSharp;
   pcl::PointCloud<PointType> cornerPointsLessSharp;
   pcl::PointCloud<PointType> surfPointsFlat;
   pcl::PointCloud<PointType> surfPointsLessFlat;
   pcl::PointCloud<PointType>::Ptr laserCloud;
};

#endif // FEATUREEXTRACTION_H
