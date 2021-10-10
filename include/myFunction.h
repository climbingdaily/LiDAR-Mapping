#pragma once

#ifndef MYFUNCTION_H
#define MYFUNCTION_H

#include "point_cloud_reader.h"
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/visualization/cloud_viewer.h>
#include "common.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
typedef Eigen::Matrix<double, 6, 1> Vector6d;

//显示特征点
inline void ShowFeature(const pcl::PointCloud<PointType>::Ptr cornerPointsSharp,
                        const pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp,
                        const pcl::PointCloud<PointType>::Ptr surfPointsFlat,
                        const pcl::PointCloud<PointType>::Ptr surfPointsLessFlat,
                        pcl::visualization::PCLVisualizer *viewerfeature = new pcl::visualization::PCLVisualizer("defalut feature"))
{
   //pcl::visualization::PCLVisualizer *viewerfeature = new pcl::visualization::PCLVisualizer ("Show feature");
   viewerfeature->removePointCloud("cloud1");
   viewerfeature->removePointCloud("cloud2");
   viewerfeature->removePointCloud("cloud3");
   viewerfeature->removePointCloud("cloud4");
   pcl::visualization::PointCloudColorHandlerCustom<PointType> feature1(cornerPointsSharp, 255, 0, 0);      //red
   pcl::visualization::PointCloudColorHandlerCustom<PointType> feature2(cornerPointsLessSharp, 0, 255, 0);  //green
   pcl::visualization::PointCloudColorHandlerCustom<PointType> feature3(surfPointsFlat, 0, 0, 255);         //blue
   pcl::visualization::PointCloudColorHandlerCustom<PointType> feature4(surfPointsLessFlat, 255, 255, 255); //white
   viewerfeature->addPointCloud(cornerPointsSharp, feature1, "cloud1");
   viewerfeature->addPointCloud(cornerPointsLessSharp, feature2, "cloud2");
   viewerfeature->addPointCloud(surfPointsFlat, feature3, "cloud3");
   viewerfeature->addPointCloud(surfPointsLessFlat, feature4, "cloud4");
   viewerfeature->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud1");
   viewerfeature->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud3");
   while (!viewerfeature->wasStopped())
      viewerfeature->spinOnce();
   viewerfeature->~PCLVisualizer();
}

inline void addCloud(const PointCloud::Ptr cloud, int color,
                     pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("defalut feature"))
{
   switch (color%4)
   {
   case 0:
   {
      pcl::visualization::PointCloudColorHandlerCustom<PointType> red(cloud, 255, 0, 0); //red
      viewer->addPointCloud(cloud, red, std::to_string(color).c_str());
      break;
   }
   case 1:
   {
      pcl::visualization::PointCloudColorHandlerCustom<PointType> green(cloud, 0, 255, 0); 
      viewer->addPointCloud(cloud, green, std::to_string(color).c_str());
      break;
   }
   case 2:
   {
      pcl::visualization::PointCloudColorHandlerCustom<PointType> blue(cloud, 0, 0, 255); 
      viewer->addPointCloud(cloud, blue, std::to_string(color).c_str());
      break;
   }
   default:
      pcl::visualization::PointCloudColorHandlerCustom<PointType> white(cloud, 255, 255, 255); 
      viewer->addPointCloud(cloud, white, std::to_string(color).c_str());
      break;
   }
   viewer->spinOnce();
}

//显示点云
inline void ShowCloud(const PointCloud::Ptr cloud,
                      pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("defalut feature"),
                      std::string name = "cloud")
{
   viewer->removePointCloud(name);
   //从电云intensity字段生成颜色信息
   pcl::visualization::PointCloudColorHandlerGenericField<PointType> cloud_handle(cloud, "intensity");
   viewer->addPointCloud(cloud, cloud_handle, name); 
   viewer->spinOnce();
}
/*
 * Align the pointcloud to map(ry*rx*rz)
 */
inline void pointAssociateToMap(PointType const *const pi, PointType *const po, Vector6d transformTobeMapped)
{
   double rx, ry, rz, tx, ty, tz;
   rx = transformTobeMapped[0];
   ry = transformTobeMapped[1];
   rz = transformTobeMapped[2];
   tx = transformTobeMapped[3];
   ty = transformTobeMapped[4];
   tz = transformTobeMapped[5];
   float x1 = cos(rz) * pi->x - sin(rz) * pi->y;
   float y1 = sin(rz) * pi->x + cos(rz) * pi->y;
   float z1 = pi->z;

   float x2 = x1; //x1
   float y2 = cos(rx) * y1 - sin(rx) * z1;
   float z2 = sin(rx) * y1 + cos(rx) * z1;

   po->x = cos(ry) * x2 + sin(ry) * z2 + transformTobeMapped[3];
   po->y = y2 + ty;
   po->z = -sin(ry) * x2 + cos(ry) * z2 + tz;
   po->intensity = pi->intensity;
}

/*
 * 输入当前帧的地址
 * 点在该帧中的位置
 * 求周围10个点的向量和的长度平方(Σ(x-xi))²,不是距离平方和
 */
inline float computeCurveture(PointCloud::Ptr frame, int i)
{
   int size = frame->size();
   if (i < 0 || i > size)
   {
      std::cerr << "MYFUNCTION_H: Wrong size!" << std::endl;
      return 0;
   }

   float dis, diffX, diffY, diffZ;
   diffX = diffY = diffZ = 0;
   for (int k = 0; k < 11; k++)
   {
      dis = sqrt(
          (frame->points[i].x - frame->points[i - 5 + k].x) * (frame->points[i].x - frame->points[i - 5 + k].x) + (frame->points[i].y - frame->points[i - 5 + k].y) * (frame->points[i].y - frame->points[i - 5 + k].y) + (frame->points[i].z - frame->points[i - 5 + k].z) * (frame->points[i].z - frame->points[i - 5 + k].z));
      if (dis != 0)
      {
         diffX += (frame->points[i].x - frame->points[i - 5 + k].x) / dis;
         diffY += (frame->points[i].y - frame->points[i - 5 + k].y) / dis;
         diffZ += (frame->points[i].z - frame->points[i - 5 + k].z) / dis;
      }
   }
   //后5+前5-10*当前坐标,相当于求10个向量的差值
   /*
   float diffX = frame->points[i - 5].x + frame->points[i - 4].x
   + frame->points[i - 3].x + frame->points[i - 2].x
   + frame->points[i - 1].x - 10 * frame->points[i].x
   + frame->points[i + 1].x + frame->points[i + 2].x
   + frame->points[i + 3].x + frame->points[i + 4].x
   + frame->points[i + 5].x;
   float diffY = frame->points[i - 5].y + frame->points[i - 4].y
   + frame->points[i - 3].y + frame->points[i - 2].y
   + frame->points[i - 1].y - 10 * frame->points[i].y
   + frame->points[i + 1].y + frame->points[i + 2].y
   + frame->points[i + 3].y + frame->points[i + 4].y
   + frame->points[i + 5].y;
   float diffZ = frame->points[i - 5].z + frame->points[i - 4].z
   + frame->points[i - 3].z + frame->points[i - 2].z
   + frame->points[i - 1].z - 10 * frame->points[i].z
   + frame->points[i + 1].z + frame->points[i + 2].z
   + frame->points[i + 3].z + frame->points[i + 4].z
   + frame->points[i + 5].z;*/

   //如果点在平面,向量就约等于0
   float cloudCurvature = diffX * diffX + diffY * diffY + diffZ * diffZ;
   return cloudCurvature;
}
inline void myPointAssociateToMap(PointType &pi, PointType &po, const Vector6d &transfrom)
{
   Eigen::Vector3d in(pi.x, pi.y, pi.z);
   Eigen::Vector3d out(0, 0, 0);
   Eigen::Matrix3d R;

   Eigen::Matrix3d Rx = Eigen::AngleAxisd(transfrom[0], Eigen::Vector3d(1, 0, 0)).toRotationMatrix();
   Eigen::Matrix3d Ry = Eigen::AngleAxisd(transfrom[1], Eigen::Vector3d(0, 1, 0)).toRotationMatrix();
   Eigen::Matrix3d Rz = Eigen::AngleAxisd(transfrom[2], Eigen::Vector3d(0, 0, 1)).toRotationMatrix();

   R = Ry * Rx * Rz;
   out = R * in;
   po.x = out[0] + transfrom[3];
   po.y = out[1] + transfrom[4];
   po.z = out[2] + transfrom[5];
}
/*
void quaterniondToMap( PointType &pi, PointType &po, Eigen::Quaterniond &q, Eigen::Vector3d T)
{
   Eigen::Vector3d in(pi.x, pi.y, pi.z);
   Sophus::SO3 SO3_q( q );
   Eigen::Vector3d out(0, 0, 0);

   out = SO3_q.matrix() * in + T;
   po.x = out[0];
   po.y = out[1];
   po.z = out[2];
}*/

/*
inline void transformToMaptest(std::string fileNamePcap, std::string calibrationPath,
                               std::string txtlocation)
{
   Vector6d transform;
   pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("Show cloud");
   pcl::visualization::PCLVisualizer *viewer2 = new pcl::visualization::PCLVisualizer("Show cloud2");
   PointCloud::Ptr subMap(new PointCloud);
   PointCloud::Ptr filtered_map(new PointCloud);
   PointCloud::Ptr map(new PointCloud);
   PointCloud::Ptr cur_frame(new PointCloud);
   PointCloud::Ptr filtered_frame(new PointCloud);
   PointCloud::Ptr dist_frame(new PointCloud);
   PointCloud::Ptr corner_frame(new PointCloud);
   PointCloud::Ptr surf_frame(new PointCloud);

   pcl::VoxelGrid<PointType> downSizeFilterFrame;
   downSizeFilterFrame.setLeafSize(0.02, 0.02, 0.02);
   pcl::VoxelGrid<PointType> downSizeCornerFrame;
   downSizeCornerFrame.setLeafSize(0.02, 0.02, 0.02);
   pcl::VoxelGrid<PointType> downSizeSurfFrame;
   downSizeSurfFrame.setLeafSize(0.2, 0.2, 0.2);
   pcl::VoxelGrid<PointType> downSizeFilterMap;
   downSizeFilterMap.setLeafSize(0.04, 0.04, 0.04);
   pcl::StatisticalOutlierRemoval<PointType> sor;

   PointCloudReader reader(fileNamePcap, calibrationPath);
   fileNamePcap = fileNamePcap.substr(fileNamePcap.rfind('/') + 1);
   std::ifstream file(txtlocation.c_str());

   if (!file)
   {
      std::cerr << "File open error!\n";
      return;
   }
   int tempi = 0;
   while (reader.readPointCloud(cur_frame))
   {
      tempi++;
      viewer->spinOnce();

      std::string ss;
      if (std::getline(file, ss))
      {
         std::getline(file, ss);
         std::getline(file, ss);
         std::istringstream line(ss);
         std::string temp;
         int id;
         Eigen::Quaterniond q;
         line >> temp >> id;
         line >> transform[3] >> transform[4] >> transform[5] >> q.x() >> q.y() >> q.z() >> q.w();
         while (id > tempi)
         {
            viewer->spinOnce();
            if (reader.readPointCloud(cur_frame))
               tempi++;
            else
            {
               //std::cerr << "End of frame!" << std::endl;
               return;
            }
         }
         line >> temp >> id;
         line >> transform[3] >> transform[4] >> transform[5] >> q.x() >> q.y() >> q.z() >> q.w();

         line >> temp >> transform[0] >> transform[1] >> transform[2] >> transform[3] >> transform[4] >> transform[5]; //read transformTobeMapped
         cout << "read frame id = " << tempi << endl;

         filtered_frame = cur_frame;
         corner_frame->clear();
         surf_frame->clear();
         int cur_frameNum = filtered_frame->points.size();
         for (int i = 0; i < cur_frameNum; i++)
         {
            PointType &pi = filtered_frame->points[i];
            double x = pi.y;
            double y = pi.z;
            double z = pi.x;
            if (x * x + y * y + z * z > 225)
               continue;
            myPointAssociateToMap(pi, pi, transform);
            //pointAssociateToMap(&pi, &pi, transform);
            if (computeCurveture(filtered_frame, i) > 0.2)
               corner_frame->push_back(pi);
            else
               surf_frame->push_back(pi);
         } //for
         ShowCloud(corner_frame, viewer2);

         filtered_frame->clear();
         downSizeCornerFrame.setInputCloud(corner_frame);
         downSizeCornerFrame.filter(*filtered_frame);
         *subMap += *filtered_frame;

         filtered_frame->clear();
         downSizeSurfFrame.setInputCloud(surf_frame);
         downSizeSurfFrame.filter(*filtered_frame);
         *subMap += *filtered_frame;

         ShowCloud(subMap, viewer);
         
	//    if(tempi % 300 == 0)
	//    {
	//       filtered_map->clear();
	//       downSizeFilterMap.setInputCloud(subMap);
	//       downSizeFilterMap.filter(*filtered_map);
	//   map += *filtered_map;
	  
	//       std::ostringstream os;
	//       os << fileNamePcap << tempi/300;
	//       std::string tempName = os.str();
	//       pcl::io::savePCDFileASCII ("/home/daiyudi/Desktop/seg_" + tempName + ".xyz",  *filtered_map);
	//       subMap->clear();
   //    }
      } //if
      else
         break;
   } //while
   filtered_map->clear();
   downSizeFilterMap.setInputCloud(subMap);
   downSizeFilterMap.filter(*filtered_map);
   *map += *filtered_map;

   filtered_map->clear();
   downSizeFilterMap.setInputCloud(map);
   downSizeFilterMap.filter(*filtered_map);
   map->clear();
   *map += *filtered_map;
   pcl::io::savePCDFileASCII("/home/daiyudi/Desktop/" + fileNamePcap + ".xyz", *map);
   viewer->spin();
}
*/

/*
 * 输入点云指针，帧号，名称
 * */
inline void showTime(PointCloud::Ptr frame, int frameID, std::string frameName = "Frame", double start_frameTime = 0)
{
   double st = frame->points[0].data_n[0] + frame->points[0].data_n[1];
   double ft;
   if (start_frameTime > 0)
      ft = st - start_frameTime; 
   else
      ft = frame->points.back().data_n[0] - frame->points[0].data_n[0] + frame->points.back().data_n[1] - frame->points[0].data_n[1];
   cout << frameName << "\tFrameID: " << frameID << "\tsize = " << frame->points.size() << "\t";
   cout << "statTime = " << std::fixed << std::setprecision(2) << st
        << "\tframeTime = " << std::fixed << std::setprecision(4) << ft
        << endl;
}

inline void getCloudFromPcd(pcl::PointCloud<pcl::PointXYZI>::Ptr pcdcloud, std::string pcdFile)
{
   // pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("Show cloud");
   pcdcloud->clear();
   if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcdFile, *pcdcloud) == -1)
	{
		PCL_ERROR("Cloudn't read pcd file!");
		return ;
	}
   // ShowCloud(pcdcloud, viewer);
   // while (!viewer->wasStopped())
   //    viewer->spinOnce();
   // viewer->close();
   return ;
}

inline void getCloudFromeXYZ(pcl::PointCloud<pcl::PointXYZI>::Ptr xyzcloud, std::string xyzfile)
{
   xyzcloud->clear();
   ifstream cloudfile(xyzfile);
   double t;
   if(!cloudfile)
   {
      cout << "读取XYZ文件错误：" << xyzfile << endl;
      exit(-1);
   }
   while(cloudfile)
   {
      pcl::PointXYZI pt;
      cloudfile >> pt.x >> pt.y >> pt.z >> pt.intensity >> t;
      xyzcloud->push_back(pt);
   }
   cloudfile.close();
}


inline void getCloudFromeSpinXYZ(PointCloud::Ptr xyzcloud, std::string xyzfile, double &frametime)
{
   xyzcloud->clear();
   ifstream cloudfile(xyzfile);
   double timestamp;
   int ring;
   bool firstpoint = true;
   std::string line;
   int temp;

   if(!cloudfile)
   {
      cout << "读取XYZ文件错误：" << xyzfile << endl;
      exit(-1);
   }
   while(getline(cloudfile, line))
   {
      line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
      line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
      std::istringstream line_data(line);

      PointType pt;
      line_data >> pt.x >> pt.y >> pt.z >> pt.intensity >> timestamp >> ring >> temp;

      // skip NaN and INF valued points
      if (!pcl_isfinite(pt.x) ||
          !pcl_isfinite(pt.y) ||
          !pcl_isfinite(pt.z))
         continue;

      // skip zero valued points
      float distance = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z; 
      if (distance < 0.4 * 0.4 || distance > 50 * 50)
         continue;
      
      if (firstpoint){
         frametime = timestamp;
         firstpoint = false;
      }
      pt.data_n[0] = int(timestamp);           //秒
      pt.data_n[1] = timestamp - pt.data_n[0]; //微秒
      pt.data_n[2] = ring;
      pt.data_n[3] = sqrt(distance);
      xyzcloud->push_back(pt);
   }
   cloudfile.close();
}

#endif