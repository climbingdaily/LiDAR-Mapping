#include "featureextraction.h"
using std::atan2;
using std::cos;
using std::sin;

void FeatureExtraction::setInputCloud(pcl::PointCloud<PointType> &laserCloudIn)
{

   cornerPointsSharp.clear();
   cornerPointsLessSharp.clear();
   surfPointsFlat.clear();
   surfPointsLessFlat.clear();
   laserCloud->clear();
   //领域点的ID
   std::vector<int> scanStartInd(N_SCANS, 0);
   std::vector<int> scanEndInd(N_SCANS, 0);

   std::vector<int> indices;
   pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
   int cloudSize = laserCloudIn.points.size();
   float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
   float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y, laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;

   //2 * PI 在上一句就可以去掉
   if (endOri - startOri > 3 * M_PI)
   {
      endOri -= 2 * M_PI;
   }
   else if (endOri - startOri < M_PI)
   {
      endOri += 2 * M_PI;
   }
   bool halfPassed = false;
   int count = cloudSize;
   PointType point;
   //存储每个scan中的点
   //通过计算pitch角度值将点划分到不同的“线”中。
   //代码坑，point.x=laserCloudIn.points[i].y；
   //做个迷惑的赋值。Pitch=atan(z/(x2+y2));和代码中只是形式的不同。
   std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
   for (int i = 0; i < cloudSize; i++)
   {
      laserCloudScans[laserCloudIn.points[i].data_n[2]].push_back(laserCloudIn.points[i]); //按照线存储
   }
   cloudSize = count;
   //pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
   for (int i = 0; i < N_SCANS; i++)
   {
      // cout << i << " scan time = " << laserCloudScans[i].points[0].data_n[0] + laserCloudScans[i].points[0].data_n[1]
      // << "--->" << laserCloudScans[i].points.back().data_n[0] + laserCloudScans[i].points.back().data_n[1] << endl;
      *laserCloud += laserCloudScans[i];
   }
   //曲率计算
   //因为是按照线的序列存储，因此接下来能够得到起始和终止的index；在这里滤除前五个和后五个。
   int scanCount = -1;
   for (int i = 5; i < cloudSize - 5; i++)
   {
      //后5+前5-10*当前坐标,相当于求10个向量的差值
      float dis, diffX, diffY, diffZ;
      if (curvThredhold > 0.2)
      {
         diffX = diffY = diffZ = 0;
         for (int k = 0; k < 11; k++)
         {
            dis = sqrt((laserCloud->points[i].x - laserCloud->points[i - 5 + k].x) * (laserCloud->points[i].x - laserCloud->points[i - 5 + k].x) + (laserCloud->points[i].y - laserCloud->points[i - 5 + k].y) * (laserCloud->points[i].y - laserCloud->points[i - 5 + k].y) + (laserCloud->points[i].z - laserCloud->points[i - 5 + k].z) * (laserCloud->points[i].z - laserCloud->points[i - 5 + k].z));
            if (dis != 0)
            {
               diffX += (laserCloud->points[i].x - laserCloud->points[i - 5 + k].x) / dis;
               diffY += (laserCloud->points[i].y - laserCloud->points[i - 5 + k].y) / dis;
               diffZ += (laserCloud->points[i].z - laserCloud->points[i - 5 + k].z) / dis;
            }
         }
      }
      //-----------------------------------------两种求曲率的分界线------------------------------------
      else
      {
         diffX =
             laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
         diffY =
             laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
         diffZ =
             laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;
      }
      //如果点在平面,向量就约等于0
      cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
      cloudSortInd[i] = i;
      cloudNeighborPicked[i] = 0;
      cloudLabel[i] = 0;
      if (int(laserCloud->points[i].data_n[2]) != scanCount)
      {
         scanCount = int(laserCloud->points[i].data_n[2]);

         if (scanCount > 0 && scanCount < N_SCANS)
         {
            scanStartInd[scanCount] = i + 5;
            scanEndInd[scanCount - 1] = i - 5;
         }
      }
   }
   scanStartInd[0] = 5;
   scanEndInd.back() = cloudSize - 5;

   //为了设置cloudNeighborPicked
   for (int i = 5; i < cloudSize - 6; i++)
   {
      float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
      float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
      float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
      float diff = diffX * diffX + diffY * diffY + diffZ * diffZ; //两点距离的平方

      if (diff > 0.1)
      {
         //离原点的距离
         float depth1 = laserCloud->points[i].data_n[3];
         //离原点的距离
         float depth2 = laserCloud->points[i + 1].data_n[3];

         //一段神奇的代码
         //求的是i和i+1两个向量夹角的一半的正弦值的两倍( 2sin(Θ/2))
         diffX = laserCloud->points[i + 1].x / depth2 - laserCloud->points[i].x / depth1;
         diffY = laserCloud->points[i + 1].y / depth2 - laserCloud->points[i].y / depth1;
         diffZ = laserCloud->points[i + 1].z / depth2 - laserCloud->points[i].z / depth1;
         float d2d = sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);
         if (depth1 > depth2)
         {
            //大概是5.74°
            if (d2d < 0.1)
            {
               cloudNeighborPicked[i - 5] = 1;
               cloudNeighborPicked[i - 4] = 1;
               cloudNeighborPicked[i - 3] = 1;
               cloudNeighborPicked[i - 2] = 1;
               cloudNeighborPicked[i - 1] = 1;
               cloudNeighborPicked[i] = 1;
            }
         }
         else
         {

            if (d2d < 0.1)
            {
               cloudNeighborPicked[i + 1] = 1;
               cloudNeighborPicked[i + 2] = 1;
               cloudNeighborPicked[i + 3] = 1;
               cloudNeighborPicked[i + 4] = 1;
               cloudNeighborPicked[i + 5] = 1;
               cloudNeighborPicked[i + 6] = 1;
            }
         }
      }

      float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
      float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
      float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
      float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;
      //距原点距离
      float dis = laserCloud->points[i].x * laserCloud->points[i].x + laserCloud->points[i].y * laserCloud->points[i].y + laserCloud->points[i].z * laserCloud->points[i].z;

      if (diff > 0.0002 * dis && diff2 > 0.0002 * dis)
      {
         cloudNeighborPicked[i] = 1;
      }
   }

   for (int i = 0; i < N_SCANS; i++)
   {
      pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
      for (int j = 0; j < 6; j++)
      {
         //#########################去除某些强度值不存在的数据############start
         if (scanStartInd[i] == 0)
            continue;
         //#########################去除某些强度值不存在的数据############End
         int sp = (scanStartInd[i] * (6 - j) + scanEndInd[i] * j) / 6;
         int ep = (scanStartInd[i] * (5 - j) + scanEndInd[i] * (j + 1)) / 6 - 1;

         for (int k = sp + 1; k <= ep; k++)
         {
            for (int l = k; l >= sp + 1; l--)
            {
               if (cloudCurvature[cloudSortInd[l]] < cloudCurvature[cloudSortInd[l - 1]])
               {
                  int temp = cloudSortInd[l - 1];
                  cloudSortInd[l - 1] = cloudSortInd[l];
                  cloudSortInd[l] = temp;
               }
            }
         }

         int largestPickedNum = 0;
         //检测特征点
         for (int k = ep; k >= sp; k--)
         {
            int ind = cloudSortInd[k];

            if (laserCloud->points[ind].data_n[3] < 1)
               continue;
            //相邻点未被选取，且曲率大于一个阈值
            if (cloudNeighborPicked[ind] == 0 &&
                cloudCurvature[ind] > curvThredhold &&
                (fabs(laserCloud->points[ind].x) > 0.3 ||
                 fabs(laserCloud->points[ind].y) > 0.3 ||
                 fabs(laserCloud->points[ind].z) > 0.3) &&
                fabs(laserCloud->points[ind].x) < 30 &&
                fabs(laserCloud->points[ind].y) < 30 &&
                fabs(laserCloud->points[ind].z) < 30)
            {

               largestPickedNum++;
               if (largestPickedNum <= 2)
               {
                  cloudLabel[ind] = 2;
                  cornerPointsSharp.push_back(laserCloud->points[ind]);
                  cornerPointsLessSharp.push_back(laserCloud->points[ind]);
               }
               else if (largestPickedNum <= 20)
               {
                  cloudLabel[ind] = 1;
                  cornerPointsLessSharp.push_back(laserCloud->points[ind]);
               }
               else
               {
                  break;
               }

               cloudNeighborPicked[ind] = 1;
               for (int l = 1; l <= 5; l++)
               {
                  float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                  float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                  float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                  if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                  {
                     break;
                  }

                  cloudNeighborPicked[ind + l] = 1;
               }
               for (int l = -1; l >= -5; l--)
               {
                  float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                  float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                  float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                  if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                  {
                     break;
                  }

                  cloudNeighborPicked[ind + l] = 1;
               }
            }
         }

         int smallestPickedNum = 0;
         for (int k = sp; k <= ep; k++)
         {
            int ind = cloudSortInd[k];
            if (cloudNeighborPicked[ind] == 0 &&
                cloudCurvature[ind] < curvThredhold &&
                (fabs(laserCloud->points[ind].x) > 0.3 ||
                 fabs(laserCloud->points[ind].y) > 0.3 ||
                 fabs(laserCloud->points[ind].z) > 0.3) &&
                fabs(laserCloud->points[ind].x) < 30 &&
                fabs(laserCloud->points[ind].y) < 30 &&
                fabs(laserCloud->points[ind].z) < 30)
            {

               cloudLabel[ind] = -1;
               surfPointsFlat.push_back(laserCloud->points[ind]);

               smallestPickedNum++;
               if (smallestPickedNum >= 4)
               {
                  break;
               }

               cloudNeighborPicked[ind] = 1;
               for (int l = 1; l <= 5; l++)
               {
                  float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                  float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                  float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                  if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                  {
                     break;
                  }

                  cloudNeighborPicked[ind + l] = 1;
               }
               for (int l = -1; l >= -5; l--)
               {
                  float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                  float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                  float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                  if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                  {
                     break;
                  }

                  cloudNeighborPicked[ind + l] = 1;
               }
            }
         }

         for (int k = sp; k <= ep; k++)
         {
            if (cloudLabel[k] <= 0)
            {
               surfPointsLessFlatScan->push_back(laserCloud->points[k]);
            }
         }
      }

      pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
      pcl::VoxelGrid<PointType> downSizeFilter;
      downSizeFilter.setInputCloud(surfPointsLessFlatScan);
      downSizeFilter.setLeafSize(0.05, 0.05, 0.05);
      downSizeFilter.filter(surfPointsLessFlatScanDS);

      surfPointsLessFlat += surfPointsLessFlatScanDS;
   }
}

void FeatureExtraction::getCornerSharp(pcl::PointCloud<PointType> &cornerPointsSharp)
{
   cornerPointsSharp.clear();
   cornerPointsSharp = this->cornerPointsSharp;
}

void FeatureExtraction::getCornerPointsLessSharp(pcl::PointCloud<PointType> &cornerPointsLessSharp)
{
   cornerPointsLessSharp.clear();
   cornerPointsLessSharp = this->cornerPointsLessSharp;
}

void FeatureExtraction::getSurfPointsFlat(pcl::PointCloud<PointType> &surfPointsFlat)
{
   surfPointsFlat.clear();
   surfPointsFlat = this->surfPointsFlat;
}

void FeatureExtraction::getSurfPointsLessFlat(pcl::PointCloud<PointType> &surfPointsLessFlat)
{
   surfPointsLessFlat.clear();
   surfPointsLessFlat = this->surfPointsLessFlat;
}

void FeatureExtraction::getFullres(pcl::PointCloud<PointType> &laserCloud)
{
   laserCloud.clear();
   laserCloud = *(this->laserCloud);
}

void FeatureExtraction::setNscans(int n)
{
   N_SCANS = n;
}
