// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include "multiscanregistration.h"
#include "myFunction.h"

MultiScanMapper::MultiScanMapper(const float &lowerBound,
                                 const float &upperBound,
                                 const uint16_t &nScanRings)
    : _lowerBound(lowerBound),
      _upperBound(upperBound),
      _nScanRings(nScanRings),
      _factor((nScanRings - 1) / (upperBound - lowerBound))
{
}

void MultiScanMapper::set(const float &lowerBound,
                          const float &upperBound,
                          const uint16_t &nScanRings)
{
   _lowerBound = lowerBound;
   _upperBound = upperBound;
   _nScanRings = nScanRings;
   _factor = (nScanRings - 1) / (upperBound - lowerBound);
}

int MultiScanMapper::getRingForAngle(const float &angle)
{
   return int(((angle * 180 / M_PI) - _lowerBound) * _factor + 0.5);
}

MultiScanRegistration::MultiScanRegistration(const MultiScanMapper &scanMapper)
    : _scanMapper(scanMapper){};

bool MultiScanRegistration::setupLidarType(std::string lidarName)
{
   if (lidarName == "VLP-16")
   {
      _scanMapper = MultiScanMapper::Velodyne_VLP_16();
   }
   else if (lidarName == "HDL-32")
   {
      _scanMapper = MultiScanMapper::Velodyne_HDL_32();
   }
   else if (lidarName == "HDL-64E")
   {
      _scanMapper = MultiScanMapper::Velodyne_HDL_64E();
   }
   else if (lidarName == "VLP-32C")
   {
      _scanMapper = MultiScanMapper::Velodyne_VLP_32();
   }
   else
   {
      std::cerr << "Invalid lidar parameter: %s (only \"VLP-16\", \"HDL-32\" and \"HDL-64E\" are supported)", lidarName.c_str();
      return false;
   }
   return true;
}

void MultiScanRegistration::process(const pcl::PointCloud<pcl::PointXYZI> &laserCloudIn, PointCloud::Ptr laserCloudOut, const double timeStamp)
{
   size_t cloudSize = laserCloudIn.size();

   // determine scan start and end orientations
   float startOri = -std::atan2(laserCloudIn[0].y, laserCloudIn[0].x);
   float endOri = -std::atan2(laserCloudIn[cloudSize - 1].y,
                              laserCloudIn[cloudSize - 1].x) +
                  2 * float(M_PI);
   if (endOri - startOri > 3 * M_PI)
   {
      endOri -= 2 * M_PI;
   }
   else if (endOri - startOri < M_PI)
   {
      endOri += 2 * M_PI;
   }

   bool halfPassed = false;
   PointType point;
   // _laserCloudScans.resize(_scanMapper.getNumberOfScanRings());
   // clear all scanline points
   laserCloudOut->clear();
   // std::for_each(_laserCloudScans.begin(), _laserCloudScans.end(), [](auto &&v) { v.clear(); });

   // extract valid points from input cloud
   int ca, cb, cc;
   ca = cb = cc = 0;
   int scanid[64] = {0};
   for (int i = 0; i < cloudSize; i++)
   {
      point.x = laserCloudIn[i].x;
      point.y = laserCloudIn[i].y;
      point.z = laserCloudIn[i].z;

      // skip NaN and INF valued points
      if (!pcl_isfinite(point.x) ||
          !pcl_isfinite(point.y) ||
          !pcl_isfinite(point.z))
      {
         ++ca;
         continue;
      }

      // skip zero valued points
      if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001)
      {
         cb++;
         continue;
      }

      // calculate vertical point angle and scan ID
      float angle = std::atan(point.z / std::sqrt(point.x * point.x + point.y * point.y));
      int scanID = _scanMapper.getRingForAngle(angle);
      ++scanid[scanID + 10];

      if (scanID >= _scanMapper.getNumberOfScanRings() || scanID < 0)
      {
         cc++;
         continue;
      }

      // calculate horizontal point angle
      float ori = -std::atan2(point.y, point.x);
      if (!halfPassed)
      {
         if (ori < startOri - M_PI / 2)
         {
            ori += 2 * M_PI;
         }
         else if (ori > startOri + M_PI * 3 / 2)
         {
            ori -= 2 * M_PI;
         }

         if (ori - startOri > M_PI)
         {
            halfPassed = true;
         }
      }
      else
      {
         ori += 2 * M_PI;

         if (ori < endOri - M_PI * 3 / 2)
         {
            ori += 2 * M_PI;
         }
         else if (ori > endOri + M_PI / 2)
         {
            ori -= 2 * M_PI;
         }
      }

      // calculate relative scan time based on point orientation
      double relTime = scanPeriod * (ori - startOri) / (endOri - startOri) + timeStamp;
      point.intensity = laserCloudIn[i].intensity;
      point.data_n[0] = int(relTime);           //秒
      point.data_n[1] = relTime - point.data_n[0]; //微秒
      point.data_n[2] = scanID;
      point.data_n[3] = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
      laserCloudOut->points.push_back(point);

      //projectPointToStartOfSweep(point, relTime);

      // _laserCloudScans[scanID].push_back(point);
   }
   // cout << "ca = " << ca << endl;
   // cout << "cb = " << cb << endl;
   // cout << "cc = " << cc << endl;
   // for (int i = 0; i < 64; i++)
   // {
      // if (scanid[i] > 0)
         // cout << i-10 << " = "<< scanid[i] << endl;
   // }
   
   // ShowCloud(laserCloudOut);
   // processScanlines(scanTime, _laserCloudScans); //提取特征
   // publishResult();
}
