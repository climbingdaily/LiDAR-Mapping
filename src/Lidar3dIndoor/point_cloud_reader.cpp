#include "point_cloud_reader.h"

PointCloudReader::PointCloudReader()
{
   frameNumber = 0;
   scanGap = 0;
   frameGap = 1;
   distanceControl = 40.0;
   voxel_grid_leafsize = 0.1;
   sparseThreshold = 100;
   isReadSuc = false;
}

void PointCloudReader::setCalibrationPath(std::string ca)
{
   calibrationPath = ca; 
   //根据输入的xml文件地址，设置Lidar类型
   // printf("%s\n", ca);
   int length = ca.length();
   if (ca.substr(length-10,length-1).compare("HDL-32.xml") == 0)
      cloudFromeScan.setupLidarType("HDL-32");
   else if (ca.substr(length-10,length-1).compare("VLP-16.xml") == 0)
      cloudFromeScan.setupLidarType("VLP-16");
   else if (ca.substr(length-11,length-1).compare("VLP-32c.xml") == 0)
      cloudFromeScan.setupLidarType("VLP-32C");
   else if (ca.substr(length-10,length-1).compare("HDL-64E.xml") == 0)
      cloudFromeScan.setupLidarType("HDL-64E");
   else
      cloudFromeScan.setupLidarType("VLP-16"); //默认16线   
}

PointCloudReader::PointCloudReader(std::string pc, std::string cp)
{
   fileNamePcap = pc;
   scanGap = 0;
   frameGap = 1;
   calibrationPath = cp;
   isReadSuc = false;
   frameNumber = 0;
   distanceControl = 40.0;
   voxel_grid_leafsize = 0.1;
   sparseThreshold = 100;
   OpenPcap();
}

bool compare_file(const std::string& left,const std::string& right){

   std::string lf,rf;
   lf = left.substr(0, left.size());
   rf = right.substr(0, right.size());
   replace(lf.begin(), lf.end(), '_', '.');
   replace(rf.begin(), rf.end(), '_', '.');
   std::istringstream isl(lf);
   std::istringstream isr(rf);
   double t1,t2;
   isl >> t1;
   isr >> t2;
   return t1 < t2; //升序排列
}

/*
* 根据文件夹名字，自动读入pcd的文件名，转换成时间戳，按照升序排列
*/
void PointCloudReader::setPcapPath(std::string _filePath)
{
   int length = _filePath.length();
   std::string sstr = _filePath.substr(length - 5, length - 1); 
   if (sstr.compare(".pcap") == 0)
   {
      fileNamePcap = _filePath;
      isPcap = true;
      // printf("This is a pcap!\n");
      // exit(0);
   }
   else
   {
      // printf("This is PCD files!\n");
      isPcap = false;
      filePath = _filePath;
      getFiles(filePath, files);
      std::sort(files.begin(), files.end(),compare_file);
      for (size_t i = 0; i < files.size(); i++)
      {
         // std::istringstream iss(files[i].substr(0, files[i].size() - 6)); //位数太多了, 读取文件名的时间戳
         std::string temp = files[i];
         replace(temp.begin(), temp.end(), '_', '.');
         std::istringstream iss(temp);
         double timeStamp;
         iss >> timeStamp;
         timeStamps.push_back(timeStamp);
      }
   }
}

bool PointCloudReader::OpenPcap()
{
   //printf("OpenPcap pcap File=%s\n", fileNamePcap.c_str());
   //printf("OpenPcap calib File=%s\n", calibrationPath.c_str());
   if (isPcap)
   {
      if (fileNamePcap == "" || calibrationPath == "")
         return false;
      isReadSuc = reader.open(fileNamePcap, calibrationPath);
   }
   isReadSuc = true;
   return isReadSuc;
}

void PointCloudReader::VoxelGrid(PointCloud::Ptr cloud)
{
   PointCloud::Ptr cloud_filtered(new PointCloud);
   grid.setLeafSize(voxel_grid_leafsize, voxel_grid_leafsize, voxel_grid_leafsize);
   grid.setInputCloud(cloud);
   grid.filter(*cloud_filtered);
   *cloud = *cloud_filtered;

   // remove NAN points from the cloud
   std::vector<int> indices;
   PointCloud::Ptr cloud_remove_nan(new PointCloud);
   pcl::removeNaNFromPointCloud(*cloud, *cloud_remove_nan, indices);
   *cloud = *cloud_remove_nan;
}

void PointCloudReader::PointCloudFilter(pcl::PointCloud<PointType>::Ptr cloudin, pcl::PointCloud<PointType>::Ptr cloudout, float Leafsize)
{
   cloudout->clear();
   grid.setLeafSize(Leafsize, Leafsize, Leafsize);
   grid.setInputCloud(cloudin);
   grid.filter(*cloudout);
   std::vector<int> indices;
   pcl::removeNaNFromPointCloud(*cloudout, *cloudout, indices);
}

/**
 * @param[in] 点云指针
 * @param[out] 为真则返回true，读取失败则返回-1;
 */
int scale = 1;
int PointCloudReader::readPointCloud(PointCloud::Ptr frame_point_cloud, bool record)
{
   frame_point_cloud->clear();

   if (isPcap)
   {
      long long count = 0;
      PointType pt;
      if (!reader.capture(frame, frameNumber, startTimeStamp, gpsFile, record))
      {
         //std::cout << "End of Frame!" << std::endl;
         return -1;
      }

      double reletiveTime = 0;
      for (int n = 0; n < frame.numLine; n++)
      {
         reletiveTime += frame.lines[0].pTimestamp[0]/1e6;
      }

      reletiveTime /= frame.numLine;

      for (int n = 0; n < frame.numLine; n++)
      {
         // if (n % 2 == 1)
         //    continue;
         for (int i = 0; i < frame.lines[n].num; i++)
         {
            pt.x = (float)frame.lines[n].pPosition[i].x * scale;
            pt.y = (float)frame.lines[n].pPosition[i].y * scale;
            pt.z = (float)frame.lines[n].pPosition[i].z * scale;
            float dist = frame.lines[n].pDistance[i] * scale;
            // dist = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if (distanceControl && (dist > distanceControl))
            {
               count++;
               continue;
            }
            pt.intensity = frame.lines[n].pIntensity[i] + 0.1f;
            double timestamp = frame.lines[n].pTimestamp[i] / 1e6 - reletiveTime + startTimeStamp;
            pt.data_n[0] = int(timestamp);           //秒
            pt.data_n[1] = timestamp - pt.data_n[0]; //微秒
            pt.data_n[2] = n + scanGap;
            pt.data_n[3] = dist;
            frame_point_cloud->points.push_back(pt);
            
            // pt.data_n[2] = n + scanGap + 1;
            // cloud->points.push_back(pt);

         }
      }
      float theta = M_PI / 2.0;
      Eigen::Matrix4f Tx = Eigen::Matrix4f::Identity(); //绕X轴转90°
      Tx(1, 1) = cos(theta);
      Tx(1, 2) = -sin(theta);
      Tx(2, 1) = sin(theta);
      Tx(2, 2) = cos(theta);

      Eigen::Matrix4f Ty = Eigen::Matrix4f::Identity(); //绕X轴转90°
      Ty(0, 0) = cos(theta);
      Ty(0, 2) = sin(theta);
      Ty(2, 0) = -sin(theta);
      Ty(2, 2) = cos(theta);

      Eigen::Matrix4f Tz = Eigen::Matrix4f::Identity(); //绕z轴转90°
      Tz(0, 0) = cos(theta);
      Tz(0, 1) = -sin(theta);
      Tz(1, 0) = sin(theta);
      Tz(1, 1) = cos(theta);
      // pcl::transformPointCloud(*frame_point_cloud, *frame_point_cloud, Tx * Tx);        
   }
   else
   {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
      if (frameNumber < files.size())
      {
         // std::cout << "filesize " << files.size() << std::endl;
         // std::cout << timeStamps[frameNumber]  << std::endl;
         // getCloudFromPcd(cloud, filePath + '/' + files[frameNumber]);
         getCloudFromeXYZ(cloud, filePath + '/' + files[frameNumber]);
      }
      else
      {
         return -1;
      }
      // cloudFromeScan.process(*cloud, frame_point_cloud, timeStamps[frameNumber]); //处理pcd
      cloudFromeScan.process(*cloud, frame_point_cloud, (frameNumber + 10) * 0.05); //XYZ
   }
   
   frame_point_cloud->height = 1;
   frame_point_cloud->width = frame_point_cloud->points.size();
   frame_point_cloud->is_dense = true;
   if (frameGap <= 0)
      frameNumber++;
   else
      frameNumber += frameGap;
   //if(distanceControl)
   //std::cout << "remove " << count << " points in " << frameNumber << " frame." << std::endl;

   return 0;
}

int PointCloudReader::LoadFirstFrame(PointCloud::Ptr frame)
{
   frameNumber = 0;
   if (readPointCloud(frame) == -1)
      return -1;
   // Less than sparseThreshold
   while (frame->points.size() < sparseThreshold)
   {
      if (readPointCloud(frame) == -1)
         return -1;
   }
   VoxelGrid(frame);
   return 0;
}

void PointCloudReader::setScansGap(int scanGap)
{
   this->scanGap = scanGap;
}

void PointCloudReader::setFrameGap(int frameGap)
{
   if (frameGap <= 0)
      printf("\nframeGap Error,min is 0!\n");
   this->frameGap = frameGap;
}

double PointCloudReader::getTimpStamp(int frameid)
{
   //仅仅用于返回
   // if (!isPcap)
   return timeStamps[frameid];
}