#include "build_map.h"
#include "LasOperator.h"
#include <pcl/io/pcd_io.h>
/**********************************************************
 * class BuildMap:
 *     abstract class, map builder
 * ********************************************************/
BuildMap::BuildMap()
{
   // io_tool.makeDir("./tmp/log");
   // std::string logPath;
   // CreateTempFile("build_map.log","./tmp/log/",logPath)
   // build_map_log.open(logPath.c_str());
   calib_flag = false;
   horizon_flag = false;
   showCloud = false;
   map_data = NULL;
   map_res = 0.1;
   frameOffset = 0;
   save_mapdata = false;
   output_cloud_path = exepath + "/output";
   printFlag = false;
   preID[0] = -1;
   preID[1] = -1;
}

BuildMap::~BuildMap()
{
   // build_map_log.close();
   // unlink(logPath.c_str());
}

void BuildMap::setCalibrationMatrixPath(string _matrix_filename)
{
   io_tool.setCalibMatrixPath(_matrix_filename);
}

void BuildMap::setCalibrationFlag(bool _calib_flag)
{
   calib_flag = _calib_flag;
   io_tool.setBackbagSystem(_calib_flag);
}

void BuildMap::setOutputPath(string _out_path)
{
   io_tool.makeDir(_out_path);
   output_cloud_path = _out_path;
}

void BuildMap::setShowCloud(bool _shcl_bmp)
{
   showCloud = _shcl_bmp;
   if (_shcl_bmp)
      debug.p = new pcl::visualization::PCLVisualizer("Building Map");
}

void BuildMap::setDistanceLimit(double _dis_limit)
{
   reader.SetDistanceControl(_dis_limit); // set effective distance
   reader2.SetDistanceControl(_dis_limit);
}

double BuildMap::errorTransform(Matrix4Type preFramePose, Matrix4Type curFramePose)
{
   Matrix4Type delta = curFramePose - preFramePose;
   double delta_sum = 0;
   for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
         delta_sum += delta(i, j);

   return delta_sum;
}

void BuildMap::setPointCloudReader2(string pcappath, string calipath2)
{
   reader2.setFrameGap(1);
   reader2.setPcapPath(pcappath);
   reader2.setCalibrationPath(calipath2);
   if (!reader2.OpenPcap())
   {
      printf("Cannot open pcap file!\n");
      exit(0);
      return;
   }
   // if (reader2.getTotalFrame() < 1000 && !TEST)
   // {
   //    printf("The pcap data is too small!\n");
   //    exit(0);
   //    return;
   // }
}

int BuildMap::Run()
{
   if (init() == FILE_READ_ERROR)
      return FILE_READ_ERROR;
   if (!reader.OpenPcap())
   {
      cout << "Error in reading pcap file, please check its path." << endl;
      return FILE_READ_ERROR;
   }

   if (!map_data)
   {
      cout << "You didn't alloc space for map_data." << endl;
      return ALLOC_ERROR;
   }
   OctreePtr octree(new OctreePointCloudSearch(map_res));
   PointCloud::Ptr cur(new PointCloud);
   map_octree = octree;
   curFrame = cur;
   MapManager mg(map_octree, map_data);
   if (save_mapdata)
   {
      consoleProgress(83);
   }
   if (calib_flag)
      io_tool.LoadCalibMatrix(calib_matrix);
   Matrix4Type preFramePose;
   long long preFrameId;
   bool _tmp_flag = true;
   int ii = 1;
   while (1)
   {
      if (readPose() == END_OF_TRACKFILE)
         break;

      if (_tmp_flag)
      {
         preFramePose = curFramePose;
         preFrameId = curFrameId;
         _tmp_flag = false;
      }
      if (loadFrame() == FILE_READ_ERROR)
      {
         return FILE_READ_ERROR;
      }
      if (horizon_flag) // horizon cloud
         curFramePose = horizon_matrix * curFramePose;

      double tmp_delta = errorTransform(preFramePose, curFramePose);
      //if(tmp_delta > 4.0)
      //continue;
      //cout << "Delta between " << curFrameId << " and " << preFrameId << " is " << tmp_delta << endl;
      pcl::transformPointCloud(*curFrame, *curFrame, curFramePose);
      preFramePose = curFramePose;
      preFrameId = curFrameId;

      mg.AddFrameToMap(curFrame);
      if (showCloud)
         debug.ShowCloud(map_data);
      //if(curFrameId > 5000 * ii)
      //{
      //  pcl::io::savePCDFile(output_cloud_path + "/map" + io_tool.toString(ii) +".xyz", *map_data);
      //  ii++;
      //  mg.UpdateMap(curFrame);
      //}
   }
   if (save_mapdata)
   {
      consoleProgress(94);
      if (curFrameId == -1)
         saveLasFile(output_cloud_path + "/map.las", map_data);
      else
         saveLasFile(output_cloud_path + "/map_endId" + io_tool.toString<long long>(curFrameId) + ".las", map_data);
      // if (curFrameId == -1)
      //     pcl::io::savePCDFile(output_cloud_path + "/map_full.xyz", *map_data);
      // else
      //     pcl::io::savePCDFile(output_cloud_path + "/map_endId" + io_tool.toString<long long>(curFrameId) + ".xyz", *map_data);
      consoleProgress(96);
   }

   return NORMAL_STATUS;
}

void BuildMap::setMapPointer(PointCloud::Ptr &map_input)
{
   map_data = map_input;
}

PointCloud::Ptr BuildMap::getMapPointer()
{
   return map_data;
}

void BuildMap::setMapResolution(double res)
{
   map_res = res;
}

double BuildMap::getMapResolution()
{
   return map_res;
}

int BuildMap::loadFrame()
{
   PointCloud::Ptr frame2(new PointCloud);
   while (1)
   {
      reader.frameNumber = curFrameId;
      while (reader.readPointCloud(curFrame) != END_OF_PCAPFILE) // load frame by curFrameId
      {
         if (calib_flag) // pre calibration
         {
            reader2.frameNumber = curFrameId + frameOffset;
            reader2.readPointCloud(frame2);
            //中间也可能掉帧，判断一下
            float frameTime = frame2->points.back().data_n[0] + frame2->points.back().data_n[1] - (frame2->points[0].data_n[0] + frame2->points[0].data_n[1]);
            if (frame2->points.size() < 8000 ||
                frameTime > 0.06
               //  || frameTime < 0.04
                )
            {
               cout << "Attention: 201可能存在掉帧或者数据不稳定情况!!!\n";
               cout << "202\tFrameID = " << curFrameId << endl;
               cout << "201\tFrameID = " << curFrameId + frameOffset << endl;
               goto OutMerge2Cloud;
            }
            int frameID2 = curFrameId + frameOffset;
            //通过时间来判断两帧的时间偏移
            double timeOffset = frame2->points[0].data_n[0] + frame2->points[0].data_n[1] - (curFrame->points[0].data_n[0] + curFrame->points[0].data_n[1]);
            frameTime = 0.05;
            //计算两个激光头之间的时间差，让时间差位于半帧的时间之内
            int _a = timeOffset / frameTime;
            _a += 1.9 * (timeOffset - _a * frameTime) / frameTime;
            _a = 0;
            // frameOffset -= _a;

            if (abs(_a) > 0.1)
            {
               if (preID[0] == -1 && preID[1] == -1)
               {
                  preID[0] = frameID2;
               }
               else if (preID[1] == -1)
               {
                  preID[1] = frameID2;
               }
               else if (frameID2 == preID[0])
               {
                  preID[0] = -1;
                  preID[1] = -1;
                  goto OutMerge2Cloud;
               }
               else
               {
                  preID[0] = preID[1];
                  preID[1] = frameID2;
               }
               cout << "=================Build Map FrameOffset=======================\n"
                    << "FrameID = " << curFrameId << "\t"
                    << "Offset = " << -_a << "\t"
                    << "SumFrameOffset = " << frameOffset << "\t "
                    << "Timeoffset  = " << timeOffset << endl
                    << "=============================================================\n";
               if (abs(_a) > 1)
               {
                  cout << "\nAttention: Frame2's timestamp is changing more than 0.10s!!!\n";
                  goto OutMerge2Cloud;
               }
               reader.frameNumber = curFrameId;
               continue;
            }
            pcl::transformPointCloud<PointType>(*frame2, *frame2, calib_matrix);
            *curFrame += *frame2;
         }
      OutMerge2Cloud:
         if (reader.frameNumber >= curFrameId)
            return CONTINUE_READ;
      }

      if (curFrameId > reader.frameNumber)
      { // curFrameId out of boundary
         cout << "BuildMap::loadFrame warning: curFrameId in track.txt is too bigger than frameNumber of pcapFile." << endl;
         return FILE_READ_ERROR;
      }

      // curFrameId in boundary, so open pcap again to read
      reader.frameNumber = -1;
      if (!reader.OpenPcap())
      {
         cout << "BuildMap::loadFrame warning: Error in reading pcap file, please check its path." << endl;
         return FILE_READ_ERROR;
      }
   }
   return 0;
}

void BuildMap::setPcapFilePath(std::string _pcap_path)
{
   int tmp_i = _pcap_path.rfind("/");
   input_pcap = _pcap_path.substr(tmp_i + 1, _pcap_path.length() - tmp_i - 6);
   reader.setPcapPath(_pcap_path);
}

void BuildMap::setCalibPath(string _calib_path)
{
   reader.setCalibrationPath(_calib_path);
}

/**********************************************************************
 * class BuildMapfromTrajectory
 *     build map from trajectory
 * *******************************************************************/

BuildMapfromTrajectory::BuildMapfromTrajectory()
{
}

BuildMapfromTrajectory::~BuildMapfromTrajectory()
{
}

int BuildMapfromTrajectory::init()
{
}

void BuildMapfromTrajectory::SetTrajectory(Trajectory &t)
{
   traj_it = t.begin();
   traj_end = t.end();
}

void BuildMapfromTrajectory::SetTrajectory(PosSet &ps, Trajectory &t)
{
   t.clear();
   Pos tmp_pos;
   for (PosSet::iterator ps_it = ps.begin(); ps_it != ps.end(); ++ps_it)
   {
      tmp_pos = *ps_it;
      t.push_back(tmp_pos);
   }
   traj_it = t.begin();
   traj_end = t.end();
}

int BuildMapfromTrajectory::readPose()
{
   curFrameId = traj_it->frame_id;
   curFramePose = traj_it->transform;
   traj_it++;
   return traj_it == traj_end ? END_OF_TRACKFILE : CONTINUE_READ;
}

/**********************************************************************
 * class BuildMapfromG2oVertex
 *     build map from g2o vertex
 * *******************************************************************/
#if 0
BuildMapfromG2oVertex::BuildMapfromG2oVertex()
{

}

BuildMapfromG2oVertex::~BuildMapfromG2oVertex()
{

}
#endif
void BuildMapfromG2oVertex::SetTrackFilename(string s)
{
   trackFileName = s;
}

int BuildMapfromG2oVertex::init()
{
   trackReader.open(trackFileName);
   if (!trackReader)
   {
      cout << trackFileName << " is not exist!" << endl;
      return FILE_READ_ERROR;
   }
   return NORMAL_STATUS;
}

int BuildMapfromG2oVertex::readPose()
{
   curFrameId = g2o.vertex2Transform(trackReader, curFramePose);
   return trackReader.eof() ? END_OF_TRACKFILE : CONTINUE_READ;
}

/**********************************************************************
 * class BuildMapfromG2oVertex
 *     build map from g2o vertex
 * *******************************************************************/

BuildMapfromTMatrix::BuildMapfromTMatrix()
{
}

BuildMapfromTMatrix::~BuildMapfromTMatrix()
{
}

void BuildMapfromTMatrix::SetTrackFilename(string s)
{
   trackFileName = s;
}

int BuildMapfromTMatrix::init()
{
   trackReader.open(trackFileName);
   if (!trackReader)
   {
      cout << trackFileName << " is not exist!" << endl;
      return FILE_READ_ERROR;
   }
   return NORMAL_STATUS;
}

int BuildMapfromTMatrix::readPose()
{
   trackReader >> curFrameId;
   return trackReader.eof() ? END_OF_TRACKFILE : loadPose(trackReader, curFramePose);
}

int BuildMapfromTMatrix::loadPose(istream &in, Matrix4Type &m)
{
   for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
         in >> m(i, j);
   return CONTINUE_READ;
}

/*******************************************************
 * class BuildMapWithMultiLayers:
 *     request from xueZhang
 * *****************************************************/

BuildMapWithMultiLayers::BuildMapWithMultiLayers()
{
}

BuildMapWithMultiLayers::~BuildMapWithMultiLayers()
{
}

void BuildMapWithMultiLayers::SetMatrixFilename(string _matrix_filename)
{
   matrix_filename = _matrix_filename;
}

void BuildMapWithMultiLayers::SetBeginLayerId(long long int _begin_id)
{
   beginLayerId = _begin_id;
}

int BuildMapWithMultiLayers::Run()
{
   if (init() == FILE_READ_ERROR)
      return FILE_READ_ERROR;
   if (!reader.OpenPcap())
   {
      cout << "Error in reading pcap file, please check its path." << endl;
      return FILE_READ_ERROR;
   }
   if (!map_data)
   {
      cout << "You didn't alloc space for map_data." << endl;
      return ALLOC_ERROR;
   }
   OctreePtr octree(new OctreePointCloudSearch(map_res));
   PointCloud::Ptr cur(new PointCloud);
   map_octree = octree;
   curFrame = cur;
   MapManager mg(map_octree, map_data);
   io_tool.setCalibMatrixPath(matrix_filename);
   io_tool.setBackbagSystem(true);
   io_tool.LoadCalibMatrix(matrix);

   while (1)
   {
      if (readPose() == END_OF_TRACKFILE)
         break;
      if (loadFrame() == END_OF_PCAPFILE)
         break;

      if (curFrameId > beginLayerId)
         curFramePose = matrix * curFramePose;
      pcl::transformPointCloud(*curFrame, *curFrame, curFramePose);
      mg.AddFrameToMap(curFrame);

      if (showCloud)
         debug.ShowCloud(map_data);
   }
   return NORMAL_STATUS;
}
