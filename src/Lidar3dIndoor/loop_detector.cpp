#include "loop_detector.h"
#include "submap.h"

/****************************************************************************************
 * class LoopDetector:
 *     loop detector, use class buildmap and gicpEnd
 * *************************************************************************************/

LoopDetector::LoopDetector()
{
   //g2o.makeDir("./output/submap");
   //g2o.makeDir("./output/trajectory");
   showCloud = false;
   showTraj = false;
   io_tool.LoadCalibMatrix(cali_T);
}

LoopDetector::~LoopDetector()
{
}

int LoopDetector::Run()
{

   DetectLoop2();
   caculateAndSaveNewEdges();
}

void LoopDetector::ShowDetectedMap(bool _show_flag)
{
   showCloud = _show_flag;
   if (showCloud)
      debug.p = new pcl::visualization::PCLVisualizer("Clouds of Detected Loop");
}

void LoopDetector::SetDetectMapRes(double _detect_map_res)
{
   detect_res = _detect_map_res;
}

void LoopDetector::SetCalibFilePath(string _cali_file_path)
{
   calibFile = _cali_file_path;
}

void LoopDetector::SetPcapFilePath(string _pcap_file_path)
{
   pcapFile = _pcap_file_path;
}

/*
void LoopDetector::loadFrontResult()
{
    frontResult.open( front_result_path.c_str() );
    if(!frontResult){
    	cout << "LoopDetector::loadFrontResult() : Cannot open front result!" << endl;
    	exit(-1);
    }
}
*/
void LoopDetector::addToTrajectory(long long int frame_id, Matrix4Type T, Trajectory &_traj)
{
   Pos temp_pos_;
   temp_pos_.transform = T;
   temp_pos_.frame_id = frame_id;
   _traj.push_back(temp_pos_);
}

void LoopDetector::SetFrontResultPath(string _front_path)
{
   front_result_path = _front_path;
}

void LoopDetector::SetDetectDistance(double _dis)
{
   detect_distance = _dis;
}

void LoopDetector::openFrontResult()
{
   frontResult.open(front_result_path.c_str());
   if (!frontResult)
   {
      cout << "Cannot open front result!" << endl;
      exit(-1);
   }
}

void LoopDetector::loadTrajectory()
{
   origin_trajectory.clear();
   bool _flag = true, _flag2 = true;
   long long preFrameId;
   while (1)
   {
      curFrameId = g2o.vertex2Transform(frontResult, curPose);
      if (_flag)
      {
         preFrameId = curFrameId;
         _flag = false;
         continue;
      }
      if (_flag2)
      {
         loam_step = curFrameId - preFrameId;
         _flag2 = false;
      }

      if (frontResult.eof())
         break;
      addToTrajectory(curFrameId, curPose, origin_trajectory);
   }
}

bool LoopDetector::findPoseNeighbors(long long curPoseId, SubMap2 &_align_submap)
{
   bool detect_flag = false;
   for (long long i = curPoseId + 500 / loam_step; i < origin_trajectory.size(); ++i)
   {
      double dis = g2o.getDistance(origin_trajectory[curPoseId].transform, origin_trajectory[i].transform);
      if (dis < detect_distance)
      {
         detect_flag = true;
         _align_submap.traj.insert(origin_trajectory[i]);
         loop_flag[i] = true;
      }
   }
   return detect_flag;
}

void LoopDetector::DetectLoop2()
{
   openFrontResult();
   loadTrajectory();
   long long poseNumber = origin_trajectory.size();
   loop_flag = new bool[poseNumber];
   consoleProgress(75);
   // initialize
   for (int i = 0; i < poseNumber; ++i)
      loop_flag[i] = false;
   curPose = origin_trajectory[0].transform;
   curFrameId = origin_trajectory[0].frame_id;

   PointCloud::Ptr refer_cloud(new PointCloud);
   PointCloud::Ptr align_cloud(new PointCloud);
   PointCloud::Ptr temp_map_(new PointCloud);
   OctreePointCloudSearch::Ptr octree(new OctreePointCloudSearch(detect_res));
   MapManager mg(octree, temp_map_);
   double err = 999;
   Matrix4Type T;
   Trajectory tmp_traj;

   buildMap.setMapResolution(detect_res);
   buildMap.setCalibPath(calibFile);
   buildMap.setPcapFilePath(pcapFile);
   //buildMap.setOutputPath("tmp/submap");

   if (showTraj)
      debug.p = new pcl::visualization::PCLVisualizer("Detect Loop Trajectory");
   // find  point[i]'s  neighbors
   for (long long i = 0; i < poseNumber; ++i)
   {
      if (!loop_flag[i])
      {
         SubMap2 refer_submap, aligned_submap;
         //Trajectory refer_traj, align_traj;
         while (findPoseNeighbors(i, aligned_submap) == true)
         {
            refer_submap.traj.insert(origin_trajectory[i]);
            ++i;
            //curPose = origin_trajectory[i].transform;
            //curFrameId = origin_trajectory[i].frame_id;
         }
         if (refer_submap.traj.size() > 0 && aligned_submap.traj.size() > 0)
         {
            // load refer cloud
            refer_cloud->points.clear();
            buildMap.setMapPointer(refer_cloud);
            buildMap.SetTrajectory(refer_submap.traj, tmp_traj);
            buildMap.Run();
            mg.UpdateMap(refer_cloud); // use octree to search neighbors

            // load align cloud
            align_cloud->points.clear();
            buildMap.setMapPointer(align_cloud);
            buildMap.SetTrajectory(aligned_submap.traj, tmp_traj);
            buildMap.Run();
            err = gicp.UseNeighborsAlign(octree, align_cloud, T);
            if (err < icp_error_threshold)
            {
               cout << "\nLoop is detected!";
               cout << "\nDetected frame = " << refer_submap.GetCenterPosId() << " and " << aligned_submap.GetCenterPosId() << endl;
               if (TEST)
                  cout << "\nRegistration err = " << err << endl;
               addConstraint(refer_submap.GetCenterPosId(), aligned_submap.GetCenterPosId(), T, err);
            }
#if DEBUG
            //   cout << "Detect pose" << refer_submap.GetCenterPosId() << " and pose" << aligned_submap.GetCenterPosId() << endl;
#if 0

		refer_submap.ShowTraj(&debug);
	        aligned_submap.ShowTraj(&debug);

#endif
            //   cout << "  transform:\n" << T << endl;
            //   cout << "  icp_error: " << err << endl << "------------------------------------------------" << endl;
            //   refer_submap.SaveTraj("tmp/trajectory_refer");
            //   aligned_submap.SaveTraj("tmp/trajectory_align");
#endif
         }
      }
      consoleProgress(75 + int(1.0 * i / poseNumber * 5));
   }
}

void LoopDetector::SetOutputFile(string _out_file)
{
   result.open(_out_file.c_str());
   if (!result)
   {
      std::cout << "cannot create file " << _out_file << std::endl;
      exit(-1);
   }
}

void LoopDetector::caculateAndSaveNewEdges()
{

   g2o.startId = origin_trajectory.begin()->frame_id;
   long long endId = origin_trajectory.rbegin()->frame_id;

   PosSet track;
   Pos tmpPos;

   for (Trajectory::iterator it = origin_trajectory.begin(); it != origin_trajectory.end(); ++it)
   {
      tmpPos.frame_id = it->frame_id;
      tmpPos.transform = it->transform;
      track.insert(tmpPos);
      result << g2o.transform2Vertex(it->frame_id, it->transform) << endl;
   }
   consoleProgress(81);
   // calculate old edges
   long long ii = 0;
   for (PosSet::iterator ps_it = track.begin(); ps_it != track.end() && ps_it->frame_id < endId; ++ps_it)
   {
      result << g2o.caculate_edge(ps_it->frame_id, ps_it->frame_id + loam_step, track) << endl;
      ii++;
   }
   consoleProgress(82);
   // calculate new edges
   for (int i = 0; i < constraints.size(); ++i)
   {
      result << g2o.caculate_edge(constraints[i].reference_frameid, constraints[i].aligned_frameid, track, constraints[i].icp_delta_T) << endl;
   }
   result.close();
   consoleProgress(83);
}

void LoopDetector::addConstraint(long long int refer_, long long int align_, Matrix4Type T, double error)
{
   Constraint temp_cons_;
   temp_cons_.reference_frameid = refer_;
   temp_cons_.aligned_frameid = align_;
   temp_cons_.icp_delta_T = T;
   temp_cons_.icp_error = error;
   constraints.push_back(temp_cons_);
}
