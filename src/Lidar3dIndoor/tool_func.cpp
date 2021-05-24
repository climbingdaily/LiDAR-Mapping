#include "tool_func.h"

void ToolFunc::makeDir(string dir)
{
   // int dirEnd;
   // vector<std::string> dirList;
   // string temps;

   // while ((dirEnd = dir.find("/")) != -1)
   // {
   //    temps = dir.substr(0, dirEnd);
   //    if (temps != "" && temps != ".")
   //       dirList.push_back(temps);
   //    dir = dir.substr(dirEnd + 1, dir.size() - dirEnd - 1);
   // }
   // if (dir != "" && dir != ".")
   //    dirList.push_back(dir);

   // temps = dirList[0];
   makedir(dir);
   // for (vector<string>::iterator it = dirList.begin() + 1; it != dirList.end(); ++it)
   // {
   //    temps = temps + "/" + *it;
   //    makedir(temps);
   // }
}

void ToolFunc::makedir(string dir)
{
   int state;
   if ((state = access(dir.c_str(), 0)) == -1)
   { // 测试不存在

#ifdef WIN32
      state = mkdir(dir.c_str());
#endif
#ifdef linux
      state = mkdir(dir.c_str(), 0777);
      // cout << "Make dir" << dir  << endl;
#endif
      if (state != 0)
         cout << "Make " << dir << " error!" << endl;

      
   }
}

/**************************************************************************************
 * class TimeCounter:
 *      functions for programs runtime countering
 * ***********************************************************************************/

TimeCounter::TimeCounter()
{
}

TimeCounter::TimeCounter(string s)
{
   processName = s;
}

void TimeCounter::setProcessName(string s)
{
   processName = s;
}

void TimeCounter::begin()
{
   start = clock();
}

string TimeCounter::end()
{
   finish = clock();
   return "Process " + processName + " consumes " + toString<double>((double)(finish - start) / CLOCKS_PER_SEC) + "s.";
}

string TimeCounter::record(std::string processName)
{
   finish = clock();
   return processName + "\t"+ toString<double>((double)(finish - start) / CLOCKS_PER_SEC) + " s";
}

double TimeCounter::getFinishTime()
{
   finish = clock();
   return (double)finish / CLOCKS_PER_SEC;
}
/***************************************************
 * class IOToolFunc:
 *     io tool functions of files
 * *************************************************/
IOToolFunc::IOToolFunc()
{
   use_backbag_system = false;
}

void IOToolFunc::LoadCalibMatrix(Matrix4Type &cali_T)
{
   if (use_backbag_system)
   {
      ifstream matrix_file(matrix_path);
      if (!matrix_file)
      {
         cout << "Cannot read " << matrix_path << endl;
         matrix_file.close();
         exit(-1);
      }
      for (int i = 0; i < 4; ++i)
         for (int j = 0; j < 4; ++j)
            matrix_file >> cali_T(i, j);
      matrix_file.close();
   }
   else
      cali_T = Matrix4Type::Identity();
}

void IOToolFunc::setBackbagSystem(bool _ubgs)
{
   use_backbag_system = _ubgs;
}

void IOToolFunc::setCalibMatrixPath(string _cmp)
{
   matrix_path = _cmp;
}

void IOToolFunc::SetMatrixFilePath(string _matrix_path)
{
   matrix_path = _matrix_path;
}

void IOToolFunc::LoadMatrixFromFile(Matrix4Type &m)
{
   ifstream matrix_file(matrix_path);
   if (!matrix_file)
   {
      cout << "Cannot read " << matrix_path << endl;
      matrix_file.close();
      exit(-1);
   }
   for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
         matrix_file >> m(i, j);
   matrix_file.close();
}

void IOToolFunc::setSubMapSavePath(string _smsp)
{
   submap_save_path = _smsp;
}

void IOToolFunc::SaveSubMap(int id, PointCloud::Ptr cloud)
{
   pcl::io::savePCDFile(submap_save_path + "/" + toString<int>(id) + ".xyz", *cloud);
}

void IOToolFunc::SaveCloudByPcapName(string pcapName, PointCloud::Ptr cloud)
{
   int i = pcapName.find(".");
   std::string saveName = pcapName.substr(0, i) + ".xyz";
   pcl::io::savePCDFile(saveName, *cloud);
}

/******************************************************
 * class G2oToolFunc:
 *     tool functions of g2o data converting g2o data
 * ****************************************************/

Vector3Type G2oToolFunc::getTranslation(Matrix4Type &transform)
{
   Vector3Type t;
   for (int i = 0; i < 3; ++i)
      t(i) = transform(i, 3);
   return t;
}

double G2oToolFunc::getDistance(Matrix4Type pos1, Matrix4Type pos2)
{
   Vector3Type delta_pos = getTranslation(pos1) - getTranslation(pos2);
   RowVector3Type delta_pos_trans = delta_pos.transpose();
   return sqrt(delta_pos_trans * delta_pos);
}

Matrix3Type G2oToolFunc::getRotation(Matrix4Type &transform)
{
   Matrix3Type r;
   for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
         r(i, j) = transform(i, j);
   return r;
}

QuaternionType G2oToolFunc::rotation2Quaternion(Matrix3Type rotation)
{
   QuaternionType qua(rotation);
   return qua;
}

long long int G2oToolFunc::vertex2QT(ifstream &g2o_file, QuaternionType &q, Vector3Type &t)
{
   std::string label;
   Vertex v;
   if (!g2o_file)
   {
      cout << "g2o_file is not exist!" << endl;
   }
   g2o_file >> label;
   if (g2o_file.eof())
      return -1;

   if (label != "VERTEX_SE3:QUAT")
   {
      cout << "g2o file format error!" << endl;
      return -1;
   }

   g2o_file >> v.id >> v.pos(0) >> v.pos(1) >> v.pos(2) >> v.qua.x() >> v.qua.y() >> v.qua.z() >> v.qua.w();

   q = v.qua;
   for (int i = 0; i < 3; ++i)
      t(i, 3) = v.pos(i);

   return v.id;
}

Matrix4Type G2oToolFunc::rt2Transform(Matrix3Type &r, Vector3Type &t)
{
   Matrix4Type trans;
   for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
         trans(i, j) = r(i, j);
   for (int i = 0; i < 3; ++i)
      trans(i, 3) = t(i);
   for (int i = 0; i < 3; ++i)
      trans(3, i) = 0;
   trans(3, 3) = 1;
   return trans;
}

void G2oToolFunc::printInfoMatrix(ostream &g2o_file)
{
   g2o_file << " " << 10000 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 10000 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 10000 << " " << 0 << " " << 0 << " " << 0 << " " << 40000 << " " << 0 << " " << 0 << " " << 40000 << " " << 0 << " " << 40000;
}

string G2oToolFunc::informationMatrixInline()
{
   return " 10000 0 0 0 0 0 10000 0 0 0 0 10000 0 0 0 40000 0 0 40000 0 40000";
}

string G2oToolFunc::transform2Vertex(long long int id, Matrix4Type m)
{
   Vector3Type pos = getTranslation(m);
   QuaternionType qua = rotation2Quaternion(getRotation(m));
   std::string new_vertex = "VERTEX_SE3:QUAT " + toString<long long>(id) + " ";
   new_vertex += (toString<NumberType>(pos(0)) + " " + toString<NumberType>(pos(1)) + " " + toString<NumberType>(pos(2)) + " ");
   new_vertex += (toString<NumberType>(qua.x()) + " " + toString<NumberType>(qua.y()) + " " + toString<NumberType>(qua.z()) + " " + toString<NumberType>(qua.w()));
   return new_vertex;
}

string G2oToolFunc::transform2Edge(long long int from, long long int to, Matrix4Type m)
{
   Vector3Type pos = getTranslation(m);
   QuaternionType qua = rotation2Quaternion(getRotation(m));
   std::string edge = "EDGE_SE3:QUAT " + toString<long long>(from) + " " + toString<long long>(to) + " " + toString<double>(pos(0)) + " " + toString<double>(pos(1)) + " " + toString<double>(pos(2)) + " " + toString<double>(qua.x()) + " " + toString<double>(qua.y()) + " " + toString<double>(qua.z()) + " " + toString<double>(qua.w()) + informationMatrixInline();
   return edge;
}

long long int G2oToolFunc::vertex2Transform(ifstream &g2o_file, Matrix4Type &transform)
{
   std::string label;
   Vertex v;
   if (!g2o_file)
   {
      cout << "g2o_file is not exist!" << endl;
   }
   g2o_file >> label;
   if (g2o_file.eof())
      return -1;

   if (label != "VERTEX_SE3:QUAT")
   {
      cout << "g2o file format error!" << endl;
      // return -1;
      exit(0);
   }

   g2o_file >> v.id >> v.pos(0) >> v.pos(1) >> v.pos(2) >> v.qua.x() >> v.qua.y() >> v.qua.z() >> v.qua.w();
   Matrix3Type r = v.qua.toRotationMatrix();

   // Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
   // odom.translation() = Eigen::Vector3d(v.pos(0), v.pos(1), v.pos(2));
   // odom.linear() = v.qua.normalized().toRotationMatrix();
   // odom.matrix();

   for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
         transform(i, j) = r(i, j);
   for (int i = 0; i < 3; ++i)
      transform(i, 3) = v.pos(i);
   for (int i = 0; i < 3; ++i)
      transform(3, i) = 0;
   transform(3, 3) = 1;
   return v.id;
}

string G2oToolFunc::caculate_edge(long long int from, long long int to)
{
   Matrix4Type from_pos = track[from - startId];
   Matrix4Type to_pos = track[to - startId];
   Matrix4Type deltaH = from_pos.inverse() * to_pos;
   return transform2Edge(from, to, deltaH);
}

string G2oToolFunc::caculate_edge(long long int from, long long int to, Matrix4Type icp_delta)
{
   Matrix4Type from_pos = track[from - startId];
   Matrix4Type to_pos = track[to - startId];
   Matrix4Type corrected_to_pos = icp_delta * to_pos;
   Matrix4Type deltaH = from_pos.inverse() * corrected_to_pos;
   return transform2Edge(from, to, deltaH);
}

string G2oToolFunc::caculate_edge(long long from, long long to, PosSet &track)
{
   Matrix4Type from_pos, to_pos;
   Pos tmpPos;
   tmpPos.frame_id = from;
   tmpPos.transform = Matrix4Type::Identity();

   PosSet::iterator ps_it = track.find(tmpPos);
   if (ps_it != track.end())
   {
      from_pos = ps_it->transform;
   }

   tmpPos.frame_id = to;
   ps_it = track.find(tmpPos);
   if (ps_it != track.end())
   {
      to_pos = ps_it->transform;
   }

   Matrix4Type deltaH = from_pos.inverse() * to_pos;
   return transform2Edge(from, to, deltaH);
}

string G2oToolFunc::caculate_edge(long long int from, long long int to, PosSet &track, Matrix4Type icp_delta)
{
   Matrix4Type from_pos, to_pos;
   Pos tmpPos;
   tmpPos.frame_id = from;
   tmpPos.transform = Matrix4Type::Identity();

   PosSet::iterator ps_it = track.find(tmpPos);
   if (ps_it != track.end())
   {
      from_pos = ps_it->transform;
   }

   tmpPos.frame_id = to;
   ps_it = track.find(tmpPos);
   if (ps_it != track.end())
   {
      to_pos = ps_it->transform;
   }

   Matrix4Type corrected_to_pos = icp_delta * to_pos;
   Matrix4Type deltaH = from_pos.inverse() * corrected_to_pos;
   return transform2Edge(from, to, deltaH);
}
/**********************************************************************
 * class DebugToolFunc:
 *     tool functions for debuging
 * ********************************************************************/

string DebugToolFunc::getEularAngle(Matrix4Type &m)
{
   Matrix3Type r = g2o.getRotation(m);
   Vector3Type eulerAngle = r.eulerAngles(2, 1, 0);
   return toString<double>(eulerAngle(0) * 180 / M_PI) + " " + toString<double>(eulerAngle(1) * 180 / M_PI) + " " + toString<double>(eulerAngle(2) * 180 / M_PI);
}

void DebugToolFunc::ShowCloud(const PointCloud::Ptr cloud)
{
   p->removePointCloud("cloud");
   pcl::visualization::PointCloudColorHandlerGenericField<PointType> cloud_handle(cloud, "intensity");
   p->addPointCloud(cloud, cloud_handle, "cloud");
   p->spinOnce();
}

void DebugToolFunc::ShowCloud(const pcl::PointCloud<PointType>::Ptr cloud, long long int id)
{
   //pcl::visualization::PointCloudColorHandlerGenericField<PointType> cloud_handle (cloud, "simple");
   p->addPointCloud<PointType>(cloud, toString<int>(id).c_str());
   p->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, toString<int>(id).c_str());
   p->addCoordinateSystem(1.0);
   p->initCameraParameters();
   p->spinOnce(100);
}

void DebugToolFunc::ShowCloudWithCenter(const PointCloud::Ptr cloud, Matrix4Type &center, int mapid)
{
   p->removePointCloud("cloud");
   pcl::visualization::PointCloudColorHandlerGenericField<PointType> cloud_handle(cloud, "intensity");
   p->addPointCloud(cloud, cloud_handle, "cloud");
   Matrix3Type r = g2o.getRotation(center);

   p->addCube(g2o.getTranslation(center), g2o.rotation2Quaternion(r), 0.5, 0.5, 0.5, "cube" + toString<int>(mapid));
   p->spin();
}
