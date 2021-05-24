#include "make_horizonal.h"
#include "myFunction.h"

#define MAKEHORIZONAL_CORNERPOINT 24
#define MAKEHORIZONAL_GETDATA_NUMBER 300
void MakeHorizonal::ReadXYZfromVertex(PointCloud::Ptr cloud)
{

   input_file.open(input_path.c_str());

   if (cloud->points.size() > 0)
      cloud->clear();
   PointCloud::Ptr allPoints(new PointCloud);
   long long _tmp;
   PointType pt;
   std::string label, _tmp_s;

   //取所有轨迹点的数据
   while (true)
   {
      input_file >> label;
      if (input_file.eof())
         break;
      if (label != "VERTEX_SE3:QUAT")
      {
         std::cout << "Error in track file format!" << std::endl;
         exit(-1);
      }
      input_file >> _tmp >> pt.x >> pt.y >> pt.z;
      allPoints->points.push_back(pt);

      getline(input_file, _tmp_s); // read residual data
   }
   if (allPoints->points.size() <= 0)
   {
      cout << "Error in make_horizonal: input track_file is empty!" << endl;
      // exit(-1);
   }
   std::vector<int> index;
   int count = 0;
   //根据曲率取拐角

   double x, y, z;
   int gap;
   bool awayFromStart = false;
   x = allPoints->points[5].x;
   y = allPoints->points[5].y;
   z = allPoints->points[5].z;
   for (int i = 5; i < allPoints->points.size() - 5; i += (100 - frameGap) % 4)
   {
      if (!awayFromStart &&
          (allPoints->points[i].x - x) * (allPoints->points[i].x - x) +
                  (allPoints->points[i].y - y) * (allPoints->points[i].y - y) +
                  (allPoints->points[i].z - z) * (allPoints->points[i].z - z) <
              3)
      {
         continue;
      }
      awayFromStart = true;
      float cur = computeCurveture(allPoints, i);
      if (cur > 0.05)
      {
         count++;
      }
      else
      {
         gap++;
         if (gap > 1)
         {
            count = 0;
            gap = 0;
         }
      }
      if (count > MAKEHORIZONAL_CORNERPOINT)
      {
         index.push_back(i - count / 2);
         count = 0;
         if (index.size() >= 2)
            break;
      }
   }

   int iIndex = 0;
   //找不到拐角程序结束
   if (index.size() == 0)
   {
      // std::cout << "\n可能不能正确拟合出地面！" << std::endl;
      // std::cout << "(make_horizonal)Error in finding the corner, which could cause an error in finding the horizon!" << std::endl;
      //exit(-1);
      iIndex = MAKEHORIZONAL_GETDATA_NUMBER; //上面的话可以测试时注释掉来看结果
   }
   //取第2个拐角
   else if (index.size() > 1)
      iIndex = index[1];
   else
      //如果只有一个拐角
      iIndex = index[0];

   //如果序号少于两边各获取的数量则从最初开始取
   if (iIndex < MAKEHORIZONAL_GETDATA_NUMBER)
   {
      iIndex = MAKEHORIZONAL_GETDATA_NUMBER;
   }

   for (int i = iIndex - MAKEHORIZONAL_GETDATA_NUMBER;
        i < allPoints->points.size() - 10 && i < iIndex + MAKEHORIZONAL_GETDATA_NUMBER;
        i++)
   {
      cloud->push_back(allPoints->points[i]);
   }
   cloud->height = 1;
   cloud->width = cloud->points.size();
   cloud->is_dense = false;

   input_file.close();
}

void MakeHorizonal::ReadXYZfromVertex(PointCloud::Ptr cloud, double startDistance, double endDistance)
{
   input_file.open(input_path.c_str());

   if (cloud->points.size() > 0)
      cloud->clear();
   PointType pt;
   std::string label, _tmp_s;
   long long _tmp;
   double distance = 0;
   PointType beforePt;
   bool isnotFrist = false;
   double dx, dy, dz;
   while (!input_file.eof() && distance < endDistance)
   {
      input_file >> label;
      if (label != "VERTEX_SE3:QUAT")
      {
         std::cout << "Error in track file format!" << std::endl;
         exit(-1);
      }
      input_file >> _tmp >> pt.x >> pt.y >> pt.z;

      if (isnotFrist)
      {
         dx = beforePt.x - pt.x;
         dy = beforePt.y - pt.y;
         dz = beforePt.z - pt.z;
         distance += sqrt(dx * dx + dy * dy + dz * dz);
         //printf("distance=%lf\n",distance);
      }
      else
      {
         isnotFrist = true;
      }

      getline(input_file, _tmp_s); // read residual data
      beforePt = pt;

      if (distance > startDistance)
         cloud->points.push_back(pt);
   }
   cloud->height = 1;
   cloud->width = cloud->points.size();
   cloud->is_dense = false;

   input_file.close();
}

void MakeHorizonal::ReadXYZfromVertex(PointCloud::Ptr cloud, long long int startId)
{
   input_file.open(input_path.c_str());

   if (cloud->points.size() > 0)

      cloud->clear();
   PointType pt;
   std::string label, _tmp_s;
   long long _tmp;
   int endFrame = int(total_Frame / 3) + 500;
   while (cloud->points.size() < endFrame)
   {
      input_file >> label;
      if (input_file.eof())
         break;
      if (label != "VERTEX_SE3:QUAT")
      {
         std::cout << "Error in track file format!" << std::endl;
         exit(-1);
      }
      input_file >> _tmp >> pt.x >> pt.y >> pt.z;
      getline(input_file, _tmp_s); // read residual data
      if (_tmp < startId)
         continue;

      cloud->points.push_back(pt);
   }
   cloud->height = 1;
   cloud->width = cloud->points.size();
   cloud->is_dense = false;

   input_file.close();
}

Matrix4Type MakeHorizonal::calculateMatrix(double a, double b, double c)
{
   double r[3][3];

   r[0][0] = a * a - b * b - c * c;
   r[0][1] = 2 * a * b;
   r[0][2] = 2 * a * c;
   r[1][0] = 2 * a * b;
   r[1][1] = b * b - a * a - c * c;
   r[1][2] = 2 * b * c;
   r[2][0] = 2 * a * c;
   r[2][1] = 2 * b * c;
   r[2][2] = c * c - a * a - b * b;

   double t[4][4];
   for (int i = 0; i < 3; ++i)
   {
      for (int j = 0; j < 3; ++j)
         t[i][j] = r[i][j];
   }
   for (int i = 0; i < 3; ++i)
   {
      t[i][3] = 0;
      t[3][i] = 0;
   }
   t[3][3] = 1;

   Matrix4Type m;
   for (int i = 0; i < 4; ++i)
   {
      for (int j = 0; j < 4; ++j)
         m(i, j) = t[i][j];
   }
   return m;
}

void MakeHorizonal::SetInputFile(string _input_file)
{
   input_path = _input_file;
}

void MakeHorizonal::RemoveEdges()
{
   input_file.open(input_path.c_str());
   if (!input_file)
   {
      cout << "MakeHorizonal::RemoveEdges() : track_optimized.txt is not exist." << endl;
      exit(-1);
   }
   std::vector<string> VertexSet;
   std::string s, s1;
   while (1)
   {
      input_file >> s;
      if (input_file.eof())
         break;
      if (s != "VERTEX_SE3:QUAT")
      {
         getline(input_file, s);
         continue;
      }
      getline(input_file, s1);
      s = s + s1;
      VertexSet.push_back(s);
   }
   input_file.close();

#if 1
   std::ofstream output(input_path.c_str());
   for (std::vector<string>::iterator it = VertexSet.begin(); it != VertexSet.end(); ++it)
   {
      output << *it << std::endl;
   }
   output.close();
#endif
}

void MakeHorizonal::Run()
{
   RemoveEdges();
   PointCloud::Ptr cloud(new PointCloud);
   //ReadXYZfromVertex(cloud, startDis_, endDis_);
   ReadXYZfromVertex(cloud);

   consoleProgress(85);
   // calculate normal vector
   double in, inliers_percent;
   ;

   for (int z = 0; z < 200; z++)
   {
      in = 0.01 * (z + 1);
      std::vector<int> inliers;

      pcl::SampleConsensusModelPlane<PointType>::Ptr model_p(new pcl::SampleConsensusModelPlane<PointType>(cloud));
      pcl::RandomSampleConsensus<PointType> ransac(model_p);
      ransac.setDistanceThreshold(in);
      ransac.computeModel();
      ransac.getInliers(inliers);

      inliers_percent = inliers.size() / cloud->points.size();

      Eigen::VectorXf coef = Eigen::VectorXf::Zero(4, 1);
      ransac.getModelCoefficients(coef);
      double a = coef[0];
      double b = coef[1];
      double c = coef[2];
      double d = coef[3];
      if (c < 0)
      {
         a = -a;
         b = -b;
         c = -c;
      }
      double mo = sqrt(a * a + b * b + c * c);
      a = a / mo;
      b = b / mo;
      c = c / mo;
      a = a / 2;
      b = b / 2;
      c = (c + 1) / 2;
      mo = sqrt(a * a + b * b + c * c);
      a = a / mo;
      b = b / mo;
      c = c / mo;

#if 0
	    double min_z, max_z;
	    min_z = 10000;
	    max_z = -10000;
	    double cloud_temp;
	    for (size_t i = 0; i <cloud->points.size(); i++)
	    {
		    cloud_temp = 2 * c*(a*cloud->points[i].x + b*cloud->points[i].y) + (c*c - a*a - b*b)*cloud->points[i].z;

		    if (cloud_temp < min_z)
			    min_z = cloud_temp;
		    if (cloud_temp > max_z)
			    max_z = cloud_temp;

	    }
#endif
      if (inliers_percent > 0.99)
      {
         horizon_matrix = calculateMatrix(a, b, c);
         break;
      }
      consoleProgress(86 + int(1.0 * z / 200 * 4));
   }
   consoleProgress(90);
#if DEBUG
   // std::cout << "\n============================================================\nHorizon Matrix:\n"
   // 		  << horizon_matrix << endl;
#endif
   input_file.open(input_path.c_str());
   std::vector<Pos*> traj;
   //Trajectory *traj = new Trajectory();
   QuaternionType q;
   // Vector3Type vt;
   Matrix3Type horizon_r = g2o_tool.getRotation(horizon_matrix);
 
   // cout << endl << horizon_matrix << endl; //平面拟合矩阵
   // int count = 0;
   while (1)
   {
      Pos *tmpPos = new Pos();
      tmpPos->frame_id = g2o_tool.vertex2Transform(input_file, tmpPos->transform);
      if (input_file.eof())
         break;

      Matrix3Type pos_r = g2o_tool.getRotation(tmpPos->transform);
      Vector3Type pos_t = g2o_tool.getTranslation(tmpPos->transform);

      pos_r = horizon_r * pos_r;
      pos_t = horizon_r * pos_t;
      // tmpPos->transform = g2o_tool.rt2Transform(pos_r, pos_t);
      for (int i = 0; i < 3; ++i)
         for (int j = 0; j < 3; ++j)
            tmpPos->transform(i, j) = pos_r(i, j);
      for (int i = 0; i < 3; ++i)
         tmpPos->transform(i, 3) = pos_t(i);
      for (int i = 0; i < 3; ++i)
         tmpPos->transform(3, i) = 0;
      tmpPos->transform(3, 3) = 1;

      traj.push_back(tmpPos);
      // count++;
   }
   
   input_file.close();
   std::ofstream output2(input_path.c_str()); // cover old track
   for (size_t i = 0; i < traj.size(); ++i)
   {
      output2 << g2o_tool.transform2Vertex(traj[i]->frame_id, traj[i]->transform) << endl;
      delete traj[i];
   }
   output2.close();
   consoleProgress(91);
}
