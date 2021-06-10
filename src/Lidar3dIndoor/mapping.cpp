#include "mapping.h"

const float scanPeriod = 0.1;

const int stackFrameNum = 1;
const int mapFrameNum = 5;

double timeLaserCloudCornerLast = 0;
double timeLaserCloudSurfLast = 0;
double timeLaserCloudFullRes = 0;
double timeLaserOdometry = 0;

bool newLaserCloudCornerLast = false;
bool newLaserCloudSurfLast = false;
bool newLaserCloudFullRes = false;
bool newLaserOdometry = false;

int laserCloudCenWidth = 10;
int laserCloudCenHeight = 5;
int laserCloudCenDepth = 10;
const int laserCloudWidth = 21;
const int laserCloudHeight = 11;
const int laserCloudDepth = 21;
const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;

int laserCloudValidInd[125];
int laserCloudSurroundInd[125];

pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudCornerStack2(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfStack2(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudOri(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr coeffSel(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurround(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurround2(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<PointType>());
//pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];
pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];
pcl::PointCloud<PointType>::Ptr laserCloudCornerArray2[laserCloudNum];
pcl::PointCloud<PointType>::Ptr laserCloudSurfArray2[laserCloudNum];

pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());

typedef Eigen::Matrix<double, 6, 1> Vector6d;
Vector6d transform;
Vector6d transformSum;
Vector6d transformIncre;
Vector6d transformTobeMapped;
Vector6d transformLast20[20];
Vector6d transformBefMapped; //就是odometry传来的transformsum
Vector6d transformAftMapped;
Vector6d preTransformVector; //上一帧的变换向量
Vector6d iniTransformVector;

int imuPointerFront = 0;
int imuPointerLast = -1;
const int imuQueLength = 200;

double imuTime[imuQueLength] = {0};
float imuRoll[imuQueLength] = {0};
float imuPitch[imuQueLength] = {0};

inline void pointPickingEventOccurred (const pcl::visualization::PointPickingEvent &event,
                         void* viewer_void)
{  
	if (event.getPointIndex() == -1)
		return;

   pcl::visualization::PCLVisualizer *viewer = 
            static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	PointType current_point;
	event.getPoint(current_point.x, current_point.y, current_point.z);
 
	// Draw clicked points in blue:
   pcl::PointCloud<PointType>::Ptr clickedPoints(new pcl::PointCloud<PointType>());
   clickedPoints->points.push_back(current_point);
	pcl::visualization::PointCloudColorHandlerCustom<PointType> blue(clickedPoints, 0, 0, 255);
	viewer->removePointCloud("clicked_points");
	viewer->addPointCloud(clickedPoints,blue, "clicked_points");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
 
	int num=event.getPointIndex();

}
 
unsigned int __frameId = 0;
unsigned int __frameCnt = 0;
PointType __framePointPOs;
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
	void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getKeySym() == "e" && event.keyDown())
	{
      exit(0);
	}else if (event.getKeySym() == "space" && event.keyDown()){ 
      std::string id = "__frameId" + std::to_string(__frameCnt);
      // viewer->addText(std::to_string(__frameId).c_str(), 10, 80+__frameCnt*20, 24,1.0,1.0,1.0,id);
      id = "__frameId3D" + std::to_string(__frameCnt);
      PointType b3 = __framePointPOs;
      b3.x -= 0.1;
      b3.y -= 0.1; 
      b3.z -= 0.1; 

      viewer->addText3D(std::to_string(__frameId),b3,0.4, 1.0,1.0,1.0,id);

      pcl::PointCloud<PointType>::Ptr clickedPoints(new pcl::PointCloud<PointType>());
      clickedPoints->points.push_back(__framePointPOs);
      pcl::visualization::PointCloudColorHandlerCustom<PointType> blue(clickedPoints, 0, 255, 255); 
      id = "__frameId3Dpoints" + std::to_string(__frameCnt);
      viewer->addPointCloud(clickedPoints,blue, id);
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20, id);
      __frameCnt++;
   }
}

Eigen::Matrix4f Vector6dToRotate(Vector6d vector)
{
   Eigen::Matrix3d Rx = Eigen::AngleAxisd(vector[0], Eigen::Vector3d(1, 0, 0)).toRotationMatrix();
   Eigen::Matrix3d Ry = Eigen::AngleAxisd(vector[1], Eigen::Vector3d(0, 1, 0)).toRotationMatrix();
   Eigen::Matrix3d Rz = Eigen::AngleAxisd(vector[2], Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
   Eigen::Matrix3d R = Ry * Rx * Rz;

   Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();
   for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
         transformMatrix(i, j) = R(i, j);

   transformMatrix(0, 3) = vector[3];
   transformMatrix(1, 3) = vector[4];
   transformMatrix(2, 3) = vector[5];
   return transformMatrix;
}

Vector6d RotateToVector6d(Eigen::Matrix4f matrix)
{
   Eigen::Matrix3d Rotate;
   for (size_t i = 0; i < 3; i++)
      for (size_t j = 0; j < 3; j++)
         Rotate(i, j) = matrix(i, j);
   Eigen::Vector3d yxz = Rotate.eulerAngles(1, 0, 2);
   Vector6d v6d;
   v6d[0] = yxz[1];
   v6d[1] = yxz[0];
   v6d[2] = yxz[2];
   v6d[3] = matrix(0, 3);
   v6d[4] = matrix(1, 3);
   v6d[5] = matrix(2, 3);
   return v6d;
}

Mapping::Mapping()
{
   isMerge2Cloud = false;
   // isInitTraj = false;
   startFrameNumber = 0;
   endFrameNumber = -1;
   Nscans = 32;
   Nscans2 = 16;
   skipFrameNumber = 3;
   isShowCloud = false;
   transformation << 0.99958, 0.01688, -0.0235602, -0.000400302,
       -0.0286371, 0.700434, -0.713142, -0.0264561,
       0.00446452, 0.713517, 0.700623, -0.236375,
       0, 0, 0, 1; //our backpack
}

void Mapping::transformAssociateToMap()
{
   float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
   float y1 = transformBefMapped[4] - transformSum[4];
   float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

   float x2 = x1;
   float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
   float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

   transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
   transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
   transformIncre[5] = z2;

   float sbcx = sin(transformSum[0]);
   float cbcx = cos(transformSum[0]);
   float sbcy = sin(transformSum[1]);
   float cbcy = cos(transformSum[1]);
   float sbcz = sin(transformSum[2]);
   float cbcz = cos(transformSum[2]);

   float sblx = sin(transformBefMapped[0]);
   float cblx = cos(transformBefMapped[0]);
   float sbly = sin(transformBefMapped[1]);
   float cbly = cos(transformBefMapped[1]);
   float sblz = sin(transformBefMapped[2]);
   float cblz = cos(transformBefMapped[2]);

   float salx = sin(transformAftMapped[0]);
   float calx = cos(transformAftMapped[0]);
   float saly = sin(transformAftMapped[1]);
   float caly = cos(transformAftMapped[1]);
   float salz = sin(transformAftMapped[2]);
   float calz = cos(transformAftMapped[2]);

   float srx = -sbcx * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz) - cbcx * sbcy * (calx * calz * (cbly * sblz - cblz * sblx * sbly) - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) - cbcx * cbcy * (calx * salz * (cblz * sbly - cbly * sblx * sblz) - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx);
   transformTobeMapped[0] = -asin(srx);

   float srycrx = sbcx * (cblx * cblz * (caly * salz - calz * salx * saly) - cblx * sblz * (caly * calz + salx * saly * salz) + calx * saly * sblx) - cbcx * cbcy * ((caly * calz + salx * saly * salz) * (cblz * sbly - cbly * sblx * sblz) + (caly * salz - calz * salx * saly) * (sbly * sblz + cbly * cblz * sblx) - calx * cblx * cbly * saly) + cbcx * sbcy * ((caly * calz + salx * saly * salz) * (cbly * cblz + sblx * sbly * sblz) + (caly * salz - calz * salx * saly) * (cbly * sblz - cblz * sblx * sbly) + calx * cblx * saly * sbly);
   float crycrx = sbcx * (cblx * sblz * (calz * saly - caly * salx * salz) - cblx * cblz * (saly * salz + caly * calz * salx) + calx * caly * sblx) + cbcx * cbcy * ((saly * salz + caly * calz * salx) * (sbly * sblz + cbly * cblz * sblx) + (calz * saly - caly * salx * salz) * (cblz * sbly - cbly * sblx * sblz) + calx * caly * cblx * cbly) - cbcx * sbcy * ((saly * salz + caly * calz * salx) * (cbly * sblz - cblz * sblx * sbly) + (calz * saly - caly * salx * salz) * (cbly * cblz + sblx * sbly * sblz) - calx * caly * cblx * sbly);
   transformTobeMapped[1] = atan2(srycrx / cos(transformTobeMapped[0]),
                                  crycrx / cos(transformTobeMapped[0]));

   float srzcrx = (cbcz * sbcy - cbcy * sbcx * sbcz) * (calx * salz * (cblz * sbly - cbly * sblx * sblz) - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx) - (cbcy * cbcz + sbcx * sbcy * sbcz) * (calx * calz * (cbly * sblz - cblz * sblx * sbly) - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) + cbcx * sbcz * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
   float crzcrx = (cbcy * sbcz - cbcz * sbcx * sbcy) * (calx * calz * (cbly * sblz - cblz * sblx * sbly) - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) - (sbcy * sbcz + cbcy * cbcz * sbcx) * (calx * salz * (cblz * sbly - cbly * sblx * sblz) - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx) + cbcx * cbcz * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
   transformTobeMapped[2] = atan2(srzcrx / cos(transformTobeMapped[0]),
                                  crzcrx / cos(transformTobeMapped[0]));

   x1 = cos(transformTobeMapped[2]) * transformIncre[3] - sin(transformTobeMapped[2]) * transformIncre[4];
   y1 = sin(transformTobeMapped[2]) * transformIncre[3] + cos(transformTobeMapped[2]) * transformIncre[4];
   z1 = transformIncre[5];

   x2 = x1;
   y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
   z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

   transformTobeMapped[3] = transformAftMapped[3] - (cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2);
   transformTobeMapped[4] = transformAftMapped[4] - y2;
   transformTobeMapped[5] = transformAftMapped[5] - (-sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2);
}

void Mapping::transformUpdate()
{
   if (imuPointerLast >= 0)
   {
      float imuRollLast = 0, imuPitchLast = 0;
      while (imuPointerFront != imuPointerLast)
      {
         if (timeLaserOdometry + scanPeriod < imuTime[imuPointerFront])
         {
            break;
         }
         imuPointerFront = (imuPointerFront + 1) % imuQueLength;
      }

      if (timeLaserOdometry + scanPeriod > imuTime[imuPointerFront])
      {
         imuRollLast = imuRoll[imuPointerFront];
         imuPitchLast = imuPitch[imuPointerFront];
      }
      else
      {
         int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
         float ratioFront = (timeLaserOdometry + scanPeriod - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
         float ratioBack = (imuTime[imuPointerFront] - timeLaserOdometry - scanPeriod) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

         imuRollLast = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
         imuPitchLast = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
      }

      transformTobeMapped[0] = 0.998 * transformTobeMapped[0] + 0.002 * imuPitchLast;
      transformTobeMapped[2] = 0.998 * transformTobeMapped[2] + 0.002 * imuRollLast;
   }

   for (int i = 0; i < 6; i++)
   {
      transformBefMapped[i] = transformSum[i];
      transformAftMapped[i] = transformTobeMapped[i];
   }
}

/**
 * 将单个点转换到全局地图下 
 */
void Mapping::pointAssociateToMap(PointType const *const pi, PointType *const po)
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

void Mapping::pointAssociateTobeMapped(PointType const *const pi, PointType *const po)
{
   double rx, ry, rz, tx, ty, tz;
   rx = transformTobeMapped[0];
   ry = transformTobeMapped[1];
   rz = transformTobeMapped[2];
   tx = transformTobeMapped[3];
   ty = transformTobeMapped[4];
   tz = transformTobeMapped[5];
   float x1 = cos(ry) * (pi->x - tx) - sin(ry) * (pi->z - tz);
   float y1 = pi->y - ty;
   float z1 = sin(ry) * (pi->x - tx) + cos(ry) * (pi->z - tz);

   float x2 = x1;
   float y2 = cos(rx) * y1 + sin(rx) * z1;
   float z2 = -sin(rx) * y1 + cos(rx) * z1;

   po->x = cos(rz) * x2 + sin(rz) * y2;
   po->y = -sin(rz) * x2 + cos(rz) * y2;
   po->z = z2;
   po->intensity = pi->intensity;
}

/**
 * @param[in] 将要保存的轨迹地址,@param[in]pcap文件地址,@param[in]pcap对应XML地址
 * @param[in] 是否显示点云 
 */
int Mapping::run(std::string txtSaveLoc, std::string fileNamePcap, std::string caliPath)
{
   std::vector<int> pointSearchInd;
   std::vector<float> pointSearchSqDis;

   PointType pointOri, pointSel, pointProj, coeff;

   cv::Mat matA0(5, 3, CV_32F, cv::Scalar::all(0));
   cv::Mat matB0(5, 1, CV_32F, cv::Scalar::all(-1));
   cv::Mat matX0(3, 1, CV_32F, cv::Scalar::all(0));

   cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
   cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
   cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

   bool isDegenerate = false;
   cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

   pcl::VoxelGrid<PointType> downSizeFilterCorner;
   downSizeFilterCorner.setLeafSize(0.05, 0.05, 0.05);

   pcl::VoxelGrid<PointType> downSizeFilterSurf;
   downSizeFilterSurf.setLeafSize(0.1, 0.1, 0.1);

   pcl::VoxelGrid<PointType> downSizeFilterMap;
   downSizeFilterMap.setLeafSize(0.06, 0.06, 0.06);


   for (int i = 0; i < laserCloudNum; i++)
   {
      laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
      laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
      laserCloudCornerArray2[i].reset(new pcl::PointCloud<PointType>());
      laserCloudSurfArray2[i].reset(new pcl::PointCloud<PointType>());
   }

   int frameCount = stackFrameNum - 1;
   int mapFrameCount = mapFrameNum - 1;
   //--------------------初始化参数----------------------------------------
   //----------------------------------------
   //----------------------------------------
   std::fstream fileWrite;
   std::fstream errorWrite;
   FeatureExtraction getfeature; //提取特征的类
   PointCloudReader reader1;
   PointCloud::Ptr frame1(new PointCloud);
   PointCloud::Ptr framePre(new PointCloud);
   PointCloud::Ptr frameGroup(new PointCloud);
   PointCloud::Ptr frame2(new PointCloud);
   PointCloud::Ptr alignedframe(new PointCloud);
   pcl::visualization::PCLVisualizer *viewer;
   pcl::visualization::PCLVisualizer *viewerCorner;
   pcl::visualization::PCLVisualizer *viewerSurf;

   int frameID, frameID2;
   int minFlamePointSize = 2000;
   int minPcapSize = 1000;
   float outdoorTreshhold = 0.035;
   bool isStart = false;

   reader1.setFrameGap(skipFrameNumber);
   reader1.setScansGap(0);
   reader1.setPcapPath(fileNamePcap);
   reader1.setCalibrationPath(caliPath);
   reader1.frameNumber = startFrameNumber;
   reader1.setGPSFile(gpsfile);

   if (isMerge2Cloud)
   {
      reader2.setFrameGap(skipFrameNumber);
      reader2.setScansGap(Nscans);
      reader2.setPcapPath(pcap2);
      reader2.setCalibrationPath(caliPath2);
      reader2.frameNumber = startFrameNumber;
      reader2.setGPSFile(gpsfile);

      if (!reader2.OpenPcap())
      {
         printf("Cannot open pcap2 file!\n");
         return -1;
      }
      // if (reader2.getTotalFrame() < minPcapSize && !TEST)
      // {
      //    printf("The pcap2 data is too small!\n");
      //    return -1;
      // }
      printf("Frame2 = %ld\n", (long int)(reader2.getTotalFrame()));
      Nscans += Nscans2;
   }

   getfeature.setNscans(Nscans);

   if (!reader1.OpenPcap())
   {
      printf("Cannot open pcap file!\n");
      return -1;
   }
   if (endFrameNumber < 0)
      endFrameNumber = reader1.getTotalFrame();
   // if (reader1.getTotalFrame() < minPcapSize && !TEST)
   // {
   //    printf("The pcap data is too small!\n");
   //    return -1;
   // }
   if (Nscans == 16)
   {
      // minFlamePointSize = 8000;
      minFlamePointSize = 1000;
   }
   /**
    * 
   // 判断是否在室外
   float outdoorProb = isOutdoor(reader1, minFlamePointSize);
   reader1.frameNumber = startFrameNumber;registerPointPickingCallback
   if (outdoorProb > outdoorTreshhold && !TEST)
   {
      printf("WARNING: The program can't running in currunt environment!\n\n");
      return -1;
   }
   else if (outdoorProb > outdoorTreshhold / 2)
   {
      printf("The mapping result may be not good in currunt environment!\n\n");
   }
   else if (outdoorProb < 0)
   {
      printf("Isoutdoor:Something wrong with prob!\n");
      return -1;
   }
   */
   printf("Total Frame = %ld\n", (long int)(reader1.getTotalFrame()));
   consoleProgress(0);
   std::string temp_s = txtSaveLoc.substr(0, txtSaveLoc.find('.')) + "_error.txt";
   errorWrite.open(temp_s.c_str(), std::ios::out | std::ios::app);
   fileWrite.open(txtSaveLoc.c_str(), std::ios::out | std::ios::app);
   if (isShowCloud)
   {
      viewer = new pcl::visualization::PCLVisualizer("Show cloud"); 
      // viewer->registerPointPickingCallback(pointPickingEventOccurred, (void*)viewer);
      viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer);
      viewer->addText("", 10, 10,"frameID");
      viewer->addText("Total frame: " + std::to_string(reader1.getTotalFrame()) , 10, 105, 25, 1.0, 1.0, 1.0,"totalframe");
      viewer->addText("Press 'e' to exit", 10, 40, 20, 1.0, 1.0, 1.0, "note1");
      viewer->addText("Press 'space' to record current frame ID", 10, 20, 20, 1.0, 1.0, 1.0, "note2");
      // viewerCorner = new pcl::visualization::PCLVisualizer("Show Corner");
      // viewerSurf = new pcl::visualization::PCLVisualizer("Show Surf");
   }
   //--------------------初始化参数----------------------------------------

   //setGICP
   // gicp.setInputSource(map);
   // GICP.setRotationEpsilon(1e-9);
   GICP.setTransformationEpsilon(1e-8); //为终止条件设置最小转换差异
   GICP.setMaxCorrespondenceDistance(2); //设置对应点对之间的最大距离
   GICP.setEuclideanFitnessEpsilon(0.001); //设置收敛条件是均方误差和小于阈值， 停止迭代
   GICP.setMaximumOptimizerIterations(40);

   int frameOffset = 0;
   float preStartTime, curStartTime, frameTime;
   double frame_startTime = 0;
   int fitCount = 0;
   bool skipCurFrame = false;
   float distLast20[20] = {0};
   int preID[2] =  {-1};
   PointType linestart, lineend;
   linestart.x = 0;
   linestart.y = 0;
   linestart.z = 0;
   reader1.frameNumber = startFrameNumber;
   while (reader1.readPointCloud(frame1) != -1)
   {
      if (frame1->points.size() > 79900 || frame1->points.size() < 1000) //featureextraction.h中设置的最大上限
      {
         continue;
      }
      frameID = reader1.frameNumber - skipFrameNumber;
      if (endFrameNumber > 0 && frameID >= endFrameNumber)
         break;

      // 融合两个激光头数据，若有时间差，纠正到半帧时间之内
      if (isMerge2Cloud)
      {
         reader2.frameNumber = frameID + frameOffset;
         if (reader2.readPointCloud(frame2) == -1)
            break;
         frameID2 = reader2.frameNumber - skipFrameNumber;

         //—————————————————————————调整时间差的代码——————————————————————
         //201也可能掉帧，判断一下
         frameTime = frame2->points.back().data_n[0] + frame2->points.back().data_n[1] - (frame2->points[0].data_n[0] + frame2->points[0].data_n[1]);
         if (frame2->points.size() < 8000 
            || frameTime > 0.06 
            // || frameTime < 0.04
            )
         {
            cout << "Attention: 激光头可能存在掉帧或者数据不稳定情况!!!\n";
            showTime(frame1, frameID, "First");
            showTime(frame2, frameID2, "Second");
            goto OutMerge2Cloud;
         }

         float timeOffset = frame2->points[0].data_n[0] + frame2->points[0].data_n[1] - (frame1->points[0].data_n[0] + frame1->points[0].data_n[1]);
         frameTime = 0.05;

         //计算两个激光头之间的时间差，让时间差位于半帧的时间之内
         int _a = timeOffset / frameTime;
         _a += 1.9 * (timeOffset - _a * frameTime) / frameTime;
         _a = 0;
         // frameOffset -= _a; //对不准就算了把

         if (abs(_a) > 0.1)
         {  
            if(preID[0] == -1 && preID[1] == -1)
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

            cout << "======================FrameOffset============================\n"
                 << "FrameID = " << frameID << "\t"
                 << "Offset = " << -_a << "\t"
                 << "SumFrameOffset = " << frameOffset << "\t "
                 << "Timeoffset  = " << timeOffset << endl;
            showTime(frame1, frameID, "First");
            showTime(frame2, frameID2, "Second");
            cout << "=============================================================\n";
            if (abs(_a) > 1 && frameID - startFrameNumber > 5 && isShowCloud )
            {
               cout << "\nAttention: Frame2's timestamp is changing more than 0.25s!!!\n";
               goto OutMerge2Cloud;
            }
            if (frameID + frameOffset < 3)
               frameID += _a; //最开始调整到负帧情况要避免
            reader1.frameNumber = frameID;
            preStartTime = curStartTime - 0.05 * skipFrameNumber;
            continue;
         }
         //—————————————————————————调整时间差的代码——————————————————————
         else if (isShowCloud)
         {
            // viewerCorner->spinOnce();
            // viewerSurf->spinOnce();
            showTime(frame1, frameID, "First");
            showTime(frame2, frameID2, "Second");
         }
         pcl::transformPointCloud<PointType>(*frame2, *frame2, transformation);
         *frame1 += *frame2;
      }
      else if (isShowCloud && TEST)
      {
         // viewerCorner->spinOnce();
         // viewerSurf->spinOnce();
         showTime(frame1, frameID, "Test", frame_startTime);
      }
      frame_startTime = frame1->points[0].data_n[0] + frame1->points[0].data_n[1];

   OutMerge2Cloud:
      //------------------------------------------------------------------------------------------------
      //---------------------------------getdata
      //------------------------------------------------------------------------------------------------
      //printf("frameN:%d\n",frameN);
      PointCloud::Ptr cornerPointsSharp(new PointCloud);
      PointCloud::Ptr cornerPointsLessSharp(new PointCloud);
      PointCloud::Ptr surfPointsFlat(new PointCloud);
      PointCloud::Ptr surfPointsLessFlat(new PointCloud);

      //删除前面点数不足的帧
      if (!isStart)
      {
         //printf("cur_frame->points=%lu\n",cur_frame->points.size());
         if (frame1->points.size() < minFlamePointSize)
         {
            continue;
         }
         else
         {
            isStart = true;
         }
      }

      getfeature.setInputCloud(*frame1);
      getfeature.getCornerSharp(*cornerPointsSharp);
      getfeature.getCornerPointsLessSharp(*cornerPointsLessSharp);
      getfeature.getSurfPointsFlat(*surfPointsFlat);
      getfeature.getSurfPointsLessFlat(*surfPointsLessFlat);
      if (isShowCloud && TEST)
      {
         cout << "{Cor,Cor2,Surf,Surf2 = "
              << cornerPointsSharp->points.size() << "\t"
              << cornerPointsLessSharp->points.size() << "\t"
              << surfPointsFlat->points.size() << "\t"
              << surfPointsLessFlat->points.size() << endl;
         // ShowFeature(cornerPointsLessSharp, cornerPointsLessSharp, surfPointsLessFlat, surfPointsLessFlat);
      }
      // double sumTraj = initTraj[frameID][0] + initTraj[frameID][1] + initTraj[frameID][2] + initTraj[frameID][3] + initTraj[frameID][4] +initTraj[frameID][5];
      if(isInitTraj[frameID])
      {
         iniTransformVector = initTraj[frameID];
      }
      else
         // iniTransformVector = 2 * transformTobeMapped - preTransformVector; //一阶: 简单拟合的一个初始变换向量 
         iniTransformVector = transformTobeMapped;
         // // 二阶: t_i = t_{i-1} + (t_{i-1} - t_i{-2}) + 0.5 * (t_{i-2} - t_{i-3}) 
         // for (size_t i = 3; i < 6; i++)
         // {
         //    iniTransformVector[i] = 3 * transformLast20[(frameID - 1) % 20][i] -
         //                         3 * transformLast20[(frameID - 2) % 20][i] +
         //                         1 * transformLast20[(frameID - 3) % 20][i]; 
         // }

      preTransformVector = transformTobeMapped;
      transformTobeMapped = iniTransformVector;

      frameCount++;

      laserCloudCornerLast = cornerPointsLessSharp;
      laserCloudSurfLast = surfPointsLessFlat;

      if (frameCount >= stackFrameNum)
      {
         //将特征点放到上一帧附近
         //transformAssociateToMap();
         int laserCloudCornerLastNum = laserCloudCornerLast->points.size();
         for (int i = 0; i < laserCloudCornerLastNum; i++)
         {
            pointAssociateToMap(&laserCloudCornerLast->points[i], &pointSel);
            laserCloudCornerStack2->push_back(pointSel);
         }

         int laserCloudSurfLastNum = laserCloudSurfLast->points.size();
         for (int i = 0; i < laserCloudSurfLastNum; i++)
         {
            pointAssociateToMap(&laserCloudSurfLast->points[i], &pointSel);
            laserCloudSurfStack2->push_back(pointSel);
         }
      }

      if (frameCount >= stackFrameNum)
      {
         frameCount = 0;

         PointType pointOnYAxis;
         pointOnYAxis.x = 0.0;
         pointOnYAxis.y = 10.0;
         pointOnYAxis.z = 0.0;
         pointAssociateToMap(&pointOnYAxis, &pointOnYAxis);

         //第一帧起始(I,J,K)为(10,5,10)
         int centerCubeI = int((transformTobeMapped[3] + 25.0) / 50.0) + laserCloudCenWidth;  //(x+25)/50 + 10
         int centerCubeJ = int((transformTobeMapped[4] + 25.0) / 50.0) + laserCloudCenHeight; //5
         int centerCubeK = int((transformTobeMapped[5] + 25.0) / 50.0) + laserCloudCenDepth;  //10

         if (transformTobeMapped[3] + 25.0 < 0)
            centerCubeI--;
         if (transformTobeMapped[4] + 25.0 < 0)
            centerCubeJ--;
         if (transformTobeMapped[5] + 25.0 < 0)
            centerCubeK--;

         while (centerCubeI < 3)
         {
            for (int j = 0; j < laserCloudHeight; j++)
            {
               for (int k = 0; k < laserCloudDepth; k++)
               {
                  int i = laserCloudWidth - 1;
                  //i,j,k在空间方块中坐标公式
                  pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                      laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                  pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                      laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                  for (; i >= 1; i--)
                  {
                     laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                         laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                     laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                         laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                  }
                  laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                      laserCloudCubeCornerPointer;
                  laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                      laserCloudCubeSurfPointer;
                  laserCloudCubeCornerPointer->clear();
                  laserCloudCubeSurfPointer->clear();
               }
            }

            centerCubeI++;
            laserCloudCenWidth++;
         }

         while (centerCubeI >= laserCloudWidth - 3)
         {
            for (int j = 0; j < laserCloudHeight; j++)
            {
               for (int k = 0; k < laserCloudDepth; k++)
               {
                  int i = 0;
                  pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                      laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                  pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                      laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                  for (; i < laserCloudWidth - 1; i++)
                  {
                     laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                         laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                     laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                         laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                  }
                  laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                      laserCloudCubeCornerPointer;
                  laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                      laserCloudCubeSurfPointer;
                  laserCloudCubeCornerPointer->clear();
                  laserCloudCubeSurfPointer->clear();
               }
            }

            centerCubeI--;
            laserCloudCenWidth--;
         }

         while (centerCubeJ < 3)
         {
            for (int i = 0; i < laserCloudWidth; i++)
            {
               for (int k = 0; k < laserCloudDepth; k++)
               {
                  int j = laserCloudHeight - 1;
                  pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                      laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                  pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                      laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                  for (; j >= 1; j--)
                  {
                     laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                         laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
                     laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                         laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
                  }
                  laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                      laserCloudCubeCornerPointer;
                  laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                      laserCloudCubeSurfPointer;
                  laserCloudCubeCornerPointer->clear();
                  laserCloudCubeSurfPointer->clear();
               }
            }

            centerCubeJ++;
            laserCloudCenHeight++;
         }

         while (centerCubeJ >= laserCloudHeight - 3)
         {
            for (int i = 0; i < laserCloudWidth; i++)
            {
               for (int k = 0; k < laserCloudDepth; k++)
               {
                  int j = 0;
                  pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                      laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                  pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                      laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                  for (; j < laserCloudHeight - 1; j++)
                  {
                     laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                         laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
                     laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                         laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
                  }
                  laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                      laserCloudCubeCornerPointer;
                  laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                      laserCloudCubeSurfPointer;
                  laserCloudCubeCornerPointer->clear();
                  laserCloudCubeSurfPointer->clear();
               }
            }

            centerCubeJ--;
            laserCloudCenHeight--;
         }

         while (centerCubeK < 3)
         {
            for (int i = 0; i < laserCloudWidth; i++)
            {
               for (int j = 0; j < laserCloudHeight; j++)
               {
                  int k = laserCloudDepth - 1;
                  pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                      laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                  pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                      laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                  for (; k >= 1; k--)
                  {
                     laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                         laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
                     laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                         laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
                  }
                  laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                      laserCloudCubeCornerPointer;
                  laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                      laserCloudCubeSurfPointer;
                  laserCloudCubeCornerPointer->clear();
                  laserCloudCubeSurfPointer->clear();
               }
            }

            centerCubeK++;
            laserCloudCenDepth++;
         }

         while (centerCubeK >= laserCloudDepth - 3)
         {
            for (int i = 0; i < laserCloudWidth; i++)
            {
               for (int j = 0; j < laserCloudHeight; j++)
               {
                  int k = 0;
                  pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                      laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                  pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                      laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                  for (; k < laserCloudDepth - 1; k++)
                  {
                     laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                         laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
                     laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                         laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
                  }
                  laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                      laserCloudCubeCornerPointer;
                  laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                      laserCloudCubeSurfPointer;
                  laserCloudCubeCornerPointer->clear();
                  laserCloudCubeSurfPointer->clear();
               }
            }

            centerCubeK--;
            laserCloudCenDepth--;
         }

         //在(12,7,12) >= (I,J,K) >=(8,3,8)
         int laserCloudValidNum = 0;
         int laserCloudSurroundNum = 0;
         for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
         {
            for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
            {
               for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++)
               {
                  if (i >= 0 && i < laserCloudWidth &&
                      j >= 0 && j < laserCloudHeight &&
                      k >= 0 && k < laserCloudDepth)
                  {

                     float centerX = 50.0 * (i - laserCloudCenWidth);//真实坐标
                     float centerY = 50.0 * (j - laserCloudCenHeight);
                     float centerZ = 50.0 * (k - laserCloudCenDepth);

                     bool isInLaserFOV = false;
                     for (int ii = -1; ii <= 1; ii += 2)
                     {
                        for (int jj = -1; jj <= 1; jj += 2)
                        {
                           for (int kk = -1; kk <= 1; kk += 2)
                           {
                              float cornerX = centerX + 25.0 * ii;//中心点的两侧边缘
                              float cornerY = centerY + 25.0 * jj;
                              float cornerZ = centerZ + 25.0 * kk;
                              // (x-cornerx)²+(y-cornery)²+(z-cornerz)²
                              float squaredSide1 = (transformTobeMapped[3] - cornerX) * (transformTobeMapped[3] - cornerX) + (transformTobeMapped[4] - cornerY) * (transformTobeMapped[4] - cornerY) + (transformTobeMapped[5] - cornerZ) * (transformTobeMapped[5] - cornerZ);

                              float squaredSide2 = (pointOnYAxis.x - cornerX) * (pointOnYAxis.x - cornerX) + (pointOnYAxis.y - cornerY) * (pointOnYAxis.y - cornerY) + (pointOnYAxis.z - cornerZ) * (pointOnYAxis.z - cornerZ);

                              float check1 = 100.0 + squaredSide1 - squaredSide2 - 10.0 * sqrt(3.0) * sqrt(squaredSide1);

                              float check2 = 100.0 + squaredSide1 - squaredSide2 + 10.0 * sqrt(3.0) * sqrt(squaredSide1);

                              //当前立方体的角点、轨迹点、pointOnYAxis点，三点组成的角度在30°到150°之间
                              if (check1 < 0 && check2 > 0)
                              {
                                 isInLaserFOV = true;
                              }
                           }
                        }
                     }

                     if (isInLaserFOV)
                     {
                        laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                        laserCloudValidNum++;
                     }
                     laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                     laserCloudSurroundNum++;
                  }
               }
            }
         }

         laserCloudCornerFromMap->clear();
         laserCloudSurfFromMap->clear();
         for (int i = 0; i < laserCloudValidNum; i++)
         {
            *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
            *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
         }

         int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
         int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();

         int laserCloudCornerStackNum2 = laserCloudCornerStack2->points.size();
         for (int i = 0; i < laserCloudCornerStackNum2; i++)
         {
            pointAssociateTobeMapped(&laserCloudCornerStack2->points[i], &laserCloudCornerStack2->points[i]);
         }

         int laserCloudSurfStackNum2 = laserCloudSurfStack2->points.size();
         for (int i = 0; i < laserCloudSurfStackNum2; i++)
         {
            pointAssociateTobeMapped(&laserCloudSurfStack2->points[i], &laserCloudSurfStack2->points[i]);
         }

         laserCloudCornerStack->clear();
         downSizeFilterCorner.setInputCloud(laserCloudCornerStack2);
         downSizeFilterCorner.filter(*laserCloudCornerStack);
         int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

         laserCloudSurfStack->clear();
         downSizeFilterSurf.setInputCloud(laserCloudSurfStack2);
         downSizeFilterSurf.filter(*laserCloudSurfStack);
         int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

         laserCloudCornerStack2->clear();
         laserCloudSurfStack2->clear();

         if (isShowCloud)
         {
            // for (int i = 0; i < laserCloudValidNum; i++)
            // {
            //    //安装分块来显示点云 
            //    viewer->removePointCloud(std::to_string(laserCloudValidInd[i]).c_str());
            //    addCloud(laserCloudSurfArray[laserCloudValidInd[i]], laserCloudValidInd[i], viewer);
            // }
            lineend.x = transformTobeMapped[3];
            lineend.y = transformTobeMapped[4];
            lineend.z = transformTobeMapped[5];

            // viewerCorner->addLine(linestart, lineend, 255, 255, 255, std::to_string(frameID).c_str());
            // viewerSurf->addLine(linestart, lineend, 255, 255, 255, std::to_string(frameID).c_str());
            viewer->addLine(linestart, lineend, 0, 255, 255, std::to_string(frameID).c_str()); 
            viewer->updateText( "Current frame: " + std::to_string(frameID), 10, 80, 25,1.0,1.0,1.0,"frameID");
            __frameId = frameID;
            __framePointPOs = linestart;
            linestart = lineend;
            // ShowCloud(laserCloudCornerFromMap, viewerCorner);
            // ShowCloud(laserCloudSurfFromMap, viewerSurf);
            ShowCloud(laserCloudCornerFromMap, viewer, "corner");
            ShowCloud(laserCloudSurfFromMap, viewer, "surf");
         }
         //全部特征点云
         if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 100)
         {
            //从划分好的地图中加载当前激光扫描所在的区域部分
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);

            size_t maxIterations = 20;
            if (frameID % 4 == 0)
            {
               Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();
               Eigen::Matrix4f ICPMatrix = Eigen::Matrix4f::Identity();

               // 两帧乘上transformTobeMapped；
               transformMatrix = Vector6dToRotate(transformTobeMapped);
               pcl::transformPointCloud(*frame1, *frame1, transformMatrix);
               pcl::transformPointCloud(*framePre, *framePre, transformMatrix);

               GICP.setInputTarget(frame1);
               GICP.setInputSource(framePre);
               GICP.align(*alignedframe);
               ICPMatrix = GICP.getFinalTransformation();

               transformTobeMapped = RotateToVector6d(transformMatrix * ICPMatrix); // matrix * T
            }
            else
            {
               for (int iterCount = 0; iterCount < maxIterations; iterCount++)
               {

                  //前期静止不动的帧数
                  if (frameID < 350)
                  {
                     for (int i = 0; i < 6; i++)
                        transformTobeMapped[i] = 0;
                     continue;
                  }

                  laserCloudOri->clear(); //迭代中的中间变量
                  coeffSel->clear();

                  for (int i = 0; i < laserCloudCornerStackNum; i++)
                  {
                     pointOri = laserCloudCornerStack->points[i];
                     pointAssociateToMap(&pointOri, &pointSel);
                     kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                     if (pointSearchSqDis[4] < 1.0)
                     {
                        float cx = 0;
                        float cy = 0;
                        float cz = 0;

                        for (int j = 0; j < 5; j++)
                        {
                           cx += laserCloudCornerFromMap->points[pointSearchInd[j]].x;
                           cy += laserCloudCornerFromMap->points[pointSearchInd[j]].y;
                           cz += laserCloudCornerFromMap->points[pointSearchInd[j]].z;
                        }
                        //五个点的中心
                        cx /= 5;
                        cy /= 5;
                        cz /= 5;

                        float a11 = 0;
                        float a12 = 0;
                        float a13 = 0;
                        float a22 = 0;
                        float a23 = 0;
                        float a33 = 0;
                        for (int j = 0; j < 5; j++)
                        {
                           float ax = laserCloudCornerFromMap->points[pointSearchInd[j]].x - cx;
                           float ay = laserCloudCornerFromMap->points[pointSearchInd[j]].y - cy;
                           float az = laserCloudCornerFromMap->points[pointSearchInd[j]].z - cz;

                           a11 += ax * ax;
                           a12 += ax * ay;
                           a13 += ax * az;
                           a22 += ay * ay;
                           a23 += ay * az;
                           a33 += az * az;
                        }
                        a11 /= 5;
                        a12 /= 5;
                        a13 /= 5;
                        a22 /= 5;
                        a23 /= 5;
                        a33 /= 5;

                        matA1.at<float>(0, 0) = a11;
                        matA1.at<float>(0, 1) = a12;
                        matA1.at<float>(0, 2) = a13;
                        matA1.at<float>(1, 0) = a12;
                        matA1.at<float>(1, 1) = a22;
                        matA1.at<float>(1, 2) = a23;
                        matA1.at<float>(2, 0) = a13;
                        matA1.at<float>(2, 1) = a23;
                        matA1.at<float>(2, 2) = a33;

                        cv::eigen(matA1, matD1, matV1); //对最近邻五个特征点进行特征值平方分解

                        if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1))
                        { //最大的特征值远大于第二大的特征值

                           float x0 = pointSel.x;
                           float y0 = pointSel.y;
                           float z0 = pointSel.z;
                           float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                           float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                           float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                           float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                           float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                           float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                           //某种方法判断是否真的是直线
                           float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

                           float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

                           float la = ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) / a012 / l12;

                           float lb = -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) - (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

                           float lc = -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

                           float ld2 = a012 / l12;

                           pointProj = pointSel;
                           pointProj.x -= la * ld2;
                           pointProj.y -= lb * ld2;
                           pointProj.z -= lc * ld2;

                           float s = 1 - 0.9 * fabs(ld2);

                           coeff.x = s * la;
                           coeff.y = s * lb;
                           coeff.z = s * lc;
                           coeff.intensity = s * ld2;

                           if (s > 0.1)
                           {
                              laserCloudOri->push_back(pointOri);
                              coeffSel->push_back(coeff);
                           }
                        }
                     }
                  }

                  for (int i = 0; i < laserCloudSurfStackNum; i++)
                  {
                     pointOri = laserCloudSurfStack->points[i];
                     pointAssociateToMap(&pointOri, &pointSel);
                     kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                     if (pointSearchSqDis[4] < 1.0)
                     {
                        //matA是五个点的坐标
                        for (int j = 0; j < 5; j++)
                        {
                           matA0.at<float>(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
                           matA0.at<float>(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
                           matA0.at<float>(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
                        }
                        //matB0 = (-1,-1,-1) 实际上是求五个点的投影向量
                        cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);

                        float pa = matX0.at<float>(0, 0);
                        float pb = matX0.at<float>(1, 0);
                        float pc = matX0.at<float>(2, 0);
                        float pd = 1;

                        //齐次向量
                        float ps = sqrt(pa * pa + pb * pb + pc * pc);
                        pa /= ps;
                        pb /= ps;
                        pc /= ps;
                        pd /= ps;

                        bool planeValid = true;
                        for (int j = 0; j < 5; j++)
                        {
                           //判断是否满足要求的平面
                           if (fabs(pa * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                                    pb * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                                    pc * laserCloudSurfFromMap->points[pointSearchInd[j]].z + pd) > 0.2)
                           {
                              planeValid = false;
                              break;
                           }
                        }

                        if (planeValid)
                        {
                           float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                           pointProj = pointSel;
                           pointProj.x -= pa * pd2;
                           pointProj.y -= pb * pd2;
                           pointProj.z -= pc * pd2;

                           float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                           coeff.x = s * pa;
                           coeff.y = s * pb;
                           coeff.z = s * pc;
                           coeff.intensity = s * pd2;

                           if (s > 0.1)
                           {
                              laserCloudOri->push_back(pointOri);
                              coeffSel->push_back(coeff);
                           }
                        }
                     }
                  }

                  float srx = sin(transformTobeMapped[0]);
                  float crx = cos(transformTobeMapped[0]);
                  float sry = sin(transformTobeMapped[1]);
                  float cry = cos(transformTobeMapped[1]);
                  float srz = sin(transformTobeMapped[2]);
                  float crz = cos(transformTobeMapped[2]);

                  int laserCloudSelNum = laserCloudOri->points.size();
                  if (laserCloudSelNum < 50)
                  {
                     continue;
                  }

                  cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
                  cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
                  cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
                  cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
                  cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
                  cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
                  for (int i = 0; i < laserCloudSelNum; i++)
                  {
                     pointOri = laserCloudOri->points[i];
                     coeff = coeffSel->points[i];

                     float arx = (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y - srx * sry * pointOri.z) * coeff.x + (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) * coeff.y + (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y - cry * srx * pointOri.z) * coeff.z;

                     float ary = ((cry * srx * srz - crz * sry) * pointOri.x + (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) * coeff.x + ((-cry * crz - srx * sry * srz) * pointOri.x + (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) * coeff.z;

                     float arz = ((crz * srx * sry - cry * srz) * pointOri.x + (-cry * crz - srx * sry * srz) * pointOri.y) * coeff.x + (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y + ((sry * srz + cry * crz * srx) * pointOri.x + (crz * sry - cry * srx * srz) * pointOri.y) * coeff.z;

                     matA.at<float>(i, 0) = arx;
                     matA.at<float>(i, 1) = ary;
                     matA.at<float>(i, 2) = arz;
                     matA.at<float>(i, 3) = coeff.x;
                     matA.at<float>(i, 4) = coeff.y;
                     matA.at<float>(i, 5) = coeff.z;
                     matB.at<float>(i, 0) = -coeff.intensity;
                  }

                  cv::transpose(matA, matAt);
                  matAtA = matAt * matA;
                  matAtB = matAt * matB;
                  cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

                  if (iterCount == 0)
                  {
                     cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
                     cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
                     cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

                     cv::eigen(matAtA, matE, matV);
                     matV.copyTo(matV2);

                     isDegenerate = false;
                     float eignThre[6] = {100, 100, 100, 100, 100, 100};
                     for (int i = 5; i >= 0; i--)
                     {
                        if (matE.at<float>(0, i) < eignThre[i])
                        {
                           for (int j = 0; j < 6; j++)
                           {
                              matV2.at<float>(i, j) = 0;
                           }
                           isDegenerate = true;
                        }
                        else
                        {
                           break;
                        }
                     }
                     matP = matV.inv() * matV2;
                  }

                  if (isDegenerate)
                  {
                     cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
                     matX.copyTo(matX2);
                     matX = matP * matX2;
                  }

                  transformTobeMapped[0] += matX.at<float>(0, 0);
                  transformTobeMapped[1] += matX.at<float>(1, 0);
                  transformTobeMapped[2] += matX.at<float>(2, 0);
                  transformTobeMapped[3] += matX.at<float>(3, 0);
                  transformTobeMapped[4] += matX.at<float>(4, 0);
                  transformTobeMapped[5] += matX.at<float>(5, 0);

                  float deltaR = sqrt(
                      pow(rad2deg(matX.at<float>(0, 0)), 2) +
                      pow(rad2deg(matX.at<float>(1, 0)), 2) +
                      pow(rad2deg(matX.at<float>(2, 0)), 2));
                  float deltaT = sqrt(
                      pow(matX.at<float>(3, 0) * 100, 2) +
                      pow(matX.at<float>(4, 0) * 100, 2) +
                      pow(matX.at<float>(5, 0) * 100, 2));

                  // if (deltaR < 0.005 && deltaT < 0.01)
                  float sumDeltaR = sqrt(
                      pow(rad2deg(transformTobeMapped[0] - iniTransformVector[0]), 2) +
                      pow(rad2deg(transformTobeMapped[1] - iniTransformVector[1]), 2) +
                      pow(rad2deg(transformTobeMapped[2] - iniTransformVector[2]), 2));

                  float sumDeltaT = sqrt(
                                        pow((transformTobeMapped[3] - iniTransformVector[3]) * 100, 2) +
                                        pow((transformTobeMapped[4] - iniTransformVector[4]) * 100, 2) +
                                        pow((transformTobeMapped[5] - iniTransformVector[5]) * 100, 2)) /
                                    100;

                  if (deltaR < 0.05 && deltaT < 0.01 || iterCount >= maxIterations - 1)
                  {
                     if (isShowCloud && TEST)
                     {
                        if (iterCount >= maxIterations - 1)
                        {
                           cout << "***********************************************************************\n";
                           cout << "Iteration = " << iterCount << "\t\t"
                                << "dR,dT = " << deltaR << "\t" << deltaT << endl;
                           cout << "***********************************************************************\n";
                        }
                        else
                           cout << "Iteration = " << iterCount << "\t\t"
                                << "dR,dT = " << deltaR << "\t" << deltaT << endl;

                        // if (sumDeltaT > 0.14 || sumDeltaR > 5.1)
                        //    continue;
                        // transformTobeMapped = iniTransformVector;

                        cout << "Iteration = " << iterCount << "\t\t"
                             << "sdR,sdT = " << sumDeltaR << "\t" << sumDeltaT << endl;
                        cout << endl;
                     }
                     errorWrite << frameID << ' ' << iterCount << ' ' << sumDeltaR << ' ' << sumDeltaT << endl;
                     int lastCount = 10;

                     // 比较预测值和计算值之间的差别
                     /* 
                  if (frameID > 370)
                  {
                     float meanDist = 0;
                     float sigmaDist = 0;
                     for (size_t i = 0; i < lastCount; i++)
                        meanDist += distLast20[i];
                     meanDist /= lastCount; //平均帧间距离

                     for (size_t i = 0; i < lastCount; i++)
                        sigmaDist += pow(distLast20[i] - meanDist, 2);
                     sigmaDist = sqrt(sigmaDist / lastCount); //帧间距离的标准差

                     float fitDist = 1.5 * distLast20[(frameID - 351) % 20] - 0.5 * distLast20[(frameID - 352) % 20]; // vt + at/2
                     cout << "meanDist = " << meanDist << "\tsigmaDist = " << sigmaDist << "\tfitDist = " << fitDist << endl;
                     if (fabs(deltaT - meanDist) > 3 * sigmaDist || deltaT > 0.1)
                     {
                        cout << "---------------------------------------------------\n";
                        fitCount ++;
                        transformTobeMapped = iniTransformVector;
                     }
                  }
                  */

                     // 循环保存前20个帧间距离
                     transformLast20[(frameID) % 20] = transformTobeMapped;
                     distLast20[(frameID) % 20] = sumDeltaT;

                     break;
                  }
               }
            }
            transformUpdate();
         }

         *framePre = *frame1;

         for (int i = 0; i < laserCloudCornerStackNum; i++)
         {
            pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);

            int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
            int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
            int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

            if (pointSel.x + 25.0 < 0)
               cubeI--;
            if (pointSel.y + 25.0 < 0)
               cubeJ--;
            if (pointSel.z + 25.0 < 0)
               cubeK--;

            if (cubeI >= 0 && cubeI < laserCloudWidth &&
                cubeJ >= 0 && cubeJ < laserCloudHeight &&
                cubeK >= 0 && cubeK < laserCloudDepth)
            {
               int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
               laserCloudCornerArray[cubeInd]->push_back(pointSel);
            }
         }

         for (int i = 0; i < laserCloudSurfStackNum; i++)
         {
            pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);

            int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
            int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
            int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

            if (pointSel.x + 25.0 < 0)
               cubeI--;
            if (pointSel.y + 25.0 < 0)
               cubeJ--;
            if (pointSel.z + 25.0 < 0)
               cubeK--;

            if (cubeI >= 0 && cubeI < laserCloudWidth &&
                cubeJ >= 0 && cubeJ < laserCloudHeight &&
                cubeK >= 0 && cubeK < laserCloudDepth)
            {
               int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
               laserCloudSurfArray[cubeInd]->push_back(pointSel);
            }
         }

         //每个空间块内的点进行滤波
         for (int i = 0; i < laserCloudValidNum; i++)
         {
            int ind = laserCloudValidInd[i];

            laserCloudCornerArray2[ind]->clear();
            downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
            downSizeFilterCorner.filter(*laserCloudCornerArray2[ind]);

            laserCloudSurfArray2[ind]->clear();
            downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
            downSizeFilterSurf.filter(*laserCloudSurfArray2[ind]);

            pcl::PointCloud<PointType>::Ptr laserCloudTemp = laserCloudCornerArray[ind];
            laserCloudCornerArray[ind] = laserCloudCornerArray2[ind];
            laserCloudCornerArray2[ind] = laserCloudTemp;

            laserCloudTemp = laserCloudSurfArray[ind];
            laserCloudSurfArray[ind] = laserCloudSurfArray2[ind];
            laserCloudSurfArray2[ind] = laserCloudTemp;
         }

         mapFrameCount++;

         if (false)
         {
            if (mapFrameCount >= mapFrameNum)
            {
               mapFrameCount = 0;

               laserCloudSurround2->clear();
               for (int i = 0; i < laserCloudSurroundNum; i++)
               {
                  int ind = laserCloudSurroundInd[i];
                  *laserCloudSurround2 += *laserCloudCornerArray[ind];
                  *laserCloudSurround2 += *laserCloudSurfArray[ind];
               }

               laserCloudSurround->clear();
               downSizeFilterCorner.setInputCloud(laserCloudSurround2);
               downSizeFilterCorner.filter(*laserCloudSurround);
            }
            // 			/*
            //  int laserCloudFullResNum = laserCloudFullRes->points.size();
            //  for (int i = 0; i < laserCloudFullResNum; i++) {
            //     pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
            //  }*/
            // ShowCloud(laserCloudSurround, viewer);
         }

         Eigen::Matrix3d Rx = Eigen::AngleAxisd(transformTobeMapped[0], Eigen::Vector3d(1, 0, 0)).toRotationMatrix();
         Eigen::Matrix3d Ry = Eigen::AngleAxisd(transformTobeMapped[1], Eigen::Vector3d(0, 1, 0)).toRotationMatrix();
         Eigen::Matrix3d Rz = Eigen::AngleAxisd(transformTobeMapped[2], Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
         Eigen::Matrix3d rot = Ry * Rx * Rz;
         Eigen::Quaterniond q(rot);

         // Eigen::Vector3d euler = rot.eulerAngles(1,0,2);
         // Eigen::Matrix3d rot2 = q.normalized().toRotationMatrix();
         // cout << endl << rot << endl;
         // cout << rot2 << endl << endl;
         // cout << euler[1] << " " << euler[0] << " " << euler[2] << endl;
         // cout << transformTobeMapped[0] << " " << transformTobeMapped[1] << " " << transformTobeMapped[2] << endl;

         fileWrite << "VERTEX_SE3:QUAT " << frameID << ' '
                   << transformTobeMapped[3] << ' ' << transformTobeMapped[4] << ' ' << transformTobeMapped[5] << ' '
                   << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w()
                   << endl;
         // cout << "VERTEX_SE3:QUAT " << frameID << ' '
         // 		  << transformTobeMapped[3] << ' ' << transformTobeMapped[4] << ' ' << transformTobeMapped[5] << ' '
         // 		  << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w()
         // 		  << endl;

         consoleProgress(int(1.0 * (frameID - startFrameNumber) / (endFrameNumber - startFrameNumber) * 76));
      }
   }
   fileWrite.close();
   errorWrite.close();
   unlink(temp_s.c_str());

   while (isShowCloud && !viewer->wasStopped() && TEST)
      viewer->spinOnce();
   return frameID;
}

void Mapping::loadMatrixFile(std::string matrixPath)
{
   ifstream matrix_file(matrixPath);
   if (!matrix_file)
   {
      cout << "Cannot read " << matrixPath << endl;
      matrix_file.close();
      exit(-1);
   }
   for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
         matrix_file >> transformation(i, j);
   matrix_file.close();

   float theta = M_PI / 2.0;
   Eigen::Matrix4f Tz = Eigen::Matrix4f::Identity(); //绕z轴转90°
   Tz(0, 0) = cos(theta);
   Tz(0, 1) = -sin(theta);
   Tz(1, 0) = sin(theta);
   Tz(1, 1) = cos(theta);

   // transformation = Tz * Tz * transformation;
   // cout << transformation << endl;
}

void Mapping::setPcapPath2(std::string pcapLoc)
{
   pcap2 = pcapLoc;
}

void Mapping::setCaliPath2(std::string caliPath)
{
   caliPath2 = caliPath;
}

void Mapping::setIsMerg2Cloud(bool isMerge2Cloud)
{
   this->isMerge2Cloud = isMerge2Cloud;
}

void Mapping::setEndFrameNumber(int number)
{
   endFrameNumber = number;
}

void Mapping::setStartFrameNumber(int number)
{
   startFrameNumber = number;
}

void Mapping::setNscans(int scans)
{
   Nscans = scans;
}

void Mapping::setNscans2(int scans)
{
   Nscans2 = scans;
}

void Mapping::setInitTraj(std::string trajPath)
{
   ifstream traj(trajPath);
   if (!traj)
   {
      // cout << "traj is not exist!" << endl;
      return;
   }

   std::string vertex;
   int id;
   double x, y, z;
   Eigen::Quaterniond qua;
   while (!traj.eof())
   {
      double theta = M_PI / 2.0;
      Eigen::Matrix4d Tx = Eigen::Matrix4d::Identity(); //绕X轴转90°
      Tx(1, 1) = cos(theta);
      Tx(1, 2) = -sin(theta);
      Tx(2, 1) = sin(theta);
      Tx(2, 2) = cos(theta);

      Eigen::Matrix4d Ty = Eigen::Matrix4d::Identity(); //绕X轴转90°
      Ty(0, 0) = cos(theta);
      Ty(0, 2) = sin(theta);
      Ty(2, 0) = -sin(theta);
      Ty(2, 2) = cos(theta);

      Eigen::Matrix4d Tz = Eigen::Matrix4d::Identity(); //绕z轴转90°
      Tz(0, 0) = cos(theta);
      Tz(0, 1) = -sin(theta);
      Tz(1, 0) = sin(theta);
      Tz(1, 1) = cos(theta);

      std::getline(traj, vertex);
      std::istringstream line(vertex);
      std::string label;
      line >> label;
      if (label != "VERTEX_SE3:QUAT")
         continue;
      line >> id >> x >> y >> z >> qua.x() >> qua.y() >> qua.z() >> qua.w();

      Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
      odom.translation() = Eigen::Vector3d(x, y, z);
      odom.linear() = qua.normalized().toRotationMatrix();
      Eigen::Matrix4d RT = odom.matrix();
      Eigen::Matrix4d T3 = Tz * Tx * Tx;
      RT = RT * Tz * Tx;
      Eigen::Matrix3d rot = Eigen::Matrix3d::Identity(); 
      for (size_t i = 0; i < 3; i++)
      {
         for (size_t j = 0; j < 3; j++)
         {
            rot(i,j) = RT(i,j);
         }
      }
      Eigen::Vector3d euler2 = rot.eulerAngles(1,0,2);
      initTraj[id][0] = euler2[1];
      initTraj[id][1] = euler2[0];
      initTraj[id][2] = euler2[2];
      initTraj[id][3] = x;
      initTraj[id][4] = y;
      initTraj[id][5] = z;
      isInitTraj[id] = true;
   }
   
}

float Mapping::isOutdoor(PointCloudReader &reader, int minFlamePointSize)
{
   PointCloud::Ptr frame(new PointCloud);
   float meanSize = 0;
   float sigmaSize = 0;
   std::vector<int> sizeIndices;
   if (endFrameNumber < 0)
      endFrameNumber = reader.getTotalFrame();
   for (int i = startFrameNumber + 100; i < endFrameNumber - 50; i += 50)
   {
      reader.frameNumber = i;
      if (reader.readPointCloud(frame, false) == -1)
         break;
      if (frame->points.size() < minFlamePointSize)
         continue;
      sizeIndices.push_back(frame->points.size());
      meanSize += sizeIndices.back();
   }
   reader.frameNumber = startFrameNumber;
   meanSize /= sizeIndices.size();
   cout << "meanSize = " << meanSize << endl;
   for (int i = 0; i < sizeIndices.size(); i++)
   {
      sigmaSize += pow(sizeIndices[i] - meanSize, 2);
   }
   sigmaSize /= sizeIndices.size();
   sigmaSize = sqrt(sigmaSize) / meanSize;
   cout << "sigmaSize = " << sigmaSize << endl;
   return sigmaSize;
}