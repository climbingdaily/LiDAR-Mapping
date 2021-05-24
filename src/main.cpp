#include "add_timestamp.h"
#include "common.h"
#include "loop_detector.h"
#include "make_horizonal.h"
#include "mapping.h"
#include "optimize_end.h"
#include "parameter_reader.h"
#include <iostream>
#include <sys/stat.h>
#include <unistd.h>

using namespace std;
//ParameterReader para("./parameters.txt");
///************************************************************************/
///* 判断文件夹是否存在                                                   */
///************************************************************************/


std::string getExePath()
{
   char szBuf[256];
   ssize_t temp;
   memset(szBuf, 0x00, sizeof(szBuf));
   temp = readlink("/proc/self/exe", szBuf, sizeof(szBuf)-1);
   std::string exepath = szBuf;
   return exepath.substr(0,exepath.rfind('/'));
}

inline int CreateDir(const char *sPathName)
{
   char DirName[256];
   strcpy(DirName, sPathName);
   int i, len = strlen(DirName);
   if (DirName[len - 1] != '/')
      strcat(DirName, "/");
   len = strlen(DirName);
   for (i = 1; i < len; i++)
   {
      if (DirName[i] == '/' || DirName[i] == '\\')
      {
         DirName[i] = 0;
         if (access(DirName, 0) != 0)
         {
            if (mkdir(DirName, 0755) == -1)
            {
               perror("mkdir   error");
               return -1;
            }
         }
         DirName[i] = '/';
      }
   }

   return 0;
}

inline bool CheckFileExist(const char *p)
{
   /* Check for existence */
   if ((access(p, 0)) == 0)
   {
      return true;
   }
   return false;
}

int main(int argc, char **argv)
{

   //ParameterReader para("./parameters.txt");
   std::string exepath = getExePath();
   std::string pcapFile = "";                       // para.getData("fileNamePcap");
   std::string pcap2File = "";                      // para.getData("fileNamePcap");
   std::string calibFile = exepath + "/resource/VLP-32c.xml"; //para.getData("calibrationPath");
   std::string calibFile2 = exepath + "/resource/VLP-16.xml"; //para.getData("calibrationPath");
   std::string outputFolder = exepath + "/output";           //para.getData("outputFolder");
   std::string initTraj = "";                       //初始的轨迹，可用于建图时作为起始点
   std::fstream timeWrite;
   int nscans = 32;
   int nscans2 = 32;
   int startFrameNumber = 0;
   int endFrameNumber = -1;
   int resolution = 3; //cm
   int frameGap = 1;
   bool isMerg2Cloud = false;
   bool isShow = false;
   //Exe传入参数
   for (int i = 1; i < argc; i++)
   {
      const char *currArg = argv[i];
      if (strcmp(currArg, "--help") == 0 || strcmp(currArg, "-h") == 0)
      {
         printf("-f\t设置PCAP文件路径F！\n");
         printf("-f2\t可选。设置第二个PCAP文件路径M！\n");
         printf("-o\t可选。设置输出文件路径O。默认为可执行文件路径下的./output文件夹！\n");
         printf("-d\t设置主激光头的硬件类型。可选 0/1/2 三个参数。\t0：VLP-16\t1：HDL-32\t2:VLP-32c\n");
         printf("-d2\t设置第二激光头的硬件类型。可选 0/1/2 三个参数。\t0：VLP-16\t1：HDL-32\t2:VLP-32c\n");
         printf("-b\t可选。设置建图起始帧数B。(填数字，默认0）\n");
         printf("-e\t可选。设置建图结束的帧数E。(填数字，默认end）\n");
         printf("-g\t可选。设置每G帧进行一次建图。可选1/2/3。(默认1)\n");
         // printf("-r\t设置建图密度。默认3cm(输入整数)!\n");
         printf("-s\t可选。建图时可视化点云\n");
         cout << "样例输入：" << endl;
         cout << "./mapping -f \"a.pcap\" -m \"b.pcap\" -o ./here -d 2 -b 100 -e 10000 -g 1 -r 3 -s" << endl;
         // system("pause");
         return 0;
      }
      if (strcmp(currArg, "-F") == 0 || strcmp(currArg, "-f") == 0)
      {
         i++;
         if (i >= argc)
         {
            printf("设置PCAP文件路径参数错误！\n");
            return -1;
         }
         pcapFile = argv[i];
      }
      else if (strcmp(currArg, "-F2") == 0 || strcmp(currArg, "-f2") == 0)
      {
         i++;
         isMerg2Cloud = true;
         if (i >= argc)
         {
            printf("设置第二个PCAP文件路径参数错误！\n");
            return -1;
         }
         pcap2File = argv[i];
      }
      else if (strcmp(currArg, "-P") == 0 || strcmp(currArg, "-p") == 0)
      {
         i++;
         if (i >= argc)
         {
            printf("设置第二个PCAP文件路径参数错误！\n");
            return -1;
         }
         initTraj = argv[i];
      }
      else if (strcmp(currArg, "-O") == 0 || strcmp(currArg, "-o") == 0)
      {
         i++;
         if (i >= argc)
         {
            printf("设置输出路径参数错误！");
            return -1;
         }
         outputFolder = argv[i];
      }
      else if (strcmp(currArg, "-D") == 0 || strcmp(currArg, "-d") == 0)
      {
         i++;
         if (i >= argc)
         {
            printf("设置数据类型错误！");
            return -1;
         } 
         if (strcmp(argv[i], "2") == 0)
         {
            nscans = 32;
            calibFile = exepath + "/resource/VLP-32c.xml";
         }
         else if (strcmp(argv[i], "1") == 0)
         {
            nscans = 32;
            calibFile = exepath + "/resource/HDL-32.xml";
         }
         else if (strcmp(argv[i], "0") == 0)
         {
            nscans = 16;
            calibFile = exepath + "/resource/VLP-16.xml";
         }
         else
         {
            printf("设置数据类型错误！");
            return -1;
         }
      }
      else if (strcmp(currArg, "-D2") == 0 || strcmp(currArg, "-d2") == 0)
      {
         i++;
         if (i >= argc)
         {
            printf("设置数据类型错误！");
            return -1;
         } 
         if (strcmp(argv[i], "2") == 0)
         {
            nscans2 = 32;
            calibFile2 = exepath + "/resource/VLP-32c.xml";
         }
         else if (strcmp(argv[i], "1") == 0)
         {
            nscans2 = 32;
            calibFile2 = exepath + "/resource/HDL-32.xml";
         }
         else if (strcmp(argv[i], "0") == 0)
         {
            nscans2 = 16;
            calibFile2 = exepath + "/resource/VLP-16.xml";
         }
         else
         {
            printf("设置数据类型错误！");
            return -1;
         }
      }
      else if (strcmp(currArg, "-B") == 0 || strcmp(currArg, "-b") == 0)
      {
         i++;
         if (i >= argc)
         {
            printf("设置起始帧数错误！");
            return -1;
         }
         startFrameNumber = std::atoi(argv[i]);
         if (startFrameNumber < -1)
         {
            printf("设置起始帧数错误！(必须大于-1)");
            return -1;
         }
      }
      else if (strcmp(currArg, "-E") == 0 || strcmp(currArg, "-e") == 0)
      {
         i++;
         if (i >= argc)
         {
            printf("设置结束帧数错误！");
            return -1;
         }
         endFrameNumber = std::atoi(argv[i]);
         if (endFrameNumber <= startFrameNumber || endFrameNumber < 1)
         {
            printf("设置起始帧数错误！(必须大于起始帧数)");
            return -1;
         }
      }
      else if (strcmp(currArg, "-G") == 0 || strcmp(currArg, "-g") == 0)
      {
         i++;
         if (i >= argc)
         {
            printf("设置建图频数错误！");
            return -1;
         }
         frameGap = std::atoi(argv[i]);
         if (frameGap < 1 || frameGap > 3)
         {
            printf("设置建图频数错误！(须在1-3之间)");
            return -1;
         }
      }
      else if (strcmp(currArg, "-R") == 0 || strcmp(currArg, "-r") == 0)
      {
         i++;
         if (i >= argc)
         {
            printf("设置建图密度错误！");
            return -1;
         }
         resolution = std::atoi(argv[i]);
         if (resolution <= 0 || resolution > 10)
         {
            printf("设置建图密度错误！(范围在1-10cm)");
            return -1;
         }
      }
      else if (strcmp(currArg, "-S") == 0 || strcmp(currArg, "-s") == 0)
      {
         // if (TEST)
            isShow = true;
      }
      else
      {
         pcapFile = argv[i];
      }
   }

   if (!CheckFileExist(pcapFile.c_str()))
   {
      printf("设置PCAP文件路径参数错误！\n");
      return -1;
   }

   if (!CheckFileExist(pcap2File.c_str()) && isMerg2Cloud)
   {
      printf("设置PCAP2文件路径参数错误！\n");
      return -1;
   }

   //Create output Folder
   //CreateDir(outputFolder.c_str());
   std::string pcapFileName = pcapFile.substr(pcapFile.rfind('/') + 1);
   pcapFileName = pcapFileName.substr(0, pcapFileName.find('.'));
   //output folder + pcapFileName
   if (true)
   {
      outputFolder += "/" + pcapFileName + "_" + to_string(startFrameNumber) + "_to_";
      if(endFrameNumber>0)
         outputFolder += to_string(endFrameNumber);
      else
         outputFolder += "end";
   }
   else
      outputFolder = outputFolder + "/" + pcapFileName;
   //Create new output Folder
   CreateDir(outputFolder.c_str());

   std::string outputTraj; //=  outputFolder + "/trajectory.txt";
   std::string outputTrajWithTimeStamp = outputFolder + "/traj_with_timestamp.txt";
   std::string outputMap = outputFolder;
   std::string outputPrefaceTrj = outputFolder + "/pre_trajectory.txt";
   std::string outputTime = outputFolder + "/timeInfo.txt";
   //Create output map Folder
   CreateDir(outputMap.c_str());

   std::string calibMatrix = exepath + "/resource/autoCalibMatrix.txt"; // only bag system need
   std::string tempDir = exepath + "/tmp";
   CreateDir(tempDir.c_str());

   bool isShowCloud = false;        //atoi(para.getData("showCloud").c_str())==1 ? true : false;
   bool isOutputPrefaceTrj = false; //atoi(para.getData("outputPrefaceTrj").c_str())==1 ? true : false;
   if (DEBUG)
      isOutputPrefaceTrj = true;

   std::cout << "\n\n================================parameter===================================" << endl
             << "pcap File=" << pcapFile << endl;
   if (isMerg2Cloud)
      std::cout << "pcap2 File=" << pcap2File << endl;
   std::cout << "output Traj=" << outputTrajWithTimeStamp << endl
             << "output Map=" << outputMap << endl
             << "Nscans=" << nscans << endl
             << "==============================================================================" << endl;

   // std::string tempTraj = "trajectory.txt";
   std::string tempTraj = outputFolder + "/trajectory.txt";
   std::string gpsFile = outputFolder + "/GPGGA.txt";

   std::string temploopDetectResult = "loop_detect_result.txt";
   std::string tempoutputTraj = "loop_detect_result.txt";
   //create temp file
   //下面这句是为了程序奔溃时也能看结果
   if (CheckFileExist(tempTraj.c_str()))
   {
      remove(tempTraj.c_str());
   }
   if (
      // CreateTempFile(tempTraj.c_str(), tempDir.c_str(), tempTraj) == -1 ||
       CreateTempFile(temploopDetectResult.c_str(), tempDir.c_str(), temploopDetectResult) == -1 ||
       CreateTempFile(tempoutputTraj.c_str(), tempDir.c_str(), outputTraj) == -1)
   {
      std::cout << "Cannot create temporary files" << endl;
      return -1;
   }

   // time counter
   TimeCounter t;
   t.begin();
   timeWrite.open(outputTime.c_str(), std::ios::out | std::ios::app); //记录运行时间
   timeWrite << "开始记录时间  " << pcapFile << endl;
   //----------------------------------------------------------------
   //            0. Mapping
   //----------------------------------------------------------------
   if (CheckFileExist(gpsFile.c_str()))
   {
      unlink(gpsFile.c_str());      
   }
   Mapping mapping;
   mapping.setStartFrameNumber(startFrameNumber); //default 0
   mapping.setEndFrameNumber(endFrameNumber);     // fefault -1
   mapping.setNscans(nscans);                     //default 32
   mapping.setSkipFrameNumber(frameGap);          //default 3
   mapping.setInitTraj(initTraj);
   mapping.setGPSFile(gpsFile);
   mapping.setVisualization(isShow);
   if (isMerg2Cloud)
   {
      //mapping.setTransformaiton();
      mapping.setIsMerg2Cloud(isMerg2Cloud); // default false
      mapping.setNscans2(nscans2);                //default 16
      mapping.setCaliPath2(calibFile2);
      mapping.setPcapPath2(pcap2File);
      mapping.loadMatrixFile(calibMatrix);
   }
   int total_frame = mapping.run(tempTraj.c_str(), pcapFile, calibFile);
   if (total_frame == -1)
   {
      unlink(tempTraj.c_str());
      unlink(temploopDetectResult.c_str());
      return -2;
   }
   endFrameNumber = endFrameNumber > 0 ? endFrameNumber : total_frame;
   timeWrite << "FrameNumber\t" << total_frame << endl;
   timeWrite << "TimePerFrame\t" << t.getFinishTime() / (endFrameNumber - startFrameNumber) << endl;
   timeWrite << t.record("Mapping") << endl;

   //----------------------------------------------------------------
   //    3. use RANSAC to fit plain of trajectory, then make the trajectory horizonal
   //       the result will override the previous trajectory
   //-----------------------------------------------------------
   
   MakeHorizonal mH;
   mH.setFrameGap(frameGap); //一定要在1,3之间
   mH.SetTotalFrame(total_frame);
   mH.SetInputFile(tempTraj);
   mH.SetStartDistance(10.0);
   mH.SetEndDistance(30.0);
   mH.Run();
   //copy
   if (isOutputPrefaceTrj)
      copyFile(tempTraj.c_str(), outputPrefaceTrj.c_str());
   timeWrite << t.record("MakeHorizonal") << endl;
   bool isloop = false;
   if (isloop)
   {

      //----------------------------------------------------------------
      //            1. detect loop in track by distance
      //----------------------------------------------------------------
      LoopDetector loopDetector;

      // input for loop_detector
      loopDetector.SetPcapFilePath(pcapFile);
      loopDetector.SetCalibFilePath(calibFile);
      loopDetector.SetFrontResultPath(tempTraj.c_str());

      // if two pose distance < detect_dis, we think loop maybe happen
      loopDetector.SetDetectDistance(6);

      // the resolution of two maps for calculating constraint (use gicp)
      loopDetector.SetDetectMapRes(0.05);

      // if two submap register error > max_error, we judge that they are not loop
      loopDetector.SetMaxIcpError(5);

      // input for optimizer
      loopDetector.SetOutputFile(temploopDetectResult.c_str()); // temp file, need to delete

      loopDetector.Run();
      //----------------------------------------------------------------

      //----------------------------------------------------------------
      //          2. use new constraint to optimize trajectory
      //----------------------------------------------------------------
      OptimizeEnd optimizeEnd;

      // input for optimize_end
      optimizeEnd.SetLoopResultFilename(temploopDetectResult.c_str()); // input from optimizer
      optimizeEnd.SetOutputFile(outputTraj);

      // max iteration step for optimizer
      optimizeEnd.SetOptimizeIterations(20);
      optimizeEnd.setExePath(exepath);
      optimizeEnd.Run();

      timeWrite << t.record("loopDetector") << endl;
      //----------------------------------------------------------------
      unlink(temploopDetectResult.c_str());
      unlink(tempTraj.c_str());

      MakeHorizonal mH2;
      mH2.setFrameGap(frameGap); //一定要在1,3之间
      mH2.SetTotalFrame(total_frame);
      mH2.SetInputFile(outputTraj); //轨迹
      // mH.SetStartDistance(10.0);
      // mH.SetEndDistance(30.0);
      mH2.Run();
      timeWrite << t.record("MakeHorizonal") << endl;
   }
   else
   {
      copyFile(tempTraj.c_str(), outputTraj.c_str());
   }


   //----------------------------------------------------------------
   //          4.  build the map
   //----------------------------------------------------------------
   BuildMapfromG2oVertex buildMap;

   PointCloud::Ptr map(new PointCloud);
   // input for map_builder
   buildMap.setCalibPath(calibFile);
   buildMap.setPcapFilePath(pcapFile);
   buildMap.SetTrackFilename(outputTraj); //轨迹
   buildMap.setOutputPath(outputMap);
   buildMap.setSaveMap(true);
   buildMap.setPrintFlag(true);
   buildMap.setShowCloud(isShowCloud);
   buildMap.setMapPointer(map);
   buildMap.setDistanceLimit(50);
   buildMap.setExePath(exepath);

   if (isMerg2Cloud)
   {
      buildMap.setCalibrationFlag(true);
      buildMap.setPointCloudReader2(pcap2File, calibFile2);
      buildMap.setCalibrationMatrixPath(calibMatrix);
   }
   buildMap.setMapResolution(0.01 * resolution);
   buildMap.Run();

   timeWrite << t.record("buildMap") << endl;

   //----------------------------------------------------------------
   //     5.  add the timestamp from pcap to the trajectory file
   //----------------------------------------------------------------
   std::string outputOdom = outputFolder + "/odom";
   AddTimeStamp add_ts;
   CreateDir(outputOdom.c_str());
   // input for add_timestamp
   add_ts.setPcapFile(pcapFile);
   add_ts.setCalibFile(calibFile);
   add_ts.setInputTrack(outputTraj); //轨迹
   add_ts.setOutputTrack(outputTrajWithTimeStamp);
   add_ts.setOutputFolder(outputOdom);
   add_ts.Run();
   unlink(temploopDetectResult.c_str());
   unlink(outputTraj.c_str());
   // unlink(gpsFile.c_str());      
   unlink(tempTraj.c_str());
   //----------------------------------------------------------------

   // output time consume
   timeWrite << t.record("AddTimeStamp") << endl;
   timeWrite << "TimePerFrame\t" << t.getFinishTime() / total_frame << endl;
   timeWrite << t.end() << endl;
   std::cout << "运行结束" << endl << t.end() << endl;
   // unlink(outputTime.c_str());
   unlink(outputPrefaceTrj.c_str());
   return 0;
}