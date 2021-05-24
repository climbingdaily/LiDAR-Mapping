#pragma once

#ifndef COMMON_H
#define COMMON_H
//----------system operation----------
#define linux 1
#ifdef linux
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#endif
#ifdef WIN32
#include <io.h>
#include <direct.h>
#endif

//-----------std headers-------------------------
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <cmath>
#include <stdlib.h>
#include <string.h>
//------------pcl headers------------------------
// #include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
// namespace pcl
// {

struct EIGEN_ALIGN16 _PointXYZIRT
{
    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    float intensity;
    uint32_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
};

/**
 * @brief A point structure representing Euclidean xyz coordinates, and the ring value.
 */
struct EIGEN_ALIGN16 PointXYZIRT : public _PointXYZIRT
{
    inline PointXYZIRT (const _PointXYZIRT &p)
    {
        x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
        intensity = p.intensity;
        ring = p.ring;
        time = p.time;
    }

    inline PointXYZIRT ()
    {
        x = y = z = 0.0f;
        data[3] = 1.0f;
        intensity = 0;
        ring = 0;
        time = 0;
    }

    inline PointXYZIRT (float _x, float _y, float _z, float _intensity, uint32_t _ring, float _time)
    {
        x = _x; y = _y; z = _z; 
        data[3] = 1.0f;
        intensity = _intensity;
        ring = _ring;
        time = _time;
    }

    friend std::ostream& operator << (std::ostream& os, const PointXYZIRT& p)
    {
        os << "(" << p.x << "," << p.y << "," << p.z << "-I-" << p.intensity << "-R-" << p.ring << "-T-" << p.time << ")";
        return (os);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// } // namespace perception

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint32_t, ring, ring)
                                  (float, time, time)
)

struct PointXYZIT
{
    PCL_ADD_POINT4D;
    union {
        struct
        {
            float intensity;
        };
        float data_c[4];
    };
    float time_stamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT, // 注册点类型宏
                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time_stamp, time_stamp))

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef Eigen::Vector3f Vector3Type;
typedef Eigen::Matrix3f Matrix3Type;
typedef Eigen::Matrix4f Matrix4Type;
typedef Eigen::RowVector3f RowVector3Type;
typedef Eigen::Quaternionf QuaternionType;
typedef float NumberType;

//------------predefine-------------------------
#define FILE_READ_ERROR -1
#define ALLOC_ERROR -1
#define NORMAL_STATUS 2
#define LEN_FILE_NAME 260
//-------------debug switch-------------------------------
#define DEBUG 1   //提交程序时设为0
#define TEST 1   //提交程序时设为0

inline double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}
/* Make temparory file */
inline int mk_temp_file(char file[])
{
    /* Return value */
    int ret = 0;
    /* Make temp file */
    if (mkstemp(file) < 0)
    {
        ret = -1;
    }
    else
    {
        //printf("Temp file name: [%s] \n", file);
    }
    return ret;
}

/* Generate genru list file */
inline int CreateTempFile(const char* tempName,const char* tempdir,std::string& outPath)
{
    /* Return value */
    int ret = 0;
    /* File name buffer */
    char file[LEN_FILE_NAME];

    /* Init */
    memset(file, 0x00, sizeof(file));
    /* Set initial value (template) */
    sprintf(file, "%s/%s.XXXXXX",tempdir,tempName);
    //printf("Temp file name: [%s] \n", file);
    /* TEMP */
    if (mk_temp_file(file) < 0)
    {
        ret = -1;
    }else{
        outPath = file;
    }

    return ret;
}
#define BUFFER_SIZE 1024
/**
 * 功能：拷贝文件函数
 * 参数：
 *      sourceFileNameWithPath：源文件名（带路径）
 *      targetFileNameWithPath：目标文件名（带路径）
 * 返回值：
 *      SUCCESS: 拷贝成功
 *      FAILURE：拷贝失败
 */
inline int copyFile(const char *sourceFileNameWithPath,
        const char *targetFileNameWithPath)
{
    FILE *fpR, *fpW;
    char buffer[BUFFER_SIZE];
    int lenR, lenW;
    if ((fpR = fopen(sourceFileNameWithPath, "r")) == NULL)
    {
        return -1;
    }
    if ((fpW = fopen(targetFileNameWithPath, "w")) == NULL)
    {
        fclose(fpR);
        return -1;
    }

    memset(buffer, 0, BUFFER_SIZE);
    while ((lenR = fread(buffer, 1, BUFFER_SIZE, fpR)) > 0)
    {
        if ((lenW = fwrite(buffer, 1, lenR, fpW)) != lenR)
        {
            fclose(fpR);
            fclose(fpW);
            return -1;
        }
        memset(buffer, 0, BUFFER_SIZE);
    }

    fclose(fpR);
    fclose(fpW);
    return 0;
}

inline void consoleProgress(int progress){
    static char bar[102] = {0};
    static std::string lable = "|/-\\";
    static int inprogress = -1;
    if (inprogress == progress )
        return;

    int st = progress/2;
    for (int i=0;i< st;i++){
         bar[i] = '#';
    }
    bar[st] = 0;

    printf("[%-50s][%d%%][%c]\r", bar, progress, lable[progress%4]);
    fflush(stdout);

    inprogress = progress;

    if (progress == 100){
        printf("\n");
    }
}

//根据给定目录，获取所有的文件名
inline int getFiles(std::string path, std::vector<std::string>& files)
{
   struct dirent *ptr;
   DIR *dir;
   dir = opendir(path.c_str());
   while ((ptr = readdir(dir)) != NULL)
   {
      //跳过'.'和'..'两个目录
      if (ptr->d_name[0] == '.')
         continue;
      //cout << ptr->d_name << endl;
      files.push_back(ptr->d_name);
   }
}

/**
 * 功能：分割字符串
 * 参数：
 *      待分割的字符串s
 *      保存的字符串集tokens
 *      分割符（只能是单个）

inline void split(const std::string& s, std::vector<std::string>& tokens, const char& delim = ' ') {
    tokens.clear();
    size_t lastPos = s.find_first_not_of(delim, 0);
    size_t pos = s.find(delim, lastPos);
    while (lastPos != std::string::npos) {
        tokens.emplace_back(s.substr(lastPos, pos - lastPos));
        lastPos = s.find_first_not_of(delim, pos);
        pos = s.find(delim, lastPos);
    }
}
 */
#endif