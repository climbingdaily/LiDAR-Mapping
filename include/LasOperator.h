
#ifndef _LAS_FILE_READER_
#define _LAS_FILE_READER_

#include <fstream>
#include <vector>
#include <string>
#include "common.h"
#pragma pack(push,1)

/*===================================================================
Structure for PUBLIC HEADER BLOCK
The fields included in the public header block are as follows:
1.     File Signature ("LASF")
2.     File Source ID
3.     Reserved (Global Encoding for v1.2 or above)
4-7.   Project ID - GUID Data 1-4
8-9.   Version Major and Minor (Major.Minor)
10.    System Identifier
11.    Generating Software
12-13. File Creation Day and Year
14.    Header Size
15.    Offset to Point Data
16.    Number of Variable Length Records
17.    Point Data Format ID (0-99 for spec)
18.    Point Data Record Length
19.    Number of Point Records
20.    Number of Points by Return
21.    X Scale Factor
22.    Y Scale Factor
23.    Z Scale Factor
24.    X Offset
25.    Y Offset
26.    Z Offset
27.    Max X
28.    Min X
29.    Max Y
30.    Min Y
31.    Max Z
32.    Min Z
===================================================================*/

struct PublicHeaderBlock{

	char fileSign[4];
	unsigned short fileSourceID;
	unsigned short reserved;    // Global Encoding for v1.2 or above
	unsigned int GUID1;
	unsigned short GUID2;
	unsigned short GUID3;
	unsigned char GUID4[8];

	unsigned char versionMajor;
	unsigned char versionMinor;
	char systemID[32];
	char GenSoft[32];
	unsigned short creationDay;
	unsigned short creationYear;

	unsigned short headerSize;
	unsigned int offsetToData;
	unsigned int varRecordNum;
	unsigned char dataFormat;
	unsigned short pointRecordLen;
	unsigned int pointRecordNum;
	unsigned int returnPointNum[5];

	double xScale;
	double yScale;
	double zScale;
	double xOffset;
	double yOffset;
	double zOffset;
	double maxX;
	double minX;
	double maxY;
	double minY;
	double maxZ;
	double minZ;
};

/*===================================================================
Structure for VARIABLE LENGTH RECORD HEADER
The fields included in the variable length record header are as follows:
1.  Reserved
2.  User ID
3.  Record ID
4.  Record Length After Header
5.  Description
===================================================================*/

//#pragma pack(8)

struct VariableLengthRecordHeader{
	unsigned short reserved;
	char userID[16];
	unsigned short recordID;
	unsigned short recordLength;
	char description[32];
};

/*===================================================================
Structure for POINT DATA RECORD FORMAT
The fields included in the point data record are as follows:
1.  X
2.  Y
3.  Z
4.  Intensity
5.  Return Number
6.  Number of Returns (given pulse)
7.  Scan Direction Flag
8.  Edge of Flight Line
9.  Classification
10. Scan Angle Rank (-90 to +90) - Left Side
11. User Data
12. Point Source ID
13. GPS Time
14. Red
15. Green
16. Blue

Notes:
POINT DATA RECORD FORMAT 0:   1-12 (20 bytes)
POINT DATA RECORD FORMAT 1:   1-13 (28 bytes)
POINT DATA RECORD FORMAT 2:   1-12,14-16 (26 bytes)
POINT DATA RECORD FORMAT 3:   1-16 (34 bytes)

===================================================================*/

struct PointDataRecord{
	int x;
	int y;
	int z;
	unsigned short intensity;
	unsigned char mask;                   // 0-2 bits: Return Number;
	// 3-5 bits: Number of Returns
	// 6 bit:    Scan Direction Flag
	// 7 bit.    Edge of Flight Line
	unsigned char classification;
	unsigned char scanAngle;
	unsigned char userData;
	unsigned short pointSourceID;
	double GPS;
	unsigned short red;
	unsigned short green;
	unsigned short blue;

};

/*===================================================================
Structure for Point3D
x, y and z correspond to the x, y and z coordination of a point.
===================================================================*/

//struct Point3D{
//	Point3D(){ }
//	Point3D(double _x,double _y,double _z):x(_x),y(_y),z(_z) { }
//	Point3D(double _x,double _y,double _z,double _i):x(_x),y(_y),z(_z),intensity(_i) { }
//	double x;
//	double y;
//	double z;
//	double intensity;
//	double scale;
//	int classification;
//};

#pragma pack(pop)

/*===================================================================
CLasOperator::saveLasFile
Save the given 3D points into a Las file.
===================================================================*/

inline bool saveLasFile(std::string file_name, PointCloud::Ptr data){
	//printf("===================file_name.c_str()\n");
	if((access(file_name.c_str(), 0 )) == 0 )
		remove(file_name.c_str());
	// Open a las file
	std::fstream lasFile;
	lasFile.open(file_name.c_str(), std::ios::out | std::ios::binary);
	if (lasFile.fail())
		return false;
	//printf("===================1\n");
	// Create PulicHeaderBlock
	PublicHeaderBlock publicHeader;
	publicHeader.fileSign[0]='L';publicHeader.fileSign[1]='A';
	publicHeader.fileSign[2]='S';publicHeader.fileSign[3]='F';
	publicHeader.fileSourceID = 0;
	publicHeader.reserved = 0;
	publicHeader.GUID1 = 0;
	publicHeader.GUID2 = 0;
	publicHeader.GUID3 = 0;
	memset(publicHeader.GUID4, 0, sizeof(unsigned char)*8);
	//printf("===================2\n");
	publicHeader.versionMajor = 1;
	publicHeader.versionMinor = 2;
	strcpy(publicHeader.systemID, "XMU");
	strcpy(publicHeader.GenSoft, "XMU Software");
	publicHeader.creationDay = 1;
	publicHeader.creationYear = 2013;
	publicHeader.headerSize = sizeof(PublicHeaderBlock);
	publicHeader.offsetToData = sizeof(PublicHeaderBlock);
	publicHeader.varRecordNum = 0;
	publicHeader.dataFormat = 3;
	publicHeader.pointRecordLen = sizeof(PointDataRecord);
	publicHeader.pointRecordNum = data->points.size();
	memset(publicHeader.returnPointNum, 0, sizeof(unsigned int)*5);
	//printf("===================3\n");
	publicHeader.maxX = -DBL_MAX;
	publicHeader.minX = DBL_MAX;
	publicHeader.maxY = -DBL_MAX;
	publicHeader.minY = DBL_MAX;
	publicHeader.maxZ = -DBL_MAX;
	publicHeader.minZ = DBL_MAX;
	for (size_t i = 0; i<publicHeader.pointRecordNum; ++i){
		if (data->points[i].x>publicHeader.maxX)
			publicHeader.maxX = data->points[i].x;
		if (data->points[i].x<publicHeader.minX)
			publicHeader.minX = data->points[i].x;
		if (data->points[i].y>publicHeader.maxY)
			publicHeader.maxY = data->points[i].y;
		if (data->points[i].y<publicHeader.minY)
			publicHeader.minY = data->points[i].y;
		if (data->points[i].z>publicHeader.maxZ)
			publicHeader.maxZ = data->points[i].z;
		if (data->points[i].z<publicHeader.minZ)
			publicHeader.minZ = data->points[i].z;
	}
	//printf("===================4\n");
	double scaleX = publicHeader.maxX - publicHeader.minX>DBL_EPSILON ? publicHeader.maxX - publicHeader.minX : DBL_EPSILON;
	double scaleY = publicHeader.maxY - publicHeader.minY>DBL_EPSILON ? publicHeader.maxY - publicHeader.minY : DBL_EPSILON;
	double scaleZ = publicHeader.maxZ - publicHeader.minZ>DBL_EPSILON ? publicHeader.maxZ - publicHeader.minZ : DBL_EPSILON;
	publicHeader.xScale = 1.0e-9 * scaleX;
	publicHeader.yScale = 1.0e-9 * scaleY;
	publicHeader.zScale = 1.0e-9 * scaleZ;
	publicHeader.xOffset = publicHeader.minX;
	publicHeader.yOffset = publicHeader.minY;
	publicHeader.zOffset = publicHeader.minZ;
	//printf("===================5\n");
	// Write pulbic header block
	lasFile.write((char *)(&publicHeader), sizeof(PublicHeaderBlock));

	unsigned short cColor = 0;
	// Export data points
	for (size_t i = 0; i<data->points.size(); ++i){
		PointDataRecord pointRecord;
		pointRecord.x = static_cast<int>((data->points[i].x - publicHeader.xOffset) / publicHeader.xScale);
		pointRecord.y = static_cast<int>((data->points[i].y - publicHeader.yOffset) / publicHeader.yScale);
		pointRecord.z = static_cast<int>((data->points[i].z - publicHeader.zOffset) / publicHeader.zScale);
		pointRecord.intensity =  (unsigned short)(std::min<double>(data->points[i].intensity * 256,65535.0));
		pointRecord.mask = 0;
		pointRecord.classification = 0;
		pointRecord.scanAngle = 0;
		pointRecord.userData = 0;
		pointRecord.pointSourceID = 0;
		pointRecord.GPS = 1.0 * data->points[i].data_n[0] + data->points[i].data_n[1];
		//printf("Point timeStamp0=%f timeStamp1=%f timeStamp2=%f\n",data->points[i].data_c[0] , data->points[i].data_c[1], data->points[i].data_c[2]);
		pointRecord.red = 0;// pointRecord.intensity ;
		pointRecord.green = 0;// pointRecord.intensity ;
		pointRecord.blue = 0;//pointRecord.intensity ;
		lasFile.write((char *)&pointRecord, sizeof(PointDataRecord));
	}
	//printf("6\n");
	lasFile.close();
	return true;
}
#endif
